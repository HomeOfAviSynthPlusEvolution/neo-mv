"""Phase-2 host acceptance using approved formulas, without a legacy oracle."""
import gc
import math
import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neo_mv
P = "MVUtensils"


def blank(color=5, format=vs.GRAY8, width=8, height=8, length=3):
    return core.std.BlankClip(width=width, height=height, format=format, length=length,
                              color=color, fpsnum=24000, fpsden=1001)


def edit(clip, callback):
    def select(n, f):
        out = f.copy()
        callback(n, out)
        return out
    return core.std.ModifyFrame(clip, clip, select)


def fails(call, fragment):
    try:
        call()
    except vs.Error as error:
        assert fragment.lower() in str(error).lower(), str(error)
        return
    raise AssertionError("expected controlled error: " + fragment)


def samples(frame, plane=0):
    v = frame[plane]
    return [v[y, x] for y in range(v.shape[0]) for x in range(v.shape[1])]


def constant(node, n, value, plane=0):
    with node.get_frame(n) as f:
        assert all(x == value for x in samples(f, plane)), (n, plane, samples(f, plane)[:8], value)


def vectors(delta, sad=0, packed=0, width=8, height=8, pel=1, pad=4, chroma=False, bits=8,
            prefix=P, length=3):
    # The carrier is deliberately a different size and precision, with no Super payload.
    names = dict(Width=width, Height=height, RealWidth=width, RealHeight=height,
                 HPad=pad, VPad=pad, Pel=pel, Levels=1, Chroma=chroma,
                 XRatioUV=2 if chroma else 1, YRatioUV=2 if chroma else 1,
                 BlkSizeX=8, BlkSizeY=8, OverlapX=0, OverlapY=0,
                 NBlkX=width//8, NBlkY=height//8, DeltaFrame=delta, BitsPerSample=bits)
    props = {prefix + "Analysis" + k: int(v) for k, v in names.items()}
    count = width//8 * (height//8)
    props[prefix + "AnalysisVectors"] = [packed] * count
    props[prefix + "AnalysisSAD"] = [sad] * count
    return core.std.SetFrameProps(blank(format=vs.GRAY16, width=4, height=4, length=length), **props)


def super_clip(source, **kwargs):
    return mv.Super(source, blksize=[8], overlap=[0], pad=[4], pel=1, **kwargs)


def run():
    source = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), [80, 100, 140][n])
                                       for y in range(8) for x in range(8)])
    super = super_clip(source)
    clip = core.std.SetFrameProps(blank(), marker=[7, 9], data=b"a\0b", real=[1.5, 2.5],
                                  **{P + "AnalysisVectors": [987], "_SceneChangeNext": 9})
    clip = edit(clip, lambda n, f: f.props.__setitem__("ForeignNode", source))
    a, b = vectors(-1), vectors(1)
    low = mv.Compensate(clip, super, vectors(1, 99), thsad=100)
    high = mv.Compensate(clip, super, vectors(1, 100), thsad=100)
    constant(low, 1, 140)
    constant(high, 1, 100)
    constant(low, 2, 5)
    constant(mv.Compensate(clip, super, b, time=0), 1, 140)
    analysed = mv.Analyse(super, search=0, chroma=False)
    constant(mv.Compensate(clip, super, analysed, thscd1=16320), 1, 140)
    constant(mv.Compensate(clip, super, vectors(0)), 2, 140)
    ramp = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), x + 10*n)
                                      for y in range(8) for x in range(8)])
    shifted = mv.Compensate(clip, super_clip(ramp), vectors(0, packed=1))
    with shifted.get_frame(1) as f:
        assert samples(f) == [11, 12, 13, 14, 15, 16, 17, 17]*8
    with low.get_frame(1) as f, clip.get_frame(1) as original:
        assert set(f.props) == set(original.props)
        for key in original.props:
            if key != "ForeignNode":
                assert f.props[key] == original.props[key], key
        assert f.props.ForeignNode.num_frames == source.num_frames
    assert (low.width, low.height, low.num_frames, low.fps_num, low.fps_den) == (8, 8, 3, 24000, 1001)

    outputs = [mv.Degrain(clip, super, [a, b]), mv.Degrain1(clip, super, [a, b])]
    jobs = [(out, n) for n in [2, 0, 1, 2, 1, 0] for out in outputs]
    futures = [out.get_frame_async(n) for out, n in jobs]
    for (_, n), future in zip(jobs, futures):
        with future.result() as f:
            assert samples(f) == [[90]*64, [107]*64, [120]*64][n]
            assert f.props.marker == [7, 9]
    constant(mv.Degrain1(clip, super, [a, b], limit=[2.2]), 1, 102)
    constant(mv.Degrain(clip, super, [a, b], weights=[0, 0, 0]), 1, 100)
    constant(mv.Degrain(clip, super, [a, b], planes=[1]), 1, 5)
    for radius in range(1, 26):
        members = [vectors(d) for j in range(1, radius+1) for d in [-j, j]]
        named = getattr(mv, "Degrain" + str(radius))(clip, super, members, weights=[0]*(2*radius+1))
        constant(named, 1, 100)

    # Missing or mismatched arrays and invalid metadata are unavailable, even
    # when the wrong-sized property contains an opaque host object.
    missing = core.std.RemoveFrameProps(b, props=[P + "AnalysisVectors"])
    opaque = edit(b, lambda n, f: f.props.__setitem__(P + "AnalysisVectors", [source, source]))
    invalid_later = edit(b, lambda n, f: f.props.__setitem__(P + "AnalysisWidth", 0) if n == 1 else None)
    for field in [missing, opaque, invalid_later, vectors(1, 401)]:
        constant(mv.Compensate(clip, super, field), 1, 5)
    absent_a = core.std.RemoveFrameProps(a, props=[P + "AnalysisVectors"])
    constant(mv.Degrain(clip, super, [absent_a, missing]), 1, 100)
    constant(mv.Degrain(clip, super, [a, missing]), 1, 90)

    # Required and excluded references share one Super node. Only n=2 fails;
    # creation may legitimately inspect n=0, and current n=1 stays usable.
    seen = []
    def broken(n, f):
        seen.append(n)
        if n == 2:
            raise vs.Error("selected reference sentinel")
    failing = edit(super, broken)
    constant(mv.Compensate(clip, failing, missing), 1, 5)
    assert 2 not in seen
    fails(lambda: mv.Compensate(clip, failing, b).get_frame(1), "selected reference sentinel")
    fails(lambda: mv.Degrain(clip, failing, [a, b], weights=[0, 0, 0]).get_frame(1),
          "selected reference sentinel")
    corrupt_current = edit(super, lambda n, f: f.props.__delitem__(P + "NeoMVSuperPlanesV1") if n == 1 else None)
    fails(lambda: mv.Compensate(clip, corrupt_current, missing).get_frame(1), "missing required property")
    corrupt = vectors(1, -1)
    fails(lambda: mv.Compensate(clip, super, corrupt).get_frame(2), "malformed analysis")
    fails(lambda: mv.Degrain(clip, super, [a, corrupt], weights=[0, 0, 0]).get_frame(1), "malformed analysis")
    changed = edit(b, lambda n, f: f.props.__setitem__(P + "AnalysisDeltaFrame", 0) if n == 1 else None)
    fails(lambda: mv.Compensate(clip, super, changed).get_frame(1), "metadata changed")
    levels = edit(b, lambda n, f: f.props.__setitem__(P + "AnalysisLevels", 50))
    constant(mv.Compensate(clip, super, levels), 1, 140)

    # Precision separation, chroma plane selection and subpixel field parity.
    for fmt in [vs.YUV420P8, vs.YUV420P16, vs.YUV420PS]:
        floating = fmt == vs.YUV420PS
        colors = [[0.125, -0.25, 0.375], [0.25, -0.125, 0.5], [0.375, 0, 0.625]] if floating else [[80]*3, [100]*3, [140]*3]
        src = core.std.Splice([blank(color=c, format=fmt, width=16, height=16, length=1) for c in colors])
        visible = blank(color=[0.0625]*3 if floating else [5]*3, format=fmt, width=16, height=16)
        sup = super_clip(src)
        va, vb = [vectors(d, width=16, height=16) for d in [-1, 1]]
        out = mv.Degrain(visible, sup, [va, vb], planes=[1])
        constant(out, 1, -0.125 if floating else 107, 1)
        constant(out, 1, 0.0625 if floating else 5, 0)
        constant(out, 1, 0.0625 if floating else 5, 2)
        constant(mv.Compensate(visible, sup, vb), 1, colors[2][1], 1)

    half = mv.Super(source, blksize=[8], overlap=[0], pad=[4], pel=2)
    half_b = vectors(1, pel=2)
    fails(lambda: mv.Compensate(clip, half, half_b, fields=True).get_frame(1), "_Field")
    constant(mv.Compensate(clip, half, half_b, fields=True, tff=False), 1, 140)
    constant(mv.Compensate(clip, half, half_b, fields=True), 2, 5)
    parity = edit(half, lambda n, f: f.props.__setitem__("_Field", [n % 2, 99]))
    constant(mv.Compensate(clip, parity, half_b, fields=True), 1, 140)
    gradient = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), 16*n + 4*x + 8*y)
                                           for y in range(8) for x in range(8)])
    linear = mv.Super(gradient, blksize=[8], overlap=[0], pad=[4], pel=2, sharp=0)
    partial = mv.Compensate(clip, linear, vectors(1, packed=0xfffffffd, pel=2), time=50)
    with partial.get_frame(1) as f:
        # vx=-3 -> trunc(-3*128/256)=-1: reference at x=2.5, y=3.
        assert f[0][3, 3] == 16*2 + 4*2 + 2 + 8*3
    for top, expected in [(False, 56), (True, 48)]:
        out = mv.Compensate(clip, linear, half_b, fields=True, tff=top, thsad=0)
        with out.get_frame(1) as f:
            assert f[0][3, 3] == expected
    for invert, expected in [(False, 56), (True, 48)]:
        marked = edit(linear, lambda n, f, invert=invert: f.props.__setitem__("_Field", int(bool(n % 2) != invert)))
        out = mv.Compensate(clip, marked, half_b, fields=True, thsad=0)
        with out.get_frame(1) as f:
            assert f[0][3, 3] == expected
    fails(lambda: mv.Compensate(clip, super, b, fields=True), "field mode")
    for kwargs, message in [(dict(thsad=-1), "nonnegative"), (dict(time=101), "invalid"),
                            (dict(time=math.nan), "invalid")]:
        fails(lambda: mv.Compensate(clip, super, b, **kwargs), message)
    for kwargs, message in [(dict(weights=[]), "2R+1"), (dict(planes=[0, 0]), "duplicate"),
                            (dict(planes=[3]), "plane index"), (dict(limit=[0]), "positive"),
                            (dict(thsad=[1, 2, 3]), "at most two"), (dict(thsad2=[-1]), "nonnegative")]:
        fails(lambda: mv.Degrain(clip, super, [a, b], **kwargs), message)
    fails(lambda: mv.Degrain2(clip, super, [a, b]), "exactly 2R")
    fails(lambda: mv.Degrain(clip, super, [b, b]), "opposite")
    constant(mv.Degrain(clip, super, [a, b], thsad=[], thsad2=[], limit=[], planes=[]), 1, 107)
    custom = mv.Super(source, blksize=[8], overlap=[0], pad=[4], pel=1, prefix="Other")
    constant(mv.Compensate(clip, custom, vectors(1, prefix="Other"), prefix="Other"), 1, 140)
    print("neo-mv phase-2 VS acceptance passed")


run()
gc.collect()
blank(length=1).set_output()
