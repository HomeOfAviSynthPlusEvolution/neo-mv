"""Phase-3 Flow acceptance from approved specs, without a legacy oracle."""
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


def fails(call, fragment=""):
    try:
        call()
    except vs.Error as error:
        assert fragment.lower() in str(error).lower(), str(error)
        return
    raise AssertionError("expected a controlled Flow error: " + fragment)


def samples(frame, plane=0):
    view = frame[plane]
    return [view[y, x] for y in range(view.shape[0]) for x in range(view.shape[1])]


def constant(node, n, value, plane=0):
    with node.get_frame(n) as frame:
        assert all(x == value for x in samples(frame, plane)), (n, plane, value)


def vectors(delta=1, motion=(0, 0), sad=0, width=8, height=8, pel=1, pad=4,
            bits=8, prefix=P, length=3):
    names = dict(Width=width, Height=height, RealWidth=width, RealHeight=height,
                 HPad=pad, VPad=pad, Pel=pel, Levels=1, Chroma=0,
                 XRatioUV=1, YRatioUV=1, BlkSizeX=8, BlkSizeY=8,
                 OverlapX=0, OverlapY=0, NBlkX=width//8, NBlkY=height//8,
                 DeltaFrame=delta, BitsPerSample=bits)
    props = {prefix + "Analysis" + k: v for k, v in names.items()}
    packed = (motion[0] & 0xffffffff) | ((motion[1] & 0xffffffff) << 32)
    if packed >= 1 << 63:
        packed -= 1 << 64
    count = width//8 * (height//8)
    props[prefix + "AnalysisVectors"] = [packed] * count
    props[prefix + "AnalysisSAD"] = [sad] * count
    return core.std.SetFrameProps(blank(format=vs.GRAY16, width=4, height=4, length=length), **props)


def super_clip(source, pel=1, pad=4, **kwargs):
    return mv.Super(source, blksize=[8], overlap=[0], pad=[pad], pel=pel, sharp=0, **kwargs)


def run():
    source = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), [80, 100, 140][n])
                                       for y in range(8) for x in range(8)])
    sup = super_clip(source)
    clip = core.std.SetFrameProps(blank(), marker=[7, 9], data=b"a\0b", real=[1.5, -0.0],
                                  _SceneChangeNext=9, **{P + "AnalysisVectors": [987]})
    clip = edit(clip, lambda n, f: f.props.__setitem__("ForeignNode", source))
    field = vectors()
    outputs = [mv.Flow(clip, sup, field, time=t) for t in [0, 50, 100]]
    jobs = [(out, n) for n in [2, 0, 1, 2, 1, 0] for out in outputs]
    futures = [out.get_frame_async(n) for out, n in jobs]
    for (_, n), future in zip(jobs, futures):
        with future.result() as result, clip.get_frame(n) as original:
            assert samples(result) == [[100]*64, [140]*64, [5]*64][n]
            assert set(result.props) == set(original.props)
            for key in original.props:
                if key != "ForeignNode":
                    assert result.props[key] == original.props[key], key
            assert result.props.ForeignNode.num_frames == 3
    assert (outputs[0].width, outputs[0].height, outputs[0].num_frames,
            outputs[0].fps_num, outputs[0].fps_den) == (8, 8, 3, 24000, 1001)
    constant(mv.Flow(clip, sup, vectors(0)), 2, 140)
    constant(mv.Flow(clip, sup, field, fields=True), 1, 140)  # pel=1 is inactive.

    missing = core.std.RemoveFrameProps(field, props=[P + "AnalysisVectors"])
    invalid = edit(field, lambda n, f: f.props.__setitem__(P + "AnalysisWidth", 0) if n == 1 else None)
    for unavailable in [missing, invalid, vectors(sad=401)]:
        constant(mv.Flow(clip, sup, unavailable), 1, 5)
    # Fallback has no current/reference Super dependency after creation.
    seen = []
    def broken(n, f):
        seen.append(n)
        if n != 0:
            raise vs.Error("Flow Super dependency sentinel")
    failing = edit(sup, broken)
    constant(mv.Flow(clip, failing, missing), 1, 5)
    assert seen == [0], seen
    fails(lambda: mv.Flow(clip, failing, field).get_frame(1), "dependency sentinel")
    corrupt_current = edit(sup, lambda n, f: f.props.__delitem__(P + "NeoMVSuperPlanesV1") if n == 1 else None)
    constant(mv.Flow(clip, corrupt_current, field), 1, 140)
    fails(lambda: mv.Flow(clip, sup, vectors(sad=-1)).get_frame(2), "malformed analysis")
    changed = edit(missing, lambda n, f: f.props.__setitem__(P + "AnalysisDeltaFrame", 0) if n == 1 else None)
    fails(lambda: mv.Flow(clip, sup, changed).get_frame(1), "metadata changed")
    levels = edit(field, lambda n, f: f.props.__setitem__(P + "AnalysisLevels", 50))
    constant(mv.Flow(clip, sup, levels), 1, 140)

    gradient = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), 16*n + 4*x + 8*y)
                                          for y in range(8) for x in range(8)])
    linear = super_clip(gradient, pel=2)
    # floor((vx*128+128)/256): -3 -> -1, +3 -> +2 pel units.
    for vx, expected in [(-3, 66), (3, 72)]:
        with mv.Flow(clip, linear, vectors(pel=2, motion=(vx, 0)), time=50).get_frame(1) as f:
            assert f[0][3, 3] == expected, (vx, f[0][3, 3])
    half = vectors(pel=2)
    for time in [0, 100]:
        fails(lambda: mv.Flow(clip, linear, half, fields=True, time=time).get_frame(1), "_Field")
    constant(mv.Flow(clip, linear, half, fields=True), 2, 5)
    for tff, expected in [(True, 64), (False, 72)]:
        with mv.Flow(clip, linear, half, fields=True, tff=tff).get_frame(1) as f:
            assert f[0][3, 3] == expected, (tff, f[0][3, 3])
    with mv.Flow(clip, linear, half, fields=True, tff=True, time=50).get_frame(0) as f:
        assert f[0][3, 3] == 56
    marked = edit(linear, lambda n, f: f.props.__setitem__("_Field", [int(n % 2 == 0), 99]))
    with mv.Flow(clip, marked, half, fields=True).get_frame(1) as f:
        assert f[0][3, 3] == 64
    corrupt = edit(marked, lambda n, f: f.props.__delitem__(P + "NeoMVSuperPlanesV1") if n == 1 else None)
    fails(lambda: mv.Flow(clip, corrupt, half, fields=True).get_frame(1))
    with mv.Flow(clip, corrupt, half, fields=True, tff=True).get_frame(1) as f:
        assert f[0][3, 3] == 64

    # AnalysisChroma=false still processes every render plane. Precision is independent.
    for fmt in [vs.GRAY10, vs.GRAY16, vs.GRAYS, vs.YUV420P8, vs.YUV422P16, vs.YUV440P8, vs.YUV444PS]:
        info = core.get_video_format(fmt)
        floating = info.sample_type == vs.FLOAT
        color = [0.25, -0.125, 0.375] if floating else [80, 90, 100]
        color = color[:info.num_planes]
        src = blank(color=color, format=fmt, width=16, height=16)
        vis = blank(color=[0.0625 if floating else 5]*info.num_planes, format=fmt, width=16, height=16)
        out = mv.Flow(vis, super_clip(src), vectors(width=16, height=16))
        for p in range(info.num_planes):
            constant(out, 1, color[p], p)

    yuv = blank(color=[80, 90, 100], format=vs.YUV420P8, width=16, height=16)
    vis = blank(color=[5]*3, format=vs.YUV420P8, width=16, height=16)
    small_pad = super_clip(yuv, pel=2, pad=3)
    edge = vectors(width=16, height=16, pel=2, pad=3, motion=(-6, 0))
    admitted = mv.Flow(vis, small_pad, edge)  # Only zero displacement is checked at creation.
    fails(lambda: admitted.get_frame(1), "logical phase domain")
    constant(mv.Flow(vis, small_pad, edge, time=0), 1, 90, 1)
    constant(mv.Flow(vis, super_clip(yuv, pel=2),
                     vectors(width=16, height=16, pel=2, motion=(-6, 0))), 1, 90, 1)
    for kwargs in [dict(time=-1), dict(time=101), dict(time=math.nan), dict(thscd1=-1),
                   dict(thscd2=math.inf)]:
        fails(lambda: mv.Flow(clip, sup, field, **kwargs))
    fails(lambda: mv.Flow(clip, sup, vectors(length=2)))
    custom = super_clip(source, prefix="Other")
    constant(mv.Flow(clip, custom, vectors(prefix="Other"), prefix="Other"), 1, 140)
    print("neo-mv phase-3 Flow VS acceptance passed")


run()
gc.collect()
blank(length=1).set_output()
