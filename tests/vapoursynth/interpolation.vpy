"""Phase-4 specification acceptance; no legacy oracle."""
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
    src = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), [10, 21, 40][n]) for y in range(8) for x in range(8)])
    clip = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), [100, 110, 120][n]) for y in range(8) for x in range(8)])
    clip = core.std.SetFrameProps(clip, marker=[7, 9], data=b"a\0b", real=[1.5, -0.0], _SceneChangeNext=9,
                                  _DurationNum=[91, 92], _DurationDen=93)
    clip = edit(clip, lambda n, f: f.props.__setitem__("ForeignNode", src))
    sup = super_clip(src)
    bw, fw = vectors(1), vectors(-1)
    def absent(node):
        return core.std.RemoveFrameProps(node, props=[P + "AnalysisVectors"])
    # Main fields complete, extras unavailable; metadata-only frame zero is admitted.
    B = edit(bw, lambda n, f: f.props.__delitem__(P + "AnalysisVectors") if n == 1 else None)
    F = edit(fw, lambda n, f: f.props.__delitem__(P + "AnalysisVectors") if n == 0 else None)
    inter = mv.FlowInter(clip, sup, [B, F])
    constant(inter, 0, 15)
    constant(inter, 1, 115)
    constant(inter, 2, 120)
    constant(mv.FlowInter(clip, sup, [bw, fw]), 0, 15)  # Extra mode, zero masks.
    constant(mv.FlowInter(clip, sup, [absent(bw), fw], blend=False), 1, 110)
    constant(mv.FlowInter(clip, sup, [absent(bw), fw], blend=-2), 1, 115)
    constant(mv.FlowInter(clip, sup, [bw, fw], time=0), 0, 10)
    constant(mv.FlowInter(clip, sup, [bw, fw], time=100), 0, 21)
    with inter.get_frame(0) as out, clip.get_frame(0) as original:
        assert set(out.props) == set(original.props)
        for key in original.props:
            if key != "ForeignNode":
                assert out.props[key] == original.props[key], key
        assert out.props.ForeignNode.num_frames == 3

    bad = vectors(-1, sad=-1)
    fails(lambda: mv.FlowInter(clip, sup, [absent(bw), bad]).get_frame(0))
    fails(lambda: mv.FlowInter(clip, sup, [bw, bad], time=0).get_frame(0))
    # Out-of-range selection must precede even corrupt main arrays.
    constant(mv.FlowInter(clip, sup, [vectors(1, sad=-1), bad]), 2, 120)
    corrupt_extra = edit(bw, lambda n, f: f.props.__setitem__(P + "AnalysisSAD", [-1]) if n == 1 else None)
    fails(lambda: mv.FlowInter(clip, sup, [corrupt_extra, fw]).get_frame(0))
    seen = []
    def broken_super(n, f):
        seen.append(n)
        if n != 0:
            raise vs.Error("temporal dependency sentinel")
    broken = edit(sup, broken_super)
    constant(mv.FlowInter(clip, broken, [absent(bw), fw]), 1, 115)
    assert seen == [0], seen
    fails(lambda: mv.FlowInter(clip, broken, [bw, fw]).get_frame(0), "dependency sentinel")

    fps = mv.FlowFPS(clip, sup, [B, F], num=0)
    assert (fps.num_frames, fps.fps_num, fps.fps_den) == (6, 48000, 1001)
    for n, value in enumerate([100, 15, 110, 115, 120, 120]):
        constant(fps, n, value)
        with fps.get_frame(n) as f:
            assert f.props._DurationNum == 1001 and f.props._DurationDen == 48000
            assert list(f.props.marker) == [7, 9]
    corrupt = [vectors(1, sad=-1), vectors(-1, sad=-1)]
    constant(mv.FlowFPS(clip, sup, corrupt, num=0), 2, 110)
    constant(mv.FlowFPS(clip, broken, corrupt, num=0), 2, 110)
    fails(lambda: mv.FlowFPS(clip, sup, corrupt, num=0).get_frame(1))
    constant(mv.FlowFPS(clip, sup, [corrupt_extra, fw], num=0, extramask=False), 1, 15)
    fails(lambda: mv.FlowFPS(clip, sup, [corrupt_extra, fw], num=0).get_frame(1))
    source1 = core.std.AssumeFPS(clip, fpsnum=1, fpsden=1)
    endpoint = mv.FlowFPS(source1, sup, corrupt, num=1000)
    constant(endpoint, 999, 110)  # t=256 retains the right endpoint.
    constant(endpoint, 2999, 120)
    for num, den in [(0, 1), (60, 0), (0, 0)]:
        node = mv.FlowFPS(clip, sup, [bw, fw], num=num, den=den)
        assert (node.fps_num, node.fps_den, node.num_frames) == (48000, 1001, 6)

    for amount in [0, 50, 200]:
        blur = mv.FlowBlur(clip, sup, [bw, fw], blur=amount)
        for n, expected in enumerate([100, 21, 120]):
            constant(blur, n, expected)
    constant(mv.FlowBlur(clip, broken, [absent(bw), fw]), 1, 110)
    fails(lambda: mv.FlowBlur(clip, broken, [bw, fw], blur=0).get_frame(1), "dependency sentinel")
    fails(lambda: mv.FlowBlur(clip, sup, [absent(bw), bad]).get_frame(1))
    constant(mv.FlowBlur(clip, sup, corrupt), 0, 100)
    constant(mv.FlowBlur(clip, sup, [bw, fw], prec=(1 << 63)-1), 1, 21)
    # Only the selected current Super is used; both vector nodes are asymmetric in time.
    forward_at2 = edit(fw, lambda n, f: f.props.__setitem__(P + "AnalysisSAD", [-1]) if n == 1 else None)
    constant(mv.FlowBlur(clip, sup, [bw, forward_at2]), 1, 21)

    for fmt in [vs.GRAY10, vs.GRAY16, vs.GRAYS, vs.YUV420P8, vs.YUV422P16, vs.YUV440P8, vs.YUV444PS]:
        info = core.get_video_format(fmt)
        floating = info.sample_type == vs.FLOAT
        first = [0.25, -0.125, 0.375] if floating else [10, 30, 50]
        second = [0.5, 0.125, -0.125] if floating else [21, 41, 61]
        source = blank(color=first[:info.num_planes], format=fmt, width=16, height=16)
        def fill(n, f):
            for p in range(info.num_planes):
                a = f[p]
                for y in range(a.shape[0]):
                    for x in range(a.shape[1]):
                        a[y, x] = (second if n else first)[p]
        source = edit(source, fill)
        visible = blank(color=[0.0625 if floating else 100]*info.num_planes, format=fmt, width=16, height=16)
        pair = [vectors(1, width=16, height=16), vectors(-1, width=16, height=16)]
        supernode = super_clip(source)
        for function in [lambda: mv.FlowInter(visible, supernode, pair),
                         lambda: mv.FlowFPS(visible, supernode, pair, num=0)]:
            out = function()
            for p in range(info.num_planes):
                constant(out, 0 if out.num_frames == 3 else 1,
                         (first[p]+second[p])/2 if floating else (first[p]+second[p])//2, p)
        for p in range(info.num_planes):
            constant(mv.FlowBlur(visible, supernode, pair), 1, second[p], p)

    floating = blank(color=1.75 + 2**-23, format=vs.GRAYS)
    fs = super_clip(floating)
    constant(mv.FlowInter(floating, fs, [absent(bw), fw], time=85*100/256), 2, 1.75+2**-22)
    nanclip = edit(floating, lambda n, f: f[0].__setitem__((0, 0), math.nan))
    # Copy endpoint preserves nonfinite image data; arithmetic does not.
    with mv.FlowFPS(nanclip, fs, [bw, fw], num=0).get_frame(0) as f:
        assert math.isnan(f[0][0, 0])
    fails(lambda: mv.FlowInter(nanclip, fs, [absent(bw), fw], time=0).get_frame(2))
    gradient = edit(blank(), lambda n, f: [f[0].__setitem__((y, x), 30+4*x+8*y+16*n) for y in range(8) for x in range(8)])
    gs = super_clip(gradient, pel=2)
    pair = [vectors(1, pel=2, motion=(0, 0)), vectors(-1, pel=2, motion=(-1, 0))]
    with mv.FlowInter(clip, gs, pair).get_frame(0) as f:
        assert f[0][3, 3] == 73  # A=64 from -1 pel, C=82; integer floor mean.
    for fn, kwargs in [(mv.FlowInter, {"time":101}), (mv.FlowInter, {"ml":0}),
                       (mv.FlowFPS, {"num":-1}), (mv.FlowBlur, {"blur":201}),
                       (mv.FlowBlur, {"prec":0}), (mv.FlowInter, {"thscd2":math.nan})]:
        fails(lambda: fn(clip, sup, [bw, fw], **kwargs))
    for pair in [[fw, bw], [bw], [bw, fw, bw], [vectors(0), vectors(0)]]:
        fails(lambda: mv.FlowInter(clip, sup, pair))
    # Interleaved requests exercise independent per-output staged state.
    jobs = [(node, n) for n in [2, 0, 1, 2, 1, 0] for node in [inter, fps]]
    futures = [node.get_frame_async(n) for node, n in jobs]
    for future in futures:
        future.result().close()
    print("neo-mv phase-4 temporal VS acceptance passed")


run()
gc.collect()
blank(length=1).set_output()
