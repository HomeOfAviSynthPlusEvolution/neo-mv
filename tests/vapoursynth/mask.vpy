"""Phase-3 mask acceptance from approved specifications, without a legacy oracle."""
import gc
import math

import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neomv
P = "MVUtensils"


def blank(length=3):
    return core.std.BlankClip(width=4, height=4, format=vs.GRAY16, length=length,
                              color=4321, fpsnum=24000, fpsden=1001)


def edit(clip, callback):
    def select(n, f):
        out = f.copy()
        callback(n, out)
        return out
    return core.std.ModifyFrame(clip, clip, select)


def fails(call):
    try:
        call()
    except vs.Error:
        return
    raise AssertionError("expected a controlled VapourSynth error")


def pack(x, y):
    value = (x & 0xffffffff) | ((y & 0xffffffff) << 32)
    return value if value < (1 << 63) else value - (1 << 64)


def vectors(bits=8, delta=1, grid=(1, 1), block=(8, 8), overlap=(0, 0),
            actual=None, working=None, motion=None, sad=None, prefix=P, length=3):
    nx, ny = grid
    bx, by = block
    ox, oy = overlap
    coverage = (nx * (bx - ox) + ox, ny * (by - oy) + oy)
    width, height = working or coverage
    real_width, real_height = actual or coverage
    names = dict(Width=width, Height=height, RealWidth=real_width, RealHeight=real_height,
                 HPad=8, VPad=8, Pel=1, Levels=1, Chroma=0, XRatioUV=1, YRatioUV=1,
                 BlkSizeX=bx, BlkSizeY=by, OverlapX=ox, OverlapY=oy,
                 NBlkX=nx, NBlkY=ny, DeltaFrame=delta, BitsPerSample=bits)
    props = {prefix + "Analysis" + key: value for key, value in names.items()}
    props[prefix + "AnalysisVectors"] = [pack(x, y) for x, y in (motion or [(0, 0)] * (nx * ny))]
    props[prefix + "AnalysisSAD"] = sad if sad is not None else [0] * (nx * ny)
    # Deliberately distinct carrier format, dimensions, pixels and properties.
    props.update(marker=[7, 9], data=b"mask\0carrier", real=[1.5, -0.0],
                 _SceneChangePrev=1, _Matrix=1, _ColorRange=0)
    return core.std.SetFrameProps(blank(length), **props)


def samples(frame):
    view = frame[0]
    return [view[y, x] for y in range(view.shape[0]) for x in range(view.shape[1])]


def check_frame(frame, expected):
    assert set(frame.props) == {"_Range"}, dict(frame.props)
    assert frame.props["_Range"] == 1
    actual = samples(frame)
    if isinstance(expected, list):
        assert actual == expected, (actual, expected)
    else:
        assert all(value == expected for value in actual), (actual, expected)


def constant(node, expected, n=1):
    with node.get_frame(n) as frame:
        check_frame(frame, expected)


def precision_and_properties():
    for bits, fmt in [(8, vs.GRAY8), (10, vs.GRAY10), (16, vs.GRAY16), (32, vs.GRAYS)]:
        field = vectors(bits=bits, motion=[(4, 0)])
        foreign = blank(1)
        field = edit(field, lambda n, f, foreign=foreign: f.props.__setitem__("ForeignNode", foreign))
        magnitude = mv.VectorLengthMask(field, ml=8, gamma=2)
        assert magnitude.format.id == fmt
        assert (magnitude.width, magnitude.height, magnitude.num_frames,
                magnitude.fps_num, magnitude.fps_den) == (8, 8, 3, 24000, 1001)
        expected = 0.25 if bits == 32 else ((1 << bits) - 1) // 4
        constant(magnitude, expected)
        constant(mv.VectorLengthMask(field, ml=8, gamma=2, time=0), expected)
        constant(magnitude, expected, 2)  # n + delta is outside the clip, but is irrelevant.
        shifted_sad = 8 << (min(bits, 16) - 8)
        score = mv.SADMask(vectors(bits=bits, sad=[shifted_sad]), ml=1)
        constant(score, 0.5 if bits == 32 else ((1 << bits) - 1) // 2)
        constant(mv.OcclusionMask(field, gamma=0), 0)  # No neighbors means no event.
        missing = core.std.RemoveFrameProps(field, props=[P + "AnalysisSAD"])
        for function in [mv.VectorLengthMask, mv.SADMask, mv.OcclusionMask]:
            fallback = 1.25 if bits == 32 else 20.5
            constant(function(missing, scval=fallback), 1.25 if bits == 32 else 21)
    constant(mv.SADMask(vectors(bits=10, sad=[8]), ml=1), 127)
    constant(mv.VectorLengthMask(vectors(length=1, delta=2147483647), gamma=0), 255, 0)
    constant(mv.VectorLengthMask(vectors(length=1, delta=-2147483648), gamma=0), 255, 0)
    custom = vectors(prefix="Other", motion=[(4, 0)])
    constant(mv.VectorLengthMask(custom, ml=8, gamma=2, prefix="Other"), 63)
    fails(lambda: mv.VectorLengthMask(custom))


def fixed_score_and_resizing_examples():
    field = vectors(grid=(2, 1), block=(4, 4), delta=-1, motion=[(4, 0), (0, 0)])
    left_row = [255, 255, 223, 159, 96, 32, 0, 0]
    right_row = [0, 0, 32, 96, 159, 223, 255, 255]
    out = mv.OcclusionMask(field, ml=80)
    with out.get_frame(0) as frame:
        check_frame(frame, left_row * 4)
    positive = vectors(grid=(2, 1), block=(4, 4), delta=1, motion=[(4, 0), (0, 0)])
    constant(mv.OcclusionMask(positive, ml=80), 255)
    with mv.OcclusionMask(positive, ml=80, time=0).get_frame(1) as frame:
        check_frame(frame, right_row * 4)
    constant(mv.OcclusionMask(field, ml=80, time=0), 255)
    empty = vectors(grid=(2, 1), block=(4, 4), delta=-1, motion=[(8, 0), (0, 0)])
    constant(mv.OcclusionMask(empty, ml=80), 0)
    zero_delta = vectors(grid=(2, 1), block=(4, 4), delta=0, motion=[(4, 0), (0, 0)])
    with mv.OcclusionMask(zero_delta, ml=80).get_frame(1) as frame:
        check_frame(frame, left_row * 4)
    cropped = vectors(grid=(2, 1), block=(4, 4), delta=-1, actual=(6, 4),
                      motion=[(4, 0), (0, 0)])
    with mv.OcclusionMask(cropped, ml=80).get_frame(1) as frame:
        check_frame(frame, left_row[:6] * 4)

    # Exact grids [0,0,255] and [0,255,255], from the approved projection example.
    projected = vectors(grid=(3, 1), block=(4, 4), motion=[(0, 0), (4, 0), (0, 0)], sad=[0, 4, 8])
    rows = [(0, [0, 0, 0, 0, 0, 0, 32, 96, 159, 223, 255, 255]),
            (100, [0, 0, 32, 96, 159, 223, 255, 255, 255, 255, 255, 255])]
    for time, row in rows:
        with mv.SADMask(projected, ml=1, time=time).get_frame(1) as frame:
            check_frame(frame, row * 4)
    floating = vectors(bits=32, grid=(2, 1), block=(4, 4), motion=[(0, 0), (4, 0)])
    with mv.VectorLengthMask(floating, ml=4, gamma=2).get_frame(1) as frame:
        check_frame(frame, [0, 0, 0.125, 0.375, 0.625, 0.875, 1, 1] * 4)


def input_rules():
    source = vectors(motion=[(4, 0)])
    missing = core.std.RemoveFrameProps(source, props=[P + "AnalysisVectors"])
    wrong_size = core.std.SetFrameProps(source, **{P + "AnalysisVectors": [0, 0]})
    wrong_type = core.std.SetFrameProps(source, **{P + "AnalysisVectors": [0.0]})
    # Both lengths are considered before type checks, so the missing SAD array
    # takes precedence over the available wrong-typed vector array.
    unavailable_wrong_type = core.std.RemoveFrameProps(wrong_type, props=[P + "AnalysisSAD"])
    invalid_later = edit(source, lambda n, f: f.props.__setitem__(P + "AnalysisWidth", 0) if n == 1 else None)
    changed_later = edit(missing, lambda n, f: f.props.__setitem__(P + "AnalysisDeltaFrame", 2) if n == 1 else None)
    levels = edit(source, lambda n, f: f.props.__setitem__(P + "AnalysisLevels", n + 1))
    for name in ["VectorLengthMask", "SADMask", "OcclusionMask"]:
        function = getattr(mv, name)
        for unavailable in [missing, wrong_size, unavailable_wrong_type, invalid_later]:
            constant(function(unavailable, scval=12.5), 13)
        constant(function(missing, scval=-1), 0)
        fails(lambda: function(changed_later).get_frame(1))
        fails(lambda: function(wrong_type).get_frame(1))
        # Creation only needs scalar metadata; malformed arrays fail on output.
        corrupt = function(vectors(sad=[-1]))
        fails(lambda: corrupt.get_frame(2))
        constant(function(vectors(sad=[401]), scval=9), 9)
        constant(function(vectors(sad=[400]), gamma=0),
                 0 if name == "OcclusionMask" else 255)
        with function(levels).get_frame(1) as frame:
            check_frame(frame, 0 if name != "VectorLengthMask" else 10)
        for kwargs in [dict(ml=0), dict(ml=math.inf), dict(gamma=-1), dict(time=-1), dict(time=101),
                       dict(scval=math.nan), dict(scval=255.5), dict(thscd1=-1), dict(thscd2=101)]:
            fails(lambda: function(source, **kwargs))
        fails(lambda: function(core.std.RemoveFrameProps(source, props=[P + "AnalysisWidth"])))
        fails(lambda: function(vectors(actual=(9, 8), working=(16, 8))))
        fails(lambda: function(vectors(working=(7, 8), actual=(7, 8))))
    seen = []
    def upstream_failure(n, f):
        seen.append(n)
        if n == 1:
            raise vs.Error("mask dependency sentinel")
    failing = edit(source, upstream_failure)
    fails(lambda: mv.VectorLengthMask(failing).get_frame(1))
    assert 1 in seen


def interleaved_requests():
    source = edit(vectors(), lambda n, f: f.props.__setitem__(P + "AnalysisVectors", [pack(2 * n, 0)]))
    first = mv.VectorLengthMask(source, ml=8, gamma=2)
    second = mv.VectorLengthMask(source, ml=4, gamma=2)
    jobs = [(node, n, values[n]) for n in [2, 0, 1, 2, 1, 0]
            for node, values in [(first, [0, 15, 63]), (second, [0, 63, 255])]]
    futures = [node.get_frame_async(n) for node, n, _ in jobs]
    for (_, _, expected), future in zip(jobs, futures):
        with future.result() as frame:
            check_frame(frame, expected)


precision_and_properties()
fixed_score_and_resizing_examples()
input_rules()
interleaved_requests()
print("neo-mv phase-3 mask VS acceptance passed")
gc.collect()
blank(1).set_output()
