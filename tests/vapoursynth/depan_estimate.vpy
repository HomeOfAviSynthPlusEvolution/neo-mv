"""Phase-6 public specification acceptance using independent synthetic images."""
import gc
import math
import struct
import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neomv
selected = mv.KernelInfo()
assert selected["backend"] in ("scalar", "highway") and selected["target"], selected
MOTION = ("Depan_dx", "Depan_dy", "Depan_rot", "Depan_zoom", "Depan_goodmotion")
REMOVED = ("DepanEstimateFFT", "DepanEstimateFFT2", "DepanEstimateX", "DepanEstimateY",
           "DepanEstimateZoom", "DepanEstimateGood", "DepanEstimateTrust")
RESET = (0.0, 0.0, 0.0, 1.0, 0)


def blank(format=vs.GRAY8, width=4, height=4, length=3, color=0):
    return core.std.BlankClip(format=format, width=width, height=height, length=length,
                              color=color, fpsnum=24000, fpsden=1001)


def edit(node, callback):
    def select(n, f):
        out = f.copy()
        callback(n, out)
        return out
    return core.std.ModifyFrame(node, node, select)


def fails(call, fragment=""):
    try:
        call()
    except vs.Error as error:
        assert fragment.lower() in str(error).lower(), str(error)
        return
    raise AssertionError("expected a controlled DepanEstimate error: " + fragment)


def estimate(node, **kwargs):
    args = dict(winx=4, winy=4, wleft=0, wtop=0, dxmax=1, dymax=1, stab=0)
    args.update(kwargs)
    return mv.DepanEstimate(node, **args)


def pixels(frame):
    return [bytes(frame[p]) for p in range(frame.format.num_planes)]


def motion(node, n):
    with node.get_frame(n) as frame:
        return tuple(frame.props[key] for key in MOTION)


def close_motion(actual, expected):
    assert actual[4] == expected[4], (actual, expected)
    for value, reference in zip(actual[:4], expected[:4]):
        assert math.isfinite(value) and abs(value - reference) <= 1e-5, (actual, expected)


def copy_pixels(actual, original, n):
    with actual.get_frame(n) as a, original.get_frame(n) as b:
        assert pixels(a) == pixels(b), n


def impulse(format=vs.GRAY8, width=4, height=4, length=3, left=0, top=0,
            amplitude=1, moving=True):
    fmt = core.get_video_format(format)
    color = 0 if fmt.num_planes == 1 else [0, 17, 29]
    def draw(n, f):
        f[0][top, left + (2 - n % 3 if moving else 2)] = amplitude
    return edit(blank(format, width, height, length, color), draw)


def basic_and_properties():
    source = impulse()
    result = estimate(source)
    assert motion(result, 0) == RESET
    for n in (1, 2):
        close_motion(motion(result, n), (1, 0, 0, 1, 1))
    assert (result.width, result.height, result.num_frames, result.fps_num, result.fps_den) == (4, 4, 3, 24000, 1001)
    assert result.format.id == source.format.id

    foreign = blank(length=1)
    inherited = dict(marker=[7, 9], payload=b"a\0b", real=[1.5, -0.0],
                     DepanEstimate_info=b"inherited", DepanEstimateFFT_extra=b"keep",
                     _ColorRange=1, _Matrix=1, _ChromaLocation=0, _SceneChangePrev=1)
    source = core.std.SetFrameProps(source, **inherited)
    def replace(n, f):
        values = (b"untrusted spectrum", [1, 2], [1.0, 2.0], foreign, b"x", -7, 0.5)
        for key, value in zip(REMOVED, values):
            f.props[key] = value
        for key in MOTION:
            f.props[key] = b"wrong inherited type"
        f.props["ForeignNode"] = foreign
    source = edit(source, replace)
    result = estimate(source)
    for n in (2, 0, 1):
        with result.get_frame(n) as out, source.get_frame(n) as original:
            assert pixels(out) == pixels(original)
            assert set(out.props) == set(original.props) - set(REMOVED)
            for key, value in inherited.items():
                assert out.props[key] == original.props[key], key
            assert out.props.ForeignNode.num_frames == 1
            assert all(isinstance(out.props[key], float) for key in MOTION[:4])
            assert isinstance(out.props.Depan_goodmotion, int)
            assert all(key in original.props for key in REMOVED)
            assert original.props.Depan_dx == b"wrong inherited type"
    # Replacing inherited arrays must leave one scalar per output property.
    arrays = core.std.SetFrameProps(impulse(), **{key: [1, 2, 3] for key in MOTION})
    close_motion(motion(estimate(arrays), 2), (1, 0, 0, 1, 1))
    return result


def formats_and_samples():
    formats = [core.query_video_format(vs.GRAY, vs.INTEGER, bits, 0, 0).id for bits in (9, 10, 16)]
    formats += [vs.GRAY8, vs.GRAYS, vs.YUV420P8, vs.YUV422P10, vs.YUV444P16,
                core.query_video_format(vs.YUV, vs.INTEGER, 8, 0, 1).id]
    for format in formats:
        source = impulse(format=format)
        result = estimate(source)
        close_motion(motion(result, 2), (1, 0, 0, 1, 1))
        copy_pixels(result, source, 2)
    for bits in (9, 10, 16):
        format = core.query_video_format(vs.GRAY, vs.INTEGER, bits, 0, 0).id
        source = impulse(format=format, amplitude=(1 << bits) - 1)
        close_motion(motion(estimate(source), 2), (1, 0, 0, 1, 1))
        if bits == 16:
            continue  # Every uint16 storage value is nominally valid at 16 bits.
        source = impulse(format=format, width=8, height=6, left=2, top=1)
        outside = edit(source, lambda n, f: f[0].__setitem__((0, 0), 1 << bits))
        result = estimate(outside, wleft=2, wtop=1)
        copy_pixels(result, outside, 1)
        inside = edit(source, lambda n, f: f[0].__setitem__((1, 2), 1 << bits))
        fails(lambda: estimate(inside, wleft=2, wtop=1).get_frame(0))

    source = impulse(format=vs.GRAYS, width=8, height=6, left=2, top=1)
    nan_bytes = struct.pack("=I", 0x7fc12345)
    nan_value = struct.unpack("=f", nan_bytes)[0]
    outside = edit(source, lambda n, f: f[0].__setitem__((0, 0), nan_value))
    with outside.get_frame(0) as frame:
        assert bytes(frame[0])[:4] == nan_bytes
    result = estimate(outside, wleft=2, wtop=1)
    close_motion(motion(result, 1), (1, 0, 0, 1, 1))
    copy_pixels(result, outside, 1)
    for value in (nan_value, math.inf, -math.inf):
        inside = edit(source, lambda n, f: f[0].__setitem__((1, 2), value))
        fails(lambda: estimate(inside, wleft=2, wtop=1).get_frame(0))
    # Float chroma is copied and never inspected as numerical input.
    yuv = core.query_video_format(vs.YUV, vs.FLOAT, 32, 0, 0).id
    color = blank(format=yuv, color=[0, 0, 0])
    color = edit(color, lambda n, f: (f[0].__setitem__((0, 2 - n), 1.0),
                                     f[1].__setitem__((0, 0), nan_value)))
    copy_pixels(estimate(color), color, 1)
    odd = impulse(height=3)
    close_motion(motion(estimate(odd, winy=3, dymax=0), 2), (1, 0, 0, 1, 1))
    copy_pixels(estimate(odd, winy=3, dymax=0), odd, 2)


def parity_and_dependencies():
    source = impulse(moving=False, length=5)
    for n in (0, 2):
        fails(lambda: estimate(source, fields=True, trust=100).get_frame(n))
    one = impulse(length=1)
    fails(lambda: estimate(one, fields=True).get_frame(0))
    assert motion(estimate(one, fields=True, tff=False), 0) == RESET
    for top_first in (False, True):
        result = estimate(source, fields=True, tff=top_first)
        assert motion(result, 0) == RESET
        for n in (1, 2, 3):
            top = bool(top_first) ^ bool(n & 1)
            close_motion(motion(result, n), (0, 1 if top else -1, 0, 1, 1))
    parity = core.std.SetFrameProps(source, _Field=0)
    close_motion(motion(estimate(parity, fields=True), 2), (0, -1, 0, 1, 1))
    close_motion(motion(estimate(parity, fields=True, tff=True, pixaspect=2), 2), (0, .5, 0, 1, 1))
    # Output 2 requires basic parity at 1,2,3, but frame 0 is only a predecessor.
    predecessor_only = edit(parity, lambda n, f: f.props.__delitem__("_Field") if n == 0 else None)
    close_motion(motion(estimate(predecessor_only, fields=True), 2), (0, -1, 0, 1, 1))
    for missing in (1, 2, 3):
        incomplete = edit(parity, lambda n, f: f.props.__delitem__("_Field") if n == missing else None)
        fails(lambda: estimate(incomplete, fields=True, trust=100).get_frame(2))
        assert motion(estimate(incomplete, fields=True, tff=True, trust=100), 2) == RESET
    bad_type = core.std.SetFrameProps(source, _Field=b"not parity")
    fails(lambda: estimate(bad_type, fields=True).get_frame(1))
    close_motion(motion(estimate(bad_type, fields=False), 1), (0, 0, 0, 1, 1))
    # Errors from any required source frame survive current-record rejection.
    for broken in (0, 1, 2, 3):
        def fail_selected(n, f):
            if n == broken:
                raise vs.Error("estimate required input sentinel")
        fragile = edit(source, fail_selected)
        fails(lambda: estimate(fragile, trust=100).get_frame(2), "required input sentinel")


def display():
    constant = blank(color=0)
    assert motion(estimate(constant), 0) == RESET
    fails(lambda: estimate(constant, show=True).get_frame(0))
    fails(lambda: estimate(constant, show=True).get_frame(1))
    source = impulse(format=vs.YUV444P8, width=8, height=6, left=2, top=1)
    shown = estimate(source, wleft=2, wtop=1, show=True)
    plain = estimate(source, wleft=2, wtop=1)
    for n in (0, 1, 2):
        assert motion(shown, n) == motion(plain, n)
        with shown.get_frame(n) as out, source.get_frame(n) as original:
            for p in (1, 2):
                assert bytes(out[p]) == bytes(original[p])
            for y in range(6):
                for x in range(8):
                    if 2 <= x < 6 and 1 <= y < 5:
                        expected = 255 if (x, y) == (2 if n == 0 else 3, 1) else 0
                        assert out[0][y, x] == expected, (n, x, y, out[0][y, x])
                    else:
                        assert out[0][y, x] == original[0][y, x]
    # Two independent surfaces are placed half a clip width apart.
    source = blank(width=12, height=4)
    def pair(n, f):
        for left, amplitude in ((1, 1), (7, 10)):
            f[0][0, left + 2 - n] = amplitude
    source = edit(source, pair)
    shown = estimate(source, winx=8, wleft=1, zoommax=1.1, show=True)
    with shown.get_frame(1) as out:
        assert out[0][0, 2] == 255 and out[0][0, 8] == 255
        assert out[0][0, 1] == 0 and out[0][0, 7] == 0
    # A neighbor's constant surface is analyzed but must never be displayed.
    def late_impulse(n, f):
        if n:
            f[0][0, 2 - n] = 1
    late = edit(blank(), late_impulse)
    close_motion(motion(estimate(late, show=True), 2), (1, 0, 0, 1, 1))
    fails(lambda: estimate(late, show=True).get_frame(1))
    return shown


def diagnostics():
    source = impulse(width=96, height=48)
    available = hasattr(core, "text") and hasattr(core.text, "FrameProps")
    if not available:
        fails(lambda: estimate(source, info=True))
        return
    for show in (False, True):
        base = estimate(source, show=show)
        def diagnostic(n, f):
            f.props["DepanEstimate_info"] = (
                "fn=0 dx=0.00 dy=0.00 zoom=1.00000 trust=0.00 bad=1" if n == 0 else
                f"fn={n} dx=1.00 dy=0.00 zoom=1.00000 trust=80.81 bad=0")
        expected = core.text.FrameProps(edit(base, diagnostic), props=["DepanEstimate_info"])
        actual = estimate(source, show=show, info=True)
        for n in (2, 0, 1):
            with actual.get_frame(n) as a, expected.get_frame(n) as b:
                assert pixels(a) == pixels(b)
                assert dict(a.props) == dict(b.props)


def creation():
    source = impulse()
    for format, color in ((vs.RGB24, [0, 0, 0]), (vs.RGBS, [0, 0, 0])):
        fails(lambda: estimate(blank(format=format, color=color)))
    if hasattr(vs, "GRAYH"):
        fails(lambda: estimate(blank(format=vs.GRAYH, color=0.0)))
    for kwargs in (dict(trust=-1), dict(trust=101), dict(pixaspect=0), dict(pixaspect=-1),
                   dict(winx=3), dict(winx=6), dict(winy=5), dict(wleft=1), dict(wtop=1),
                   dict(dxmax=2), dict(dymax=2), dict(winy=1, dymax=0),
                   dict(zoommax=1.1, winx=2), dict(winx=-(1 << 63)), dict(winy=1 << 62),
                   dict(dxmax=1 << 62), dict(wleft=1 << 62)):
        fails(lambda: estimate(source, **kwargs))
    for name in ("trust", "zoommax", "stab", "pixaspect"):
        for value in (math.nan, math.inf, -math.inf, 1e300):
            fails(lambda: estimate(source, **{name: value}))
    # Empty values are supplied arguments, not omitted defaults.
    for name in ("trust", "winx", "winy", "wleft", "wtop", "dxmax", "dymax", "zoommax",
                 "stab", "pixaspect", "info", "show", "fields", "tff"):
        fails(lambda: estimate(source, **{name: []}))
    close_motion(motion(estimate(source, wleft=-(1 << 63), wtop=-(1 << 63),
                                 dxmax=-(1 << 63), dymax=-(1 << 63)), 2), (1, 0, 0, 1, 1))
    # Finite negative stab and zoommax are admitted; below-one zoom stays bad.
    estimate(source, stab=-1).get_frame(1).close()
    assert motion(estimate(source, zoommax=-1, dxmax=0), 1) == RESET
    # Variable geometry/format cannot become a constant-format estimate node.
    variable_size = core.std.Splice([blank(length=1), blank(width=8, length=1)], mismatch=True)
    fails(lambda: estimate(variable_size))
    variable_format = core.std.Splice([blank(length=1), blank(format=vs.GRAY16, length=1)], mismatch=True)
    fails(lambda: estimate(variable_format))


def concurrency():
    # Queue uncached nodes before retrieving any output, in nonchronological order.
    sources = [impulse(amplitude=1), impulse(amplitude=10), impulse(moving=False)]
    nodes = [estimate(source) for source in sources]
    jobs = [(index, n) for n in (2, 0, 1, 2, 1, 0) for index in range(3)]
    futures = [nodes[index].get_frame_async(n) for index, n in jobs]
    for (index, n), future in zip(jobs, futures):
        with future.result() as frame, sources[index].get_frame(n) as original:
            assert pixels(frame) == pixels(original)
            expected = RESET if n == 0 else (0 if index == 2 else 1, 0, 0, 1, 1)
            close_motion(tuple(frame.props[key] for key in MOTION), expected)


basic_and_properties()
formats_and_samples()
parity_and_dependencies()
display()
diagnostics()
creation()
concurrency()
print("neo-mv phase-6 DepanEstimate VS acceptance passed")
gc.collect()
blank(length=1).set_output()
