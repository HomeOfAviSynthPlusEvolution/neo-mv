"""Phase-5 specification acceptance from public synthetic properties only."""
import gc
import math
import struct
import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neo_mv
selected = mv.KernelInfo()
assert selected["backend"] in ("scalar", "highway") and selected["target"], selected
P = "MVUtensilsAnalysis"
MOTION = ("Depan_dx", "Depan_dy", "Depan_rot", "Depan_zoom", "Depan_goodmotion")


def blank(color=5, format=vs.GRAY8, width=16, height=16, length=3):
    return core.std.BlankClip(width=width, height=height, format=format, length=length,
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
    raise AssertionError("expected a controlled Depan error: " + fragment)


def pixels(frame):
    return [bytes(frame[p]) for p in range(frame.format.num_planes)]


def same_frame(first, second, n, props=True):
    with first.get_frame(n) as a, second.get_frame(n) as b:
        assert pixels(a) == pixels(b), n
        if props:
            assert set(a.props) == set(b.props), (set(a.props), set(b.props))
            for key in a.props:
                if key == "ForeignNode":
                    assert a.props[key].num_frames == b.props[key].num_frames
                else:
                    assert a.props[key] == b.props[key], key


def motion(node, n):
    with node.get_frame(n) as frame:
        return tuple(frame.props[key] for key in MOTION)


def check_fitted_motion(actual, expected):
    # Only nontrivial analysis fits use this allowance. A Highway target may
    # fuse arithmetic; the scalar oracle and all motion-validity flags stay exact.
    assert actual[4] == expected[4], (actual, expected)
    if selected["backend"] == "scalar":
        assert actual == expected, (actual, expected)
        return
    for key, value, reference in zip(MOTION[:4], actual[:4], expected[:4]):
        assert math.isfinite(value) and math.isfinite(reference), (key, value, reference)
        assert abs(value - reference) <= 1e-4 + 1e-5 * max(abs(value), abs(reference)), (key, value, reference)


def vectors(delta=-1, values=None, sad=None, length=3, missing=False):
    values = values if values is not None else [(0, 0)] * 4
    sad = sad if sad is not None else [0] * 4
    names = dict(Width=16, Height=16, RealWidth=16, RealHeight=16, HPad=16, VPad=16,
                 Pel=2, Levels=1, Chroma=0, XRatioUV=1, YRatioUV=1, BlkSizeX=8,
                 BlkSizeY=8, OverlapX=0, OverlapY=0, NBlkX=2, NBlkY=2,
                 DeltaFrame=delta, BitsPerSample=8)
    props = {P + name: value for name, value in names.items()}
    packed = []
    for x, y in values:
        value = (x & 0xffffffff) | ((y & 0xffffffff) << 32)
        packed.append(value - (1 << 64) if value >= (1 << 63) else value)
    if not missing:
        props[P + "Vectors"] = packed
        props[P + "SAD"] = sad
    return core.std.SetFrameProps(blank(format=vs.GRAY16, width=4, height=4, length=length), **props)


def data(dx=0.0, dy=0.0, rotation=0.0, zoom=1.0, good=1, length=3):
    # Carrier dimensions, format, rate, and pixels intentionally differ from clip.
    carrier = core.std.BlankClip(width=2, height=2, format=vs.RGBS, length=length,
                                  color=[0.25, 0.5, 0.75], fpsnum=7, fpsden=3)
    return core.std.SetFrameProps(carrier, Depan_dx=float(dx), Depan_dy=float(dy),
                                  Depan_rot=float(rotation), Depan_zoom=float(zoom), Depan_goodmotion=good)


def gradient(width=5, height=5, length=3):
    return edit(blank(width=width, height=height, length=length),
                lambda n, f: [f[0].__setitem__((y, x), 10*x + 40*y + 2*n)
                              for y in range(height) for x in range(width)])


def at(node, n, x=0, y=0, plane=0):
    with node.get_frame(n) as frame:
        return frame[plane][y, x]


def analysis():
    clip = core.std.SetFrameProps(blank(format=vs.GRAY16, width=48, height=48, color=1234),
                                  marker=[7, 9], payload=b"a\0b", real=[1.5, -0.0],
                                  Depan_dx=b"old wrong type", Depan_goodmotion=[3, 4],
                                  DepanAnalyse_info=b"inherited")
    foreign = blank(length=2)
    clip = edit(clip, lambda n, f: f.props.__setitem__("ForeignNode", foreign))
    field = vectors(values=[(1, -3), (0, 0), (4, 2), (-2, 1)])
    mask = blank(width=48, height=48, color=1)
    golden = mv.DepanAnalyse(clip, field, mask=mask, zoom=False, rot=False)
    expected = (0.4916095733642578, 0.0, -0.0, 1.0, 1)
    check_fitted_motion(motion(golden, 2), expected)
    if selected["backend"] == "scalar":
        assert math.copysign(1, motion(golden, 2)[2]) == -1
    with golden.get_frame(2) as out, clip.get_frame(2) as original:
        assert pixels(out) == pixels(original)
        assert set(out.props) == set(original.props) | set(MOTION)
        for key in original.props:
            if key in MOTION:
                continue
            if key == "ForeignNode":
                assert out.props[key].num_frames == 2
            else:
                assert out.props[key] == original.props[key], key
        assert all(isinstance(out.props[key], float) for key in MOTION[:4])
        assert original.props.Depan_dx == b"old wrong type"

    src = blank()
    zero_mask = blank(color=0)
    for delta, expected_good in [(1, [0, 0, 1]), (-1, [0, 1, 1])]:
        changing = edit(vectors(delta), lambda n, f: f.props.__delitem__(P + "Vectors") if n == 0 else None)
        result = mv.DepanAnalyse(src, changing, mask=zero_mask)
        assert [motion(result, n)[4] for n in range(3)] == expected_good
    # Matching wrong-type arrays are ignored at creation, then rejected when selected.
    malformed = core.std.SetFrameProps(vectors(), **{P + "Vectors": [0.0] * 4, P + "SAD": [0.0] * 4})
    created = mv.DepanAnalyse(src, malformed)
    fails(lambda: created.get_frame(0))
    absent = vectors(missing=True)
    assert motion(mv.DepanAnalyse(src, absent), 0) == (0, 0, 0, 1, 0)
    changed = edit(absent, lambda n, f: f.props.__setitem__(P + "BitsPerSample", 10) if n == 1 else None)
    assert motion(mv.DepanAnalyse(src, changed), 1) == (0, 0, 0, 1, 0)
    invalid = edit(vectors(), lambda n, f: f.props.__setitem__(P + "Width", 0) if n == 1 else None)
    assert motion(mv.DepanAnalyse(src, invalid), 1) == (0, 0, 0, 1, 0)
    scene = edit(vectors(), lambda n, f: f.props.__setitem__(P + "SAD", [401] * 4) if n == 1 else None)
    assert motion(mv.DepanAnalyse(src, scene), 1) == (0, 0, 0, 1, 0)
    levels = edit(vectors(), lambda n, f: f.props.__setitem__(P + "Levels", 7) if n == 1 else None)
    assert motion(mv.DepanAnalyse(src, levels, mask=zero_mask), 1)[4] == 1
    fails(lambda: mv.DepanAnalyse(src, vectors(sad=[-1] * 4)).get_frame(0))
    fails(lambda: mv.DepanAnalyse(src, vectors(delta=2)))
    fails(lambda: mv.DepanAnalyse(src, vectors(length=2)))
    fails(lambda: mv.DepanAnalyse(src, core.std.SetFrameProps(vectors(), **{P + "Width": 0})))

    seen = []
    def mask_error(n, f):
        seen.append(n)
        raise vs.Error("Depan mask dependency sentinel")
    broken_mask = edit(zero_mask, mask_error)
    fails(lambda: mv.DepanAnalyse(src, absent, mask=broken_mask).get_frame(1), "mask dependency sentinel")
    assert 1 in seen, seen
    fails(lambda: mv.DepanAnalyse(src, vectors(), mask=broken_mask).get_frame(1), "mask dependency sentinel")
    fails(lambda: mv.DepanAnalyse(src, invalid, mask=broken_mask).get_frame(1), "mask dependency sentinel")
    fails(lambda: mv.DepanAnalyse(src, scene, mask=broken_mask).get_frame(1), "mask dependency sentinel")
    for invalid_mask in [blank(width=8), blank(format=vs.GRAY16), blank(length=2)]:
        fails(lambda: mv.DepanAnalyse(src, absent, mask=invalid_mask))
    fails(lambda: mv.DepanAnalyse(src, absent, mask=[]))

    # Ineligibility skips fitting samples, but a supplied mask must still be retrievable.
    fails(lambda: mv.DepanAnalyse(src, absent, mask=broken_mask, error=-1).get_frame(1),
          "mask dependency sentinel")
    negative = mv.DepanAnalyse(src, absent, mask=zero_mask, error=-1)
    assert motion(negative, 1) == (0, 0, -0.0, 1, 1)
    fails(lambda: mv.DepanAnalyse(src, absent, error=-1, fields=True).get_frame(1))
    top = mv.DepanAnalyse(src, absent, error=-1, fields=True, tff=True)
    assert motion(top, 1)[1] == -1 and motion(top, 2)[1] == 1
    wide_bool = mv.DepanAnalyse(src, absent, error=-1, fields=-(1 << 63), tff=1 << 40)
    assert motion(wide_bool, 1) == motion(top, 1)
    assert motion(wide_bool, 2) == motion(top, 2)
    assert motion(mv.DepanAnalyse(src, absent, fields=True), 1)[4] == 0
    bottom_property = core.std.SetFrameProps(src, _Field=0)
    assert motion(mv.DepanAnalyse(bottom_property, vectors(), mask=zero_mask, fields=True), 0)[1] == -1
    assert motion(mv.DepanAnalyse(bottom_property, vectors(), mask=zero_mask, fields=True, tff=True), 0)[1] == 1
    assert motion(mv.DepanAnalyse(src, vectors(), mask=zero_mask, error=1, fields=True), 0)[4] == 0

    for format, color in [(vs.RGB24, [11, 22, 33]), (vs.RGBS, [0.25, 0.5, 0.75]), (vs.GRAYS, 0.5)]:
        copied = blank(format=format, color=color)
        result = mv.DepanAnalyse(copied, absent)
        same_frame(result, copied, 1, props=False)
        assert motion(result, 1) == (0, 0, 0, 1, 0)
    # Stored quiet-NaN payloads belong to copied pixels, never to fitting arithmetic.
    nan_bytes = struct.pack("=I", 0x7fc12345)
    nan_value = struct.unpack("=f", nan_bytes)[0]
    nan_clip = edit(blank(format=vs.GRAYS, color=0.5),
                    lambda n, f: f[0].__setitem__((0, 0), nan_value))
    with nan_clip.get_frame(1) as original:
        assert bytes(original[0])[:4] == nan_bytes
    same_frame(mv.DepanAnalyse(nan_clip, absent), nan_clip, 1, props=False)
    if hasattr(vs, "GRAYH"):
        fails(lambda: mv.DepanAnalyse(blank(format=vs.GRAYH, color=0.5), absent))
    return golden


def analysis_current_metadata():
    src = blank(width=48, height=48)
    zeros = blank(width=48, height=48, color=0)
    ones = blank(width=48, height=48, color=1)
    identity = (0, 0, -0.0, 1, 1)
    invalid = (0, 0, 0, 1, 0)
    golden = (0.4916095733642578, 0, -0.0, 1, 1)

    def current(properties, node=None):
        def replace(n, f):
            if n == 0:
                return
            for key, value in properties.items():
                if value is None:
                    del f.props[P + key]
                else:
                    f.props[P + key] = value
        return edit(vectors() if node is None else node, replace)

    # The current pel controls displacement units; doubling packed components
    # with p=4 reproduces the complete p=2 example's four observations exactly.
    doubled = vectors(values=[(2, -6), (0, 0), (8, 4), (-4, 2)])
    result = mv.DepanAnalyse(src, current(dict(Pel=4), doubled), mask=ones, zoom=False, rot=False)
    check_fitted_motion(motion(result, 1), golden)

    # New centers (2,2),(6,2),(2,6),(6,6) read zeros. The old centers read 255,
    # so using creation geometry would incorrectly fit a nonzero translation.
    def old_centers(n, f):
        for y in [4, 12]:
            for x in [4, 12]:
                f[0][y, x] = 255
    mask = edit(zeros, old_centers)
    smaller = current(dict(BlkSizeX=4, BlkSizeY=4), vectors(values=[(2, 0)] * 4))
    assert motion(mv.DepanAnalyse(src, smaller, mask=mask), 1) == identity

    # Creation keeps T1=400 and T2=fl32(2.04), even as precision, chroma,
    # block area or the current number of blocks changes.
    for properties in [dict(BitsPerSample=10, SAD=[500] * 4),
                       dict(Chroma=1, XRatioUV=2, YRatioUV=2, SAD=[500] * 4)]:
        assert motion(mv.DepanAnalyse(src, current(properties), mask=zeros), 1) == invalid
    smaller_sad = current(dict(BlkSizeX=4, BlkSizeY=4, SAD=[200] * 4))
    assert motion(mv.DepanAnalyse(src, smaller_sad, mask=zeros), 1) == identity
    grid = dict(Width=24, RealWidth=24, NBlkX=3)
    assert motion(mv.DepanAnalyse(src, current(grid), mask=zeros), 1) == invalid  # Four != current six.
    grid.update(Vectors=[0] * 6, SAD=[0] * 6)
    assert motion(mv.DepanAnalyse(src, current(grid), mask=zeros), 1) == identity
    assert motion(mv.DepanAnalyse(src, current(dict(grid, SAD=[401, 401, 0, 0, 0, 0])), mask=zeros), 1) == identity
    assert motion(mv.DepanAnalyse(src, current(dict(grid, SAD=[401, 401, 401, 0, 0, 0])), mask=zeros), 1) == invalid
    fails(lambda: mv.DepanAnalyse(src, current(dict(grid, Vectors=[0.0] * 6)), mask=zeros).get_frame(1))

    # At the second column vx=32 is excluded with hp=16 but allowed with hp=17.
    widened = current(dict(HPad=17), vectors(values=[(32, 0)] * 4))
    assert motion(mv.DepanAnalyse(src, widened, mask=zeros), 1) == identity
    fails(lambda: mv.DepanAnalyse(src, vectors(values=[(32, 0)] * 4), mask=zeros).get_frame(1))
    narrowed = current(dict(HPad=0), vectors(values=[(-1, 0)] * 4))
    fails(lambda: mv.DepanAnalyse(src, narrowed, mask=zeros).get_frame(1))
    # Missing/wrong-type current scalars default to zero, not creation metadata.
    for pel in [None, 0.5]:
        bad_metadata = current(dict(Pel=pel, Vectors=[0.0] * 4, SAD=[-1] * 4))
        assert motion(mv.DepanAnalyse(src, bad_metadata, mask=zeros), 1) == invalid

    # Save the creation direction for selection and conversion. Frame 2 has no
    # arrays; current frame 1 delta changes cannot redirect either operation.
    components = [(1, -3), (0, 0), (4, 2), (-2, 1)]
    for delta, altered in [(-1, 1), (-1, 0), (-1, None), (1, -1), (1, 0), (1, None)]:
        def change_direction(n, f):
            if n == 1:
                if altered is None:
                    del f.props[P + "DeltaFrame"]
                else:
                    f.props[P + "DeltaFrame"] = altered
            elif n == 2:
                del f.props[P + "Vectors"]
        carrier = edit(vectors(delta=delta, values=components), change_direction)
        result = mv.DepanAnalyse(src, carrier, mask=ones, zoom=False, rot=False)
        if delta == -1:
            check_fitted_motion(motion(result, 1), golden)
            assert motion(result, 2) == invalid
        else:
            check_fitted_motion(motion(result, 2), (-golden[0], 0, 0, 1, 1))

    # The mask dependency is indexed by output n, even for d=+1 selecting k=0.
    seen = []
    def mask_at_output(n, f):
        seen.append(n)
        if n == 1:
            raise vs.Error("Depan current-mask frame sentinel")
    mask = edit(zeros, mask_at_output)
    result = mv.DepanAnalyse(src, vectors(delta=1, missing=True), mask=mask)
    assert motion(result, 0) == invalid
    fails(lambda: result.get_frame(1), "current-mask frame sentinel")
    assert 0 in seen and 1 in seen, seen


def compensate_formats():
    formats = [vs.GRAY8, vs.GRAY10, vs.GRAY16, vs.YUV420P8, vs.YUV420P10, vs.YUV420P16,
               vs.YUV422P10, vs.YUV444P16]
    invalid = data(good=0)
    for format in formats:
        fmt = core.get_video_format(format)
        color = [10 + 16*p for p in range(fmt.num_planes)]
        source = core.std.SetFrameProps(blank(format=format, color=color[0] if fmt.num_planes == 1 else color,
                                              width=6, height=4), _ColorRange=1)
        for mode in [0, 1, 2]:
            result = mv.DepanCompensate(source, invalid, offset=1, subpixel=mode)
            with result.get_frame(1) as out, source.get_frame(0) as original:
                for p in range(fmt.num_planes):
                    view = out[p]
                    border = 0 if p == 0 else 1 << (fmt.bits_per_sample - 1)
                    for y in range(view.shape[0]):
                        for x in range(view.shape[1]):
                            expected = border if mode == 1 and x == view.shape[1]-1 and y < view.shape[0]-1 else color[p]
                            assert view[y, x] == expected, (format, mode, p, x, y, view[y, x], expected)
                assert out.props._ColorRange == original.props._ColorRange
        # All planes retain the specified numeric border, independent of range tags.
        far = mv.DepanCompensate(source, data(dx=1e300), offset=1, subpixel=0)
        with far.get_frame(1) as out:
            for p in range(fmt.num_planes):
                assert all(out[p][y, x] == (0 if p == 0 else 1 << (fmt.bits_per_sample-1))
                           for y in range(out[p].shape[0]) for x in range(out[p].shape[1]))
    for format, width, height in [(vs.GRAY8, 1, 1), (vs.YUV420P8, 2, 2), (vs.YUV420P8, 2, 4)]:
        fmt = core.get_video_format(format)
        source = blank(format=format, color=10 if fmt.num_planes == 1 else [10, 20, 30], width=width, height=height)
        for mode in [0, 1]:
            mv.DepanCompensate(source, invalid, offset=1, subpixel=mode).get_frame(1).close()
        if height >> fmt.subsampling_h < 2:
            fails(lambda: mv.DepanCompensate(source, invalid, offset=0, subpixel=2))
        else:
            same_frame(mv.DepanCompensate(source, invalid, offset=1, subpixel=2), source, 1)
    for format, color in [(vs.GRAYS, 0.5), (vs.RGB24, [10, 20, 30]), (vs.RGBS, [0.1, 0.2, 0.3])]:
        fails(lambda: mv.DepanCompensate(blank(format=format, color=color), invalid, offset=0))


def compensate_temporal():
    source = gradient()
    foreign = blank(length=2)
    source = edit(source, lambda n, f: [f.props.__setitem__("marker", n),
                                      f.props.__setitem__("Depan_dx", float(100+n)),
                                      f.props.__setitem__("DepanCompensate_info", b"inherited"),
                                      f.props.__setitem__("ForeignNode", foreign)])
    shifted = mv.DepanCompensate(source, data(dx=2), offset=1, subpixel=0)
    assert at(shifted, 1) == 20
    with shifted.get_frame(1) as out:
        assert out.props.marker == 0 and out.props.Depan_dx == 100.0
        assert out.props.DepanCompensate_info == b"inherited" and out.props.ForeignNode.num_frames == 2
    same_frame(shifted, source, 0)
    assert at(mv.DepanCompensate(source, data(dx=2), offset=1.5, subpixel=0), 2) == 20
    assert at(mv.DepanCompensate(source, data(dx=2), offset=-1.5, subpixel=0), 0, x=2) == 4
    for mode, dx, mirror, blur, expected in [(0, -2, 4, 2, 15), (0, 5, 8, 2, 30),
                                            (1, 5, 8, 2, 35), (1, 4, 0, 0, 0), (1, 4, 8, 0, 40)]:
        assert at(mv.DepanCompensate(source, data(dx=dx), offset=1, subpixel=mode,
                                    mirror=mirror, blur=blur), 1) == expected

    seen = []
    def later_error(n, f):
        seen.append(n)
        if n == 1:
            f.props["Depan_goodmotion"] = 0
        elif n == 2:
            raise vs.Error("Depan unused later data sentinel")
    stop = edit(data(), later_error)
    stopped = mv.DepanCompensate(source, stop, offset=2, subpixel=0)
    assert at(stopped, 2, x=1) == 10
    assert 1 in seen and 2 not in seen, seen
    seen.clear()
    reverse = mv.DepanCompensate(source, edit(data(), later_error), offset=-2, subpixel=0)
    assert at(reverse, 0, x=1) == 14
    assert 1 in seen and 2 not in seen, seen
    malformed_first = edit(data(), lambda n, f: f.props.__delitem__("Depan_rot") if n == 1 else
                           f.props.__setitem__("Depan_goodmotion", 0))
    fails(lambda: mv.DepanCompensate(source, malformed_first, offset=2).get_frame(2))
    for bad in [core.std.RemoveFrameProps(data(good=0), props=["Depan_rot"]),
                core.std.SetFrameProps(data(good=0), Depan_dx=0),
                core.std.SetFrameProps(data(good=0), Depan_zoom=math.inf)]:
        created = mv.DepanCompensate(source, bad, offset=1, subpixel=0)
        fails(lambda: created.get_frame(1))
    array_data = core.std.SetFrameProps(data(), Depan_dx=[2.0, math.nan], Depan_goodmotion=[-3, 0])
    assert at(mv.DepanCompensate(source, array_data, offset=1, subpixel=0), 1) == 20

    touched = []
    def data_error(n, f):
        touched.append(n)
        raise vs.Error("Depan bypass data sentinel")
    no_data = edit(data(), data_error)
    bypass = mv.DepanCompensate(source, no_data, offset=0, fields=True)
    same_frame(bypass, source, 2)
    same_frame(mv.DepanCompensate(source, no_data, offset=1, fields=True), source, 0)
    assert touched == [], touched
    fails(lambda: mv.DepanCompensate(source, no_data, offset=1).get_frame(1), "bypass data sentinel")

    def target_error(n, f):
        if n == 2:
            raise vs.Error("Depan target parity sentinel")
    fragile = edit(source, target_error)
    assert at(mv.DepanCompensate(fragile, data(good=0), offset=1, subpixel=0), 2) == 2
    assert at(mv.DepanCompensate(fragile, data(good=0), offset=1, subpixel=0,
                                fields=True, matchfields=False), 2) == 2
    fails(lambda: mv.DepanCompensate(fragile, data(good=0), offset=1, fields=True).get_frame(2), "target parity sentinel")
    assert at(mv.DepanCompensate(fragile, data(good=0), offset=1, subpixel=0,
                                fields=True, tff=True), 2) == 2
    assert at(mv.DepanCompensate(fragile, data(good=0), offset=1, subpixel=0,
                                fields=1 << 40, tff=-(1 << 63)), 2) == 2
    assert at(mv.DepanCompensate(fragile, data(good=0), offset=1, subpixel=0,
                                fields=True, tff=False), 2) == 42
    fails(lambda: mv.DepanCompensate(source, data(good=0), offset=1, fields=True).get_frame(1))
    for kwargs in [dict(offset=11), dict(subpixel=3), dict(mirror=16), dict(blur=-1), dict(pixaspect=0)]:
        fails(lambda: mv.DepanCompensate(source, data(), **kwargs))
    fails(lambda: mv.DepanCompensate(source, data(length=2)))
    return shifted


def diagnostics():
    source = blank(width=48, height=48)
    absent = vectors(missing=True)
    available = hasattr(core, "text") and hasattr(core.text, "FrameProps")
    if not available:
        fails(lambda: mv.DepanAnalyse(source, absent, info=True))
        fails(lambda: mv.DepanCompensate(source, data(), info=True))
        return
    base = mv.DepanAnalyse(source, absent)
    def analysis_text(n, f):
        f.props["DepanAnalyse_info"] = (f"fn={n} iter=0 error=30.000 dx=0.00 dy=0.00 "
                                        "rot=0.000 zoom=1.00000 bad=1")
    expected = core.text.FrameProps(edit(base, analysis_text), props=["DepanAnalyse_info"])
    actual = mv.DepanAnalyse(source, absent, info=True)
    for n in [2, 0, 1]:
        same_frame(actual, expected, n)

    source = core.std.SetFrameProps(source, DepanCompensate_info=b"inherited")
    base = mv.DepanCompensate(source, data(dx=2), offset=1, subpixel=0)
    def compensation_text(n, f):
        if n:
            f.props["DepanCompensate_info"] = (f"offset=1.00, {n-1} to {n}, dx=2.00, dy=0.00, "
                                               "rot=-0.000 zoom=1.00000")
    expected = core.text.FrameProps(edit(base, compensation_text), props=["DepanCompensate_info"])
    actual = mv.DepanCompensate(source, data(dx=2), offset=1, subpixel=0, info=True)
    for n in [0, 2, 1]:
        same_frame(actual, expected, n)
    expected = core.text.FrameProps(source, props=["DepanCompensate_info"])
    same_frame(mv.DepanCompensate(source, data(), offset=0, info=True), expected, 1)


def run():
    fitted = analysis()
    analysis_current_metadata()
    compensate_formats()
    shifted = compensate_temporal()
    diagnostics()
    jobs = [(node, n) for n in [2, 0, 1, 2, 1, 0] for node in [fitted, shifted]]
    expected = {}
    for node, n in jobs:
        with node.get_frame(n) as frame:
            expected[(id(node), n)] = pixels(frame)
    futures = [node.get_frame_async(n) for node, n in jobs]
    for (node, n), future in zip(jobs, futures):
        with future.result() as frame:
            assert pixels(frame) == expected[(id(node), n)]
    print("neo-mv phase-5 Depan VS acceptance passed")


run()
gc.collect()
blank(length=1).set_output()
