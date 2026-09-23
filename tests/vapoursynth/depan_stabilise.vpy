"""Phase-7 independent acceptance fixtures derived from public specifications."""
import concurrent.futures
import gc
import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neo_mv


def blank(format=vs.GRAY8, width=48, height=48, length=8, color=10):
    fmt = core.get_video_format(format)
    colors = color if fmt.num_planes == 1 else [color, color, color]
    return core.std.BlankClip(format=format, width=width, height=height, length=length,
                              color=colors, fpsnum=24, fpsden=1)


def edit(node, callback):
    def apply(n, f):
        out = f.copy()
        callback(n, out)
        return out
    return core.std.ModifyFrame(node, node, apply)


def data(length=8, bad=(), malformed=(), step=0.0):
    def props(n, f):
        f.props.update(Depan_dx=float(step), Depan_dy=0.0, Depan_rot=0.0,
                       Depan_zoom=1.0, Depan_goodmotion=int(n not in bad))
        if n in malformed:
            del f.props['Depan_dx']
    return edit(blank(width=4, height=4, length=length), props)


def pixels(frame):
    return [bytes(frame[p]) for p in range(frame.format.num_planes)]


def fails(call):
    try:
        call()
    except vs.Error:
        return
    raise AssertionError('expected controlled DepanStabilise failure')


def identity_and_properties():
    def props(n, f):
        f.props.update(TestMarker=[n, 9], Depan_dx=91.0, Depan_goodmotion=17,
                       DepanStabilise_info=b'earlier', _SceneChangePrev=7, _Range=1)
    clip = edit(blank(), props)
    motion = data(malformed=[0])
    for method in [0, 1]:
        node = mv.DepanStabilise(clip, motion, method=method, subpixel=0, fields=True)
        for n in [7, 0, 3, 1, 6]:
            with node.get_frame(n) as f, clip.get_frame(n) as original:
                assert pixels(f) == pixels(original)
                assert dict(f.props) == dict(original.props)
    # No bypass for identity: incomplete bilinear footprint writes a border.
    node = mv.DepanStabilise(clip, motion, subpixel=1)
    filled = mv.DepanStabilise(clip, motion, subpixel=1, prev=1)
    with node.get_frame(0) as a, filled.get_frame(0) as b:
        assert a[0][1, 47] == 0 and a[0][47, 47] == 10
        assert b[0][1, 47] == 10


def dependency_boundaries():
    clip = blank()
    # Frame zero is synthetic; malformed earlier data is hidden by the cut.
    mv.DepanStabilise(clip, data(bad=[3], malformed=[0, 2]), subpixel=0).get_frame(4)
    fails(lambda: mv.DepanStabilise(clip, data(malformed=[3])).get_frame(4))
    # Even a current-frame cut still needs the method-1 forward scan.
    fails(lambda: mv.DepanStabilise(clip, data(bad=[3], malformed=[4]), method=1).get_frame(3))
    # A bad frame at the positive interval endpoint stops the smoothing scan.
    mv.DepanStabilise(clip, data(bad=[4], malformed=[5]), method=1, cutoff=6).get_frame(3)
    # Next-fill must decode past a cut; this cannot be hidden by selection.
    fails(lambda: mv.DepanStabilise(clip, data(bad=[4], malformed=[5]), method=1,
                                   cutoff=6, next=2).get_frame(3))
    # Empty inertial adaptive recurrence does not evaluate tzoom=0.
    node = mv.DepanStabilise(clip, data(), addzoom=True, tzoom=0, initzoom=2)
    node.get_frame(0)
    node.get_frame(1)
    fails(lambda: node.get_frame(2))
    fails(lambda: mv.DepanStabilise(clip, data(), method=1, addzoom=True, tzoom=0).get_frame(0))


def formats_and_creation():
    for fmt in [vs.GRAY8, vs.GRAY9, vs.GRAY10, vs.GRAY12, vs.GRAY14, vs.GRAY16,
                vs.YUV420P8, vs.YUV422P10, vs.YUV444P16]:
        clip = blank(fmt)
        for mode in [0, 1, 2]:
            node = mv.DepanStabilise(clip, data(), method=1, prev=2, next=2,
                                    subpixel=mode, mirror=15, blur=3)
            with node.get_frame(3) as f:
                assert f.format.id == fmt and f.width == 48 and f.height == 48
    small = blank(vs.YUV420P8, width=2, height=2)
    for mode in [0, 1]:
        mv.DepanStabilise(small, data(), subpixel=mode).get_frame(0)
    fails(lambda: mv.DepanStabilise(small, data(), subpixel=2))
    for fmt in [vs.GRAYS, vs.RGB24, vs.YUV440P8]:
        fails(lambda fmt=fmt: mv.DepanStabilise(blank(fmt), data()))
    fails(lambda: mv.DepanStabilise(blank(), data(length=7)))
    for args in [dict(cutoff=0), dict(initzoom=0), dict(tzoom=-1), dict(pixaspect=0),
                 dict(method=2), dict(prev=-1), dict(next=-1), dict(mirror=16), dict(blur=-1),
                 dict(subpixel=3), dict(cutoff=float('nan')), dict(damping=float('inf'))]:
        fails(lambda args=args: mv.DepanStabilise(blank(), data(), **args))
    # Method 1 skips inertial coefficients, including overflowing damping math.
    mv.DepanStabilise(blank(), data(), damping=1e30, method=1).get_frame(2)
    fails(lambda: mv.DepanStabilise(blank(), data(), damping=1e30, method=0))
    # Data geometry/sample type need not match clip; sample NaN is never read.
    foreign = core.std.SetFrameProps(blank(vs.GRAYS, width=4, height=4, color=0),
                                    Depan_dx=0.0, Depan_dy=0.0, Depan_rot=0.0,
                                    Depan_zoom=1.0, Depan_goodmotion=1)
    foreign = edit(foreign, lambda n, f: f[0].__setitem__((0, 0), float('nan')))
    mv.DepanStabilise(blank(), foreign).get_frame(3)


def diagnostics_and_requests():
    clip = blank(width=320, height=48)
    motion = data()
    diagnostic = mv.DepanStabilise(clip, motion, info=True, subpixel=0)
    with diagnostic.get_frame(0) as f:
        assert f.props['DepanStabilise_info'].startswith('frame=0 BASE!=0 ')
    with diagnostic.get_frame(3) as f:
        assert f.props['DepanStabilise_info'].startswith('frame=3 base =0 ')
    moving = data(step=0.25)
    def create_nodes():
        return [mv.DepanStabilise(blank(), moving, method=m, addzoom=z, prev=2, next=2)
                for m, z in [(0, False), (0, True), (1, False), (1, True)]]
    references = create_nodes()
    expected = [[pixels(node.get_frame(n)) for n in range(8)] for node in references]
    nodes = create_nodes()  # Cold independent instances, not warmed output caches.
    def request(pair):
        index, n = pair
        with nodes[index].get_frame(n) as f:
            assert pixels(f) == expected[index][n]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(request, [(i, n) for n in [7, 0, 4, 1, 6, 2, 3, 7] for i in range(4)]))


def numerical_recovery():
    def pattern(n, f):
        for plane in range(f.format.num_planes):
            view = f[plane]
            for y in range(view.shape[0]):
                for x in range(view.shape[1]):
                    view[y, x] = (17 * x + 29 * y + 13 * n + 23 * plane) % 211 + 16
        f.props.update(TestMarker=n, DepanStabilise_info=b'inherited', Depan_goodmotion=1)

    clip = edit(core.std.AssumeFPS(blank(length=12), fpsnum=24000, fpsden=1001), pattern)
    moving = data(length=12, step=20)
    control = data(length=12, step=20, bad=[7])  # Later motion stays valid.
    variants = [{}, dict(initzoom=1.2), dict(addzoom=True), dict(fitlast=4),
                dict(prev=2), dict(next=2), dict(prev=2, next=2, mirror=15, blur=3)]
    for limit in [-10, 10]:
        for mode in [0, 1, 2]:
            for extra in variants:
                args = dict(dxmax=limit, subpixel=mode, **extra)
                recovered = mv.DepanStabilise(clip, moving, **args)
                expected = mv.DepanStabilise(clip, control, **args)
                with recovered.get_frame(7) as f, expected.get_frame(7) as ref:
                    assert pixels(f) == pixels(ref), args
                    assert dict(f.props) == dict(ref.props)
                    assert f.props.TestMarker == 7 and f.props.Depan_goodmotion == 1

    args = dict(dxmax=10, prev=2, next=2, initzoom=1.2, addzoom=True, mirror=15, blur=3)
    reference = mv.DepanStabilise(clip, moving, **args)
    expected = [pixels(reference.get_frame(n)) for n in range(12)]
    nodes = [mv.DepanStabilise(clip, moving, **args) for _ in range(2)]
    def request(pair):
        i, n = pair
        with nodes[i].get_frame(n) as f:
            assert pixels(f) == expected[n]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(request, [(i, n) for n in [11, 7, 0, 8, 6, 1, 9, 7, 10, 2, 5, 4, 3]
                                   for i in range(2)]))

    # Recovery must not hide required dependency errors or an independent zero divisor.
    fails(lambda: mv.DepanStabilise(clip, data(length=12, step=20, malformed=[8]),
                                   dxmax=10, next=2).get_frame(7))
    fails(lambda: mv.DepanStabilise(clip, moving, dxmax=10, addzoom=True, tzoom=0).get_frame(7))
    # Exercise the external diagnostics renderer with sufficient text width.
    wide = core.std.AssumeFPS(blank(width=320, length=12), fpsnum=24000, fpsden=1001)
    diagnostic = mv.DepanStabilise(wide, moving, dxmax=10, info=True, initzoom=1.2)
    with diagnostic.get_frame(7) as f:
        assert f.props.DepanStabilise_info.startswith('frame=7 BASE!=7 ')


identity_and_properties()
dependency_boundaries()
formats_and_creation()
diagnostics_and_requests()
numerical_recovery()
print('neo-mv phase-7 DepanStabilise independent VS acceptance passed')
gc.collect()
blank(length=1).set_output()
