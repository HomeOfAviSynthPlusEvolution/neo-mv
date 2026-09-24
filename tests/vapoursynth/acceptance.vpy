"""Host acceptance from approved phase-1 formulas; no legacy MV oracle."""
import gc
import importlib.util
import math
import os
from pathlib import Path
import vapoursynth as vs

core = vs.core
core.num_threads = 4
core.std.LoadPlugin(path=plugin)
mv = core.neo_mv
selected = mv.KernelInfo()
expected_kernel = globals().get("kernel", "scalar")
assert selected["backend"] == expected_kernel, selected
assert selected["target"], selected
# Once selected, later environment changes must not change existing graphs or
# the backend selected for new graphs in this loaded plugin.
os.environ["NEO_MV_KERNEL"] = "invalid-after-initialization"
assert mv.KernelInfo() == selected
P = "MVUtensils"


def values(value):
    return list(value) if isinstance(value, (list, tuple)) else [value]


def outputs(value):
    return [value] if isinstance(value, vs.VideoNode) else list(value)


def fails(call, fragment=None):
    try:
        call()
    except vs.Error as error:
        if fragment is not None:
            assert fragment.lower() in str(error).lower(), str(error)
        return
    raise AssertionError("expected controlled host error")


def edit(clip, callback):
    def select(n, f):
        result = f.copy()
        callback(n, result)
        return result
    return core.std.ModifyFrame(clip, clip, select)


def set_props(clip, **props):
    return edit(clip, lambda n, f: f.props.update(props))


def blank(format=vs.GRAY8, width=8, height=8, length=3, color=0):
    return core.std.BlankClip(width=width, height=height, length=length,
                              format=format, color=color, fpsnum=24000, fpsden=1001)


def super_clip(clip, **kwargs):
    args = dict(blksize=[4], overlap=[0], pad=[4], pel=2)
    args.update(kwargs)
    return mv.Super(clip, **args)


def analysis(frame, prefix=P):
    return (values(frame.props[prefix + "AnalysisVectors"]),
            values(frame.props[prefix + "AnalysisSAD"]))


def pixels(frame):
    return [bytes(frame[p]) for p in range(len(frame))]


# Independent formula fixture: rightward one-pixel motion, then boundary data.
rows = ([10, 20, 30, 30], [0, 10, 20, 30])
def paint(n, f):
    for y in range(4):
        for x in range(4):
            f[0][y, x] = rows[n][x]
pattern = edit(blank(width=4, height=4, length=2), paint)
source = set_props(pattern, marker=[7, 9], **{
    P + "AnalysisVectors": [123], P + "AnalysisSAD": [456],
    "OtherAnalysisVectors": [88], "OtherAnalysisSAD": [99]})
super_pattern = super_clip(source, pel=1)
motion = mv.Analyse(super_pattern, search=4, pelsearch=1, pnew=0)
assert analysis(motion.get_frame(0)) == ([1], [0])
boundary = motion.get_frame(1)
assert P + "AnalysisVectors" not in boundary.props and P + "AnalysisSAD" not in boundary.props
assert boundary.props.OtherAnalysisVectors == 88 and boundary.props.OtherAnalysisSAD == 99
assert boundary.props.marker == [7, 9]
assert pixels(boundary) == pixels(source.get_frame(1))
assert (motion.width, motion.height, motion.num_frames, motion.fps_num, motion.fps_den) == (4, 4, 2, 24000, 1001)

# Pixel fidelity, real auxiliary frames, quarter read domains, copy lifetimes.
for fmt, color in [(vs.GRAY8, 37), (vs.YUV420P16, [1023, 777, 999]),
                   (vs.YUV444PS, [0.25, -0.125, 0.125])]:
    original = set_props(blank(fmt, color=color), marker=123)
    produced = super_clip(original, pel=4)
    frame = produced.get_frame(0)
    assert pixels(frame) == pixels(original.get_frame(0)) and frame.props.marker == 123
    aux = values(frame.props[P + "NeoMVSuperPlanesV1"])
    assert aux[0].width == 16 and aux[0].height == 16
    assert aux[3].width == 15 and aux[3].height == 16
    assert aux[12].width == 16 and aux[12].height == 15
    copied = core.std.SetFrameProps(produced, copied=1)
    del frame, aux, produced
    gc.collect()
    measured = mv.Analyse(copied)
    for n in [1, 0, 1]:
        vectors, errors = analysis(measured.get_frame(n))
        assert vectors == [0] * 4 and errors == [0] * 4
    replaced = super_clip(copied, pel=1)
    assert replaced.get_frame(0).props[P + "SuperPel"] == 1
    assert len(values(replaced.get_frame(0).props[P + "NeoMVSuperPlanesV1"])) == len(original.get_frame(0))

# The producer node is gone; a native frame copy alone retains the payload.
def detached_super():
    producer = super_clip(blank(color=17), pel=4)
    saved = producer.get_frame(0).copy()
    del producer
    gc.collect()
    carrier = blank()
    return core.std.ModifyFrame(carrier, carrier, lambda n, f: saved.copy())
detached = detached_super()
assert analysis(mv.Analyse(detached).get_frame(0)) == ([0] * 4, [0] * 4)
assert detached.get_frame(0)[0][0, 0] == 17

# External pel samples: integer phase uses the original, including when pel=1.
gray = blank(width=8, height=8, color=3)
external = blank(width=16, height=16, color=9)
external_super = super_clip(gray, pelclip=external)
aux = values(external_super.get_frame(0).props[P + "NeoMVSuperPlanesV1"])
assert aux[0][0][4, 4] == 3 and aux[1][0][4, 4] == 9
def upstream_error(n, f):
    raise vs.Error("injected upstream failure")
unused = core.std.ModifyFrame(blank(width=4, height=4, length=1), blank(width=4, height=4, length=1), upstream_error)
assert super_clip(gray, pel=1, pelclip=unused).get_frame(2)[0][0, 0] == 3
fails(lambda: super_clip(gray, pelclip=blank(width=15, height=16)))
fails(lambda: super_clip(gray, pelclip=blank(width=16, height=16, length=1)))

# Argument omission, array inheritance, int32 saturation and full int64 truth.
s = super_clip(blank())
assert analysis(mv.Analyse(s, blksize=[], overlap=[], pnew=0).get_frame(0)) == analysis(mv.Analyse(s, pnew=0, pzero=0, pelsearch=2).get_frame(0))
fallback = mv.Super(blank(), blksize=[], overlap=[], pel=1)
assert fallback.get_frame(0).props[P + "SuperBlkSizeX"] == 8
fails(lambda: mv.Super(blank(), overlap=[0]))
fails(lambda: super_clip(blank(), blksize=[4, 4, 4]))
fails(lambda: mv.Analyse(s, delta=0))
fails(lambda: super_clip(blank(), prefix=b"x\x00y"))
huge_delta = mv.Analyse(s, delta=2**62).get_frame(0)
assert huge_delta.props[P + "AnalysisDeltaFrame"] == 2**31 - 1
assert P + "AnalysisVectors" not in huge_delta.props
truth = mv.Analyse(s, fields=-(2**62), tff=2**62).get_frame(0)
assert analysis(truth) == ([1 << 32] * 4, [0] * 4)
assert analysis(mv.Analyse(s, fields=1, tff=0).get_frame(0)) == ([-(1 << 32)] * 4, [0] * 4)
fails(lambda: mv.Analyse(s, fields=1).get_frame(2), "_Field")
assert analysis(mv.Analyse(s, fields=0, tff=1).get_frame(0)) == ([0] * 4, [0] * 4)
with_field = edit(s, lambda n, f: f.props.update({"_Field": 1 - n % 2}))
assert analysis(mv.Analyse(with_field, fields=1).get_frame(0)) == ([1 << 32] * 4, [0] * 4)
# Whole-domain admission is independent of the search and reference boundary.
unsafe = super_clip(blank(vs.YUV420P8, width=16, height=16, color=[0, 128, 128]), blksize=[8], pad=[3])
fails(lambda: mv.Analyse(unsafe, search=4, delta=100))

# Member expansion/order and independent temporal requests.
long_super = super_clip(blank(length=7))
many = outputs(mv.AnalyseMany(long_super, radius=3, delta=2, pnew=0))
assert len(many) == 6
assert [node.get_frame(0).props[P + "AnalysisDeltaFrame"] for node in many] == [2, -2, 4, -4, 6, -6]
for i, n in [(0, 3), (1, 3), (4, 0), (2, 1), (0, 3)]:
    delta = [2, -2, 4, -4, 6, -6][i]
    assert analysis(many[i].get_frame(n)) == analysis(mv.Analyse(long_super, delta=delta, pnew=0).get_frame(n))
futures = [(node, n, node.get_frame_async(n)) for node, n in [(many[0], 2), (many[1], 3), (many[0], 2), (many[2], 1)]]
for node, n, future in futures:
    assert analysis(future.result()) == analysis(node.get_frame(n))
fails(lambda: mv.AnalyseMany(s, radius=0))
fails(lambda: mv.AnalyseMany(s, radius=2**31 - 1, delta=2))

# Recalculate clamped references, metadata-only input and saved delta.
old = mv.Analyse(s, delta=-1)
recalc = outputs(mv.Recalculate(s, [old, mv.Analyse(s, delta=1)]))
assert len(recalc) == 2
assert analysis(recalc[0].get_frame(0)) == ([0] * 4, [0] * 4)
changed_delta = edit(old, lambda n, f: f.props.update({P + "AnalysisDeltaFrame": 200 if n else -1}))
assert outputs(mv.Recalculate(s, changed_delta))[0].get_frame(1).props[P + "AnalysisDeltaFrame"] == -1
def change_grid(n, f):
    if n:
        f.props[P + "AnalysisNBlkX"] = 1
changed_grid = edit(old, change_grid)
fails(lambda: outputs(mv.Recalculate(s, changed_grid))[0].get_frame(1), "geometry changed")
fails(lambda: mv.Recalculate(s, [old, blank()]), "member[1]")
fails(lambda: mv.Recalculate(s, []))
short = old[:1]
fails(lambda: outputs(mv.Recalculate(s, short))[0].get_frame(2), "requested frame")

# SCDetection: property type/count precedence and directional preservation.
carrier = set_props(blank(vs.RGB24, color=[5, 6, 7]), _SceneChangePrev=7, _SceneChangeNext=[9, 8])
positive = mv.Analyse(s)
scene = mv.SCDetection(carrier, positive)
f = scene.get_frame(0)
assert f.props._SceneChangePrev == 7 and f.props._SceneChangeNext == 0
assert pixels(f) == pixels(carrier.get_frame(0))
negative = mv.SCDetection(carrier, old)
assert negative.get_frame(0).props._SceneChangePrev == 1
assert negative.get_frame(0).props._SceneChangeNext == [9, 8]
fails(lambda: mv.SCDetection(carrier, short).get_frame(2), "requested frame")
def corrupt(n, f):
    if n:
        f.props[P + "AnalysisPel"] = blank()  # Wrong scalar type decodes to zero.
assert mv.SCDetection(carrier, edit(positive, corrupt)).get_frame(1).props._SceneChangeNext == 1
def bad_arrays(n, f):
    if n:
        f.props[P + "AnalysisVectors"] = [blank()] * 4
        f.props[P + "AnalysisSAD"] = [0] * 3
assert mv.SCDetection(carrier, edit(positive, bad_arrays)).get_frame(1).props._SceneChangeNext == 1
def typed_arrays(n, f):
    bad_arrays(n, f)
    if n:
        f.props[P + "AnalysisSAD"] = [0] * 4
fails(lambda: mv.SCDetection(carrier, edit(positive, typed_arrays)).get_frame(1), "integers")
def negative_sad(n, f):
    if n:
        f.props[P + "AnalysisSAD"] = [-1] * 4
fails(lambda: mv.SCDetection(carrier, edit(positive, negative_sad)).get_frame(1), "malformed")
fails(lambda: mv.SCDetection(carrier, edit(positive, change_grid)).get_frame(1), "descriptor")
fails(lambda: mv.SCDetection(carrier, positive, thscd2=float('nan')))

# Frame errors propagate; no successful partial output is published.
broken = core.std.ModifyFrame(blank(), blank(), upstream_error)
fails(lambda: super_clip(broken).get_frame(1), "injected upstream")
nonfinite = edit(blank(vs.GRAYS, color=0.0), lambda n, f: f[0].__setitem__((0, 0), math.inf))
fails(lambda: super_clip(nonfinite).get_frame(1), "non-finite")
def bad_payload(n, f):
    if n:
        f.props[P + "NeoMVSuperPlanesV1"] = values(f.props[P + "NeoMVSuperPlanesV1"])[0]
fails(lambda: mv.Analyse(edit(s, bad_payload)).get_frame(1), "count mismatch")

# Share only deterministic public fixtures with the black-box suite. This path
# uses neo-mv alone and checks literal expected results from the approved spec.
fixture_spec = importlib.util.spec_from_file_location(
    "search_fixtures", Path(__file__).resolve().parents[1] / "blackbox" / "cases.py")
fixtures = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixtures)
for spec in fixtures.SEARCH0_CASES:
    _, nodes = fixtures.build(vs, core, mv, spec)
    expected = spec["expected_block"]
    with nodes[0].get_frame(expected["frame"]) as frame:
        vectors, errors = analysis(frame)
        packed = vectors[expected["index"]]
        x, y = packed & 0xffffffff, (packed >> 32) & 0xffffffff
        x = x - (1 << 32) if x >= 1 << 31 else x
        y = y - (1 << 32) if y >= 1 << 31 else y
        actual = [x, y, errors[expected["index"]]]
        assert actual == expected["result"], (spec["id"], actual, expected["result"])

# 6x6 SAD: all supported sample types, chroma grids, and downstream consumers.
for fmt in (vs.GRAY8, vs.GRAY16, vs.GRAYS, vs.YUV420P8, vs.YUV420P16,
            vs.YUV420PS, vs.YUV422P8, vs.YUV444P8):
    gray = core.get_video_format(fmt).color_family == vs.GRAY
    overlaps = (0, 1, 2, 3) if gray or fmt == vs.YUV444P8 else (0, 2)
    for overlap in overlaps:
        if fmt in (vs.GRAYS, vs.YUV420PS):
            color = [0.25] if gray else [0.25, 0.0, 0.0]
        else:
            color = [17] if gray else [17, 128, 128]
        c6 = blank(fmt, width=32, height=26, length=5, color=color)
        s6 = super_clip(c6, blksize=[6], overlap=[overlap], pad=[16])
        v6 = outputs(mv.AnalyseMany(s6, radius=1, badrange=0))
        r6 = outputs(mv.Recalculate(s6, v6, thsad=0))
        for node in v6 + r6:
            vectors, errors = analysis(node.get_frame(2))
            assert all(x == 0 for x in vectors + errors)
        for node in (mv.Compensate(c6, s6, r6[0]), mv.Degrain(c6, s6, r6),
                     mv.Flow(c6, s6, r6[0]), mv.FlowInter(c6, s6, r6),
                     mv.FlowFPS(c6, s6, r6, num=48, den=1), mv.FlowBlur(c6, s6, r6)):
            assert pixels(node.get_frame(2)) == pixels(c6.get_frame(2))
        for node in (mv.VectorLengthMask(r6[0]), mv.SADMask(r6[0]), mv.OcclusionMask(r6[0])):
            node.get_frame(2)
        # No frame request: these must fail when the filter is constructed.
        fails(lambda: mv.Analyse(s6, metric="satd"), "divisible by 4")
        fails(lambda: mv.AnalyseMany(s6, metric="satd"), "divisible by 4")
        fails(lambda: mv.Recalculate(s6, v6, metric="satd"), "divisible by 4")
# A block-size override must be validated against the requested size, not the Super size.
c8 = blank(width=48, height=48)
s8 = super_clip(c8, blksize=[8], overlap=[0], pad=[16])
v8 = mv.Analyse(s8)
fails(lambda: mv.Analyse(s8, blksize=[6], metric="satd"), "divisible by 4")
fails(lambda: mv.AnalyseMany(s8, blksize=[6], metric="satd"), "divisible by 4")
fails(lambda: mv.Recalculate(s8, v8, blksize=[6], metric="satd"), "divisible by 4")
r6 = outputs(mv.Recalculate(s8, v8, blksize=[6], overlap=[0], thsad=0))[0]
assert r6.get_frame(1).props[P + "AnalysisBlkSizeX"] == 6
for overlap in (1, 3):
    fails(lambda: super_clip(blank(vs.YUV420P8, width=32, height=26, color=[0, 128, 128]),
                             blksize=[6], overlap=[overlap]), "chroma aligned")

print("neo-mv VS acceptance passed: five entry points, ownership, motion, parameters, errors, search=0 fixtures")
blank(length=1).set_output()

for name in ("", "SAD", "1", "unknown"):
    fails(lambda: mv.Analyse(s8, metric=name), "metric must be")
    fails(lambda: mv.AnalyseMany(s8, metric=name), "metric must be")
    fails(lambda: mv.Recalculate(s8, v8, metric=name), "metric must be")
