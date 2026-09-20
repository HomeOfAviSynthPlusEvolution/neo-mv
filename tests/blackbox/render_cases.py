"""Phase-2 public fixtures. No private Super data is inspected or exchanged."""


def case(name, operation="compensate", *, format="GRAY8", pel=1, overlap=0, vector=(0, 0),
         sad=0, deltas=None, params=None, missing=False, pattern="gradient", **extra):
    result = dict(id="render." + name, phase=2, operation=operation, format=format, width=32, height=24,
                length=5, prefix="MVUtensils", members=1, pattern=pattern,
                super_params=dict(blksize=[8], overlap=[overlap], pad=[16], pel=pel, onelevel=True),
                vector=list(vector), sad=sad, deltas=deltas or ([1] if operation == "compensate" else [-1, 1]),
                params=params or {}, missing=missing, requests=[[0, n] for n in [2, 0, 4, 1, 3, 2]])
    result.update(extra)
    return result


CASES = [
    *[case(f"{op}.{fmt}.pel{pel}", op, format=fmt, pel=pel)
      for op in ["compensate", "degrain"]
      for fmt in ["GRAY8", "YUV420P16", "YUV444PS"] for pel in [1, 2, 4]],
    case("compensate.negative_half", pel=2, vector=(-3, -1), params=dict(time=50.0)),
    case("compensate.chroma_floor", format="YUV420P8", pel=2, vector=(-1, -3)),
    case("compensate.quarter_phase", format="YUV420P16", pel=4, vector=(-3, 3), pattern="texture"),
    case("degrain.quarter_phase", "degrain", format="YUV444PS", pel=4, vector=(-3, 3), pattern="texture"),
    case("compensate.zero_delta", deltas=[0], vector=(1, 0)),
    case("compensate.time_zero", params=dict(time=0.0)),
    case("compensate.threshold_equal", sad=100, params=dict(thsad=100)),
    case("compensate.scene", sad=401),
    case("compensate.missing", missing=True),
    case("compensate.fields_tff_false", pel=2, params=dict(fields=True, tff=False, thsad=0)),
    case("compensate.fields_property", pel=2, params=dict(fields=True, thsad=0)),
    case("compensate.overlap", overlap=4, vector=(1, -1)),
    case("compensate.float_overlap", format="YUV444PS", overlap=4, vector=(1, -1)),
    case("degrain.overlap", "degrain", overlap=4, vector=(-1, 1)),
    case("degrain.float_overlap", "degrain", format="YUV444PS", overlap=4, vector=(-1, 1)),
    case("degrain.limit", "degrain", params=dict(limit=[2.2])),
    case("degrain.float_limit", "degrain", format="YUV444PS", params=dict(limit=[0.01])),
    case("degrain.chroma_only", "degrain", format="YUV420P16", params=dict(planes=[1], limit=[1, 2.2])),
    case("degrain.zero_weights", "degrain", params=dict(weights=[0, 0, 0])),
    case("degrain.missing", "degrain", missing=True),
    case("degrain.radius2", "degrain", deltas=[-1, 1, 2, -2],
         params=dict(weights=[1, 2, 3, 4, 5], thsad=[500], thsad2=[100]), sad=50),
    case("degrain.named2", "degrain", deltas=[-1, 1, 2, -2], entry="Degrain2"),
    case("degrain.named25", "degrain", deltas=[d for j in range(1, 26) for d in [-j, j]], entry="Degrain25"),
    *[case(f"{op}.cropped.{fmt}", op, format=fmt, pel=2, overlap=4, width=30, height=22,
           heterogeneous=True, pattern="texture", params=dict(thsad=100) if op == "compensate" else
           dict(thsad=[100, 200], thsad2=[300, 50], limit=[2.2, 3.7]))
      for op in ["compensate", "degrain"] for fmt in ["GRAY8", "YUV420P16", "YUV444PS"]],
    case("degrain.one_missing", "degrain", missing_members=[0]),
    case("degrain.nonuniform_weights", "degrain", deltas=[-1, 1, 2, -2, -3, 3],
         heterogeneous=True, pattern="texture", params=dict(weights=[2, 0, 7, 3, 5, 1, 4],
                                                             thsad=[200], thsad2=[50])),
]


def prepare(vs, core, spec):
    fmt = getattr(vs, spec["format"])
    base = core.std.BlankClip(width=spec["width"], height=spec["height"], format=fmt,
                              length=spec["length"], fpsnum=24000, fpsden=1001)

    def paint(visible):
        def callback(n, f):
            out = f.copy()
            for k in range(out.format.num_planes):
                data = out[k]
                for y in range(data.shape[0]):
                    for x in range(data.shape[1]):
                        value = 5 + k if visible else 48 + 12*n + 8*k + ((3*x + 5*y) % 64)
                        if not visible and spec["pattern"] == "texture":
                            value = (17*x + 29*y + 43*n + 11*k) % 192 + 16
                        if out.format.sample_type == vs.FLOAT:
                            value = (value - (128 if k else 0)) / 256.0
                        else:
                            value <<= out.format.bits_per_sample - 8
                        data[y, x] = value
            out.props["TestMarker"] = [7, 9, n]
            out.props["TestData"] = b"blackbox\x00phase2"
            out.props["TestFloat"] = [1.5, -0.0]
            out.props["_SceneChangePrev"] = 7
            out.props["_SceneChangeNext"] = 9
            out.props["_Field"] = n % 2
            if visible:
                out.props[spec["prefix"] + "AnalysisVectors"] = [123]
            return out
        return callback

    inputs = dict(clip=core.std.ModifyFrame(base, base, paint(True)),
                  super_source=core.std.ModifyFrame(base, base, paint(False)))
    carrier = core.std.BlankClip(width=4, height=4, format=vs.GRAY16, length=spec["length"])
    block, overlap, pad = [spec["super_params"][key][0] for key in ["blksize", "overlap", "pad"]]
    nx = (spec["width"] - overlap + block - overlap - 1) // (block - overlap)
    ny = (spec["height"] - overlap + block - overlap - 1) // (block - overlap)
    dx, dy = spec["vector"]
    packed = (dx & 0xffffffff) | ((dy & 0xffffffff) << 32)
    if packed >= 1 << 63:
        packed -= 1 << 64
    for i, delta in enumerate(spec["deltas"]):
        m = dict(Width=nx*(block-overlap)+overlap, Height=ny*(block-overlap)+overlap,
                 RealWidth=spec["width"], RealHeight=spec["height"], HPad=pad, VPad=pad,
                 Pel=spec["super_params"]["pel"], Levels=1, Chroma=0, XRatioUV=1, YRatioUV=1,
                 BlkSizeX=block, BlkSizeY=block, OverlapX=overlap, OverlapY=overlap,
                 NBlkX=nx, NBlkY=ny, DeltaFrame=delta, BitsPerSample=8)
        props = {spec["prefix"] + "Analysis" + k: v for k, v in m.items()}
        if not spec["missing"] and i not in spec.get("missing_members", []):
            packed_values = [packed]*(nx*ny)
            sad_values = [spec["sad"]]*(nx*ny)
            if spec.get("heterogeneous"):
                for j in range(nx*ny):
                    vx, vy = (j + i) % 3 - 1, (j // nx + i) % 3 - 1
                    v = (vx & 0xffffffff) | ((vy & 0xffffffff) << 32)
                    packed_values[j] = v - (1 << 64) if v >= 1 << 63 else v
                    sad_values[j] = [99, 100, 101][(j + i) % 3]
            props[spec["prefix"] + "AnalysisVectors"] = packed_values
            props[spec["prefix"] + "AnalysisSAD"] = sad_values
        inputs["vectors" + str(i)] = core.std.SetFrameProps(carrier, **props)
    return inputs


def build(plugin, spec, inputs):
    sup = plugin.Super(inputs["super_source"], prefix=spec["prefix"], **spec["super_params"])
    fields = [inputs["vectors" + str(i)] for i in range(len(spec["deltas"]))]
    entry = spec.get("entry", "Compensate" if spec["operation"] == "compensate" else "Degrain")
    out = getattr(plugin, entry)(inputs["clip"], sup, fields[0] if entry == "Compensate" else fields,
                                 prefix=spec["prefix"], **spec["params"])
    return [out]
