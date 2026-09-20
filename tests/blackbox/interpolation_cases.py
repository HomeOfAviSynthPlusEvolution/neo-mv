"""Phase-4 public fixtures; backend-private Super payloads never cross paths."""
import render_cases


def case(name, operation="FlowInter", *, params=None, distance=1, **kwargs):
    arguments = dict(num=0) if operation == "FlowFPS" else {}
    arguments.update(params or {})
    result = render_cases.case(name, operation, deltas=[distance, -distance], params=arguments, **kwargs)
    result.update(id="interpolation." + name, phase=4)
    if operation == "FlowFPS" and "requests" not in kwargs:
        result["requests"] = [[0, n] for n in [5, 0, 9, 2, 1, 8, 5]]
    return result


CASES = [
    *[case(f"{operation}.{fmt}.pel{pel}", operation, format=fmt, pel=pel)
      for operation in ["FlowInter", "FlowFPS", "FlowBlur"]
      for fmt in ["GRAY8", "YUV420P16", "YUV444PS"] for pel in [1, 2, 4]],
    case("inter.gray10", format="GRAY10", pel=2),
    case("inter.negative_half", pel=2, direction_vectors=[[-3, -1], [3, 1]], params=dict(time=37.5)),
    case("inter.occlusion_extra", format="YUV420P16", pel=2, convergence=True,
         pattern="texture", params=dict(ml=80)),
    case("inter.basic_absent_extra", convergence=True, pattern="texture", params=dict(ml=80),
         missing_fields=[[0, 3]], requests=[[0, 2], [0, 2]]),
    case("inter.missing_blend", missing=True, params=dict(time=37.5)),
    case("inter.missing_copy", missing=True, params=dict(time=37.5, blend=False)),
    case("inter.time_zero", direction_vectors=[[3, -1], [-3, 1]], params=dict(time=0)),
    case("inter.time_end", format="YUV444PS", pel=4,
         direction_vectors=[[-3, 3], [3, -3]], params=dict(time=100)),
    case("inter.distance2", distance=2, direction_vectors=[[2, 0], [-2, 0]]),
    case("inter.prefix", prefix="Other", pel=2, direction_vectors=[[1, -1], [-1, 1]]),
    case("fps.fractional_rate", "FlowFPS", params=dict(num=30, den=1),
         direction_vectors=[[3, 0], [-3, 0]], requests=[[0, n] for n in [1, 0, 5, 2, 4, 1]]),
    case("fps.double_den_zero", "FlowFPS", params=dict(num=60, den=0)),
    case("fps.distance2", "FlowFPS", distance=2, pel=2, direction_vectors=[[3, -1], [-3, 1]]),
    case("fps.extra_disabled", "FlowFPS", convergence=True, pattern="texture",
         params=dict(extramask=False, ml=80)),
    case("fps.scene_fallback", "FlowFPS", sad=401, params=dict(blend=True)),
    case("fps.rounded_right_endpoint", "FlowFPS", width=8, height=8, length=2, fps=[1, 1],
         missing=True, params=dict(num=1000, den=1),
         requests=[[0, n] for n in [999, 0, 1999, 1, 1000, 999]]),
    case("blur.asymmetric", "FlowBlur", direction_vectors=[[-4, -2], [2, 0]],
         pattern="texture", params=dict(blur=200)),
    case("blur.negative_quarter", "FlowBlur", format="YUV444PS", pel=4,
         direction_vectors=[[-7, -3], [5, 1]], pattern="texture", params=dict(blur=125)),
    case("blur.zero", "FlowBlur", direction_vectors=[[-4, 0], [4, 0]], params=dict(blur=0)),
    case("blur.full", "FlowBlur", format="YUV420P16", pel=2,
         direction_vectors=[[-8, -4], [8, 4]], params=dict(blur=200)),
    case("blur.coarse_precision", "FlowBlur", direction_vectors=[[-7, 0], [7, 0]],
         params=dict(blur=200, prec=3)),
    case("blur.missing", "FlowBlur", missing=True, params=dict(blur=0)),
]


def packed_vector(x, y):
    value = (x & 0xffffffff) | ((y & 0xffffffff) << 32)
    return value - (1 << 64) if value >= 1 << 63 else value


def prepare(vs, core, spec):
    inputs = render_cases.prepare(vs, core, spec)
    prefix = spec["prefix"]

    def visible(n, f):
        out = f.copy()
        for k in range(out.format.num_planes):
            data = out[k]
            for y in range(data.shape[0]):
                for x in range(data.shape[1]):
                    value = 8 + 7*n + 3*k + (x + y) % 8
                    if out.format.sample_type == vs.FLOAT:
                        value = (value - (128 if k else 0)) / 256.0
                    else:
                        value <<= out.format.bits_per_sample - 8
                    data[y, x] = value
        out.props["_DurationNum"] = 7
        out.props["_DurationDen"] = 13
        out.props["_Range"] = 1
        out.props["_Matrix"] = 1
        return out

    inputs["clip"] = core.std.ModifyFrame(inputs["clip"], inputs["clip"], visible)
    if "fps" in spec:
        numerator, denominator = spec["fps"]
        for name in ["clip", "super_source"]:
            inputs[name] = core.std.AssumeFPS(inputs[name], fpsnum=numerator, fpsden=denominator)

    def update(direction):
        def callback(n, f):
            out = f.copy()
            vector_key, sad_key = prefix + "AnalysisVectors", prefix + "AnalysisSAD"
            if [direction, n] in spec.get("missing_fields", []):
                if vector_key in out.props:
                    del out.props[vector_key]
                if sad_key in out.props:
                    del out.props[sad_key]
                return out
            if spec["missing"] or direction in spec.get("missing_members", []):
                return out
            if spec.get("direction_vectors") or spec.get("convergence"):
                nx = out.props[prefix + "AnalysisNBlkX"]
                ny = out.props[prefix + "AnalysisNBlkY"]
                vectors = []
                for y in range(ny):
                    for x in range(nx):
                        if spec.get("convergence"):
                            # Both directions converge; their event intervals
                            # differ through the public signed DeltaFrame.
                            vx = 4 if x % 2 == 0 else 0
                            vy = 2 if y % 2 == 0 else 0
                        else:
                            vx, vy = spec["direction_vectors"][direction]
                        vectors.append(packed_vector(vx, vy))
                out.props[vector_key] = vectors
            return out
        return callback

    for direction in range(2):
        name = "vectors" + str(direction)
        inputs[name] = core.std.ModifyFrame(inputs[name], inputs[name], update(direction))
    return inputs


def build(plugin, spec, inputs):
    reference = plugin.Super(inputs["super_source"], prefix=spec["prefix"], **spec["super_params"])
    pair = [inputs["vectors0"], inputs["vectors1"]]
    return [getattr(plugin, spec["operation"])(inputs["clip"], reference, pair,
                                              prefix=spec["prefix"], **spec["params"])]
