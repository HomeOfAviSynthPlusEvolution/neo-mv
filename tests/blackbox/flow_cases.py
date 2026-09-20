"""Public Flow fixtures; each backend builds and consumes its own Super."""
import render_cases


def case(name, *, delta=1, pad=16, **kwargs):
    result = render_cases.case(name, "Flow", deltas=[delta], **kwargs)
    result.update(id="flow." + name, phase=3)
    result["super_params"]["pad"] = [pad]
    return result


CASES = [
    *[case(f"{fmt}.pel{pel}", format=fmt, pel=pel)
      for fmt in ["GRAY8", "GRAY10", "YUV420P16", "YUV444PS"] for pel in [1, 2, 4]],
    case("negative_half_time", pel=2, vector=(-3, -1), params=dict(time=50)),
    case("float_quarter_phase", format="YUV444PS", pel=4, vector=(-3, 3), pattern="texture"),
    case("chroma_floor", format="YUV420P8", pel=2, vector=(-1, -3)),
    case("cropped_overlap", format="YUV420P16", pel=2, overlap=4, width=30, height=22,
         heterogeneous=True, pattern="texture"),
    case("time_zero", vector=(8, -8), params=dict(time=0)),
    case("zero_delta", delta=0, vector=(1, -1)),
    case("temporal_fallback_without_parity", delta=99, pel=2, field_property="missing", params=dict(fields=True)),
    case("missing_arrays", missing=True),
    case("scene_cut", sad=401),
    case("fields_tff_false", pel=2, params=dict(fields=True, tff=False, time=50)),
    case("fields_tff_true", pel=2, params=dict(fields=True, tff=True, time=50)),
    case("fields_property", pel=2, params=dict(fields=True)),
    case("fields_pel1_inactive", field_property="missing", params=dict(fields=True)),
    case("odd_padding_zero", format="YUV420P8", pel=2, pad=3),
    case("odd_padding_time_zero", format="YUV420P8", pel=2, pad=3, vector=(-6, 0), params=dict(time=0)),
    case("invalid_current", invalid_current=True),
    case("levels_vary", levels_vary=True),
    case("prefix", prefix="Other", pel=2, vector=(1, -1)),
]


def prepare(vs, core, spec):
    inputs = render_cases.prepare(vs, core, spec)
    # Flow preserves the clip property map. Range must be compared literally:
    # the existing mask-only range exception does not apply to this operation.
    inputs["clip"] = core.std.SetFrameProps(inputs["clip"], _Range=1, _Matrix=1)
    if spec.get("field_property") == "missing":
        inputs["super_source"] = core.std.RemoveFrameProps(inputs["super_source"], props=["_Field"])
    if spec.get("invalid_current") or spec.get("levels_vary"):
        source = inputs["vectors0"]
        prefix = spec["prefix"]

        def update(n, f):
            out = f.copy()
            if spec.get("invalid_current") and n == 2:
                out.props[prefix + "AnalysisWidth"] = 0
            if spec.get("levels_vary"):
                out.props[prefix + "AnalysisLevels"] = n + 1
            return out

        inputs["vectors0"] = core.std.ModifyFrame(source, source, update)
    return inputs


def build(plugin, spec, inputs):
    reference = plugin.Super(inputs["super_source"], prefix=spec["prefix"], **spec["super_params"])
    return [plugin.Flow(inputs["clip"], reference, inputs["vectors0"], prefix=spec["prefix"], **spec["params"])]
