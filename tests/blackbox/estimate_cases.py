"""Phase-6 public image fixtures; no private spectra or block-vector inputs.

Exact raw comparisons intentionally retain FFT-library rounding differences.
The fixture generates inputs only; it does not manufacture motion properties.
"""

REMOVED_KEYS = ("DepanEstimateFFT", "DepanEstimateFFT2", "DepanEstimateX", "DepanEstimateY",
                "DepanEstimateZoom", "DepanEstimateGood", "DepanEstimateTrust")
PRESERVED_KEYS = ("DepanEstimateFFT3", "DepanEstimateTrustExtra", "DepanEstimateX_extra")
OBSERVATION_KEYS = REMOVED_KEYS + PRESERVED_KEYS + ("DepanEstimate_info", "_ChromaLocation")


def case(name, *, format="GRAY8", params=None, **extra):
    arguments = dict(winx=4, winy=4, wleft=0, wtop=0, dxmax=1, dymax=1,
                     stab=0.0, trust=4.0, zoommax=1.0)
    arguments.update(params or {})
    result = dict(id="estimate." + name, phase=6, operation="DepanEstimate", format=format,
                  width=4, height=4, length=3, prefix="MVUtensils", members=1,
                  params=arguments, requests=[[0, n] for n in [2, 0, 1, 2, 1, 0]],
                  pattern="impulse", rectangles=[[0, 0, 4, 4]])
    result.update(extra)
    return result


CASES = [
    # The public 4x4 three-frame sign example: x=2,1,0, integer amplitude one.
    *[case("impulse." + fmt, format=fmt)
      for fmt in ["GRAY8", "YUV420P10", "YUV422P16", "YUV444P8", "YUV440P8", "GRAYS"]],
    case("odd_height", height=3, params=dict(winy=3, dymax=0), rectangles=[[0, 0, 4, 3]]),
    case("placed_window", width=12, height=8, params=dict(wleft=3, wtop=2),
         rectangles=[[3, 2, 4, 4]]),
    case("two_windows", width=16, height=8, params=dict(winx=8, wleft=1, wtop=2, zoommax=1.1),
         rectangles=[[1, 2, 4, 4], [9, 2, 4, 4]], second_stationary=True),
    case("two_windows_show", width=16, height=8, params=dict(winx=8, wleft=1, wtop=2, zoommax=1.5, show=True),
         rectangles=[[1, 2, 4, 4], [9, 2, 4, 4]], second_stationary=True),
    case("fields_property", params=dict(fields=True)),
    case("fields_tff", params=dict(fields=True, tff=True, pixaspect=2.0), omit_field=True),
    case("fields_bff", params=dict(fields=True, tff=False)),
    case("fields_missing_error", params=dict(fields=True, trust=100.0), omit_field=True),
    # Zero search has exactly P=M, so confidence=0 equals trust and passes.
    case("trust_zero_equality", params=dict(dxmax=0, dymax=0, trust=0.0), stationary=True),
    case("zero_image", pattern="zero", params=dict(trust=0.0)),
    case("show_integer", params=dict(show=True)),
    case("show_float", format="GRAYS", params=dict(show=True)),
    case("show_constant_error", pattern="zero", params=dict(show=True, trust=0.0)),
    # Host text pixels follow the existing Phase-5 renderer-provenance policy.
    # No fixture-side pixel substitution or numerical tolerance is applied.
    case("info", width=320, height=48, params=dict(info=True)),
    case("unused_nan_copy", format="GRAYS", width=8, nan_position=[7, 3]),
    case("used_nan_error", format="GRAYS", width=8, nan_position=[0, 3]),
    # Deliberately nontrivial transforms retain library differences in reports.
    case("gradient", width=8, height=6, pattern="gradient",
         params=dict(winx=8, winy=6, dxmax=2, dymax=1), rectangles=[[0, 0, 8, 6]]),
    case("gradient_float_show", format="GRAYS", width=8, height=6, pattern="gradient",
         params=dict(winx=8, winy=6, dxmax=2, dymax=1, show=True), rectangles=[[0, 0, 8, 6]]),
    case("single_frame", length=1, requests=[[0, 0], [0, 0]]),
]


def prepare(vs, core, spec):
    clip = core.std.BlankClip(width=spec["width"], height=spec["height"], format=getattr(vs, spec["format"]),
                              length=spec["length"], fpsnum=24000, fpsden=1001)

    def paint(n, f):
        out = f.copy()
        is_float = out.format.sample_type == vs.FLOAT
        for plane_index in range(out.format.num_planes):
            data = out[plane_index]
            for y in range(data.shape[0]):
                for x in range(data.shape[1]):
                    if plane_index:
                        value = (13*x + 19*y + 23*n + 31*plane_index) % 192 + 16
                        value = ((value - 128) / 256.0 if is_float else
                                 value << (out.format.bits_per_sample - 8))
                    elif spec["pattern"] == "gradient":
                        shifted = (x + n) % spec["width"]
                        value = (17*shifted + 29*y + 7*shifted*y + 3*n) % 193
                        if is_float:
                            value /= 256.0
                    else:
                        value = 0.0 if is_float else 0
                    data[y, x] = value
        if spec["pattern"] == "impulse":
            for index, (left, top, width, height) in enumerate(spec["rectangles"]):
                stationary = spec.get("stationary") or (index == 1 and spec.get("second_stationary"))
                x = 2 if stationary else (2 - n) % width
                # Unequal amplitudes expose independent two-surface display scaling.
                out[0][top, left + x] = (1.0 if is_float else 1) * (index + 1)
        if "nan_position" in spec:
            x, y = spec["nan_position"]
            out[0][y, x] = float("nan")
        out.props["TestMarker"] = [7, 9, n]
        out.props["TestData"] = b"blackbox\x00phase6"
        out.props["TestFloat"] = [1.5, -0.0]
        out.props["_Range"] = 1
        out.props["_Matrix"] = 1
        out.props["_ChromaLocation"] = 2
        out.props["_SceneChangePrev"] = 7
        out.props["_SceneChangeNext"] = 9
        out.props["_DurationNum"] = 7
        out.props["_DurationDen"] = 13
        if not spec.get("omit_field"):
            out.props["_Field"] = 1 - n % 2
        # Replacement is unconditional, even for inherited wrong native types.
        out.props["Depan_dx"] = b"stale dx"
        out.props["Depan_dy"] = [13.0, -17.0]
        out.props["Depan_rot"] = [3, 5]
        out.props["Depan_zoom"] = b"stale zoom"
        out.props["Depan_goodmotion"] = [7, n]
        for index, key in enumerate(REMOVED_KEYS):
            out.props[key] = (b"stale\x00not-a-spectrum" if index % 3 == 0 else
                              [11, 13, n] if index % 3 == 1 else [1.25, -0.0])
        out.props["DepanEstimateFFT3"] = b"preserve FFT3"
        out.props["DepanEstimateTrustExtra"] = [19, n]
        out.props["DepanEstimateX_extra"] = [0.5, -0.0]
        out.props["DepanEstimate_info"] = f"inherited estimate {n}".encode("ascii")
        return out

    return {"clip": core.std.ModifyFrame(clip, clip, paint)}


def build(plugin, spec, inputs):
    return [plugin.DepanEstimate(inputs["clip"], **spec["params"])]
