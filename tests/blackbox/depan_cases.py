"""Phase-5 fixtures built exclusively from public Analysis and Depan properties."""

MASK_FAILURE_SENTINEL = "blackbox mask dependency failure"


def case(name, operation="DepanAnalyse", *, format="GRAY8", params=None, **extra):
    defaults = dict(zoom=False, rot=False) if operation == "DepanAnalyse" else dict(offset=1, subpixel=2)
    defaults.update(params or {})
    result = dict(id="depan." + name, phase=5, operation=operation, format=format,
                  width=48, height=48, length=5, prefix="MVUtensils", members=1,
                  params=defaults, requests=[[0, n] for n in [2, 0, 4, 1, 3, 2]],
                  grid=[2, 2], delta=-1, mask="one", missing=False, sad=0,
                  motion=[2.0, -1.0, 0.0, 1.0, 1])
    result.update(extra)
    return result


CASES = [
    *[case(f"analyse.{fmt}.delta{delta}", format=fmt, delta=delta)
      for fmt in ["GRAY8", "YUV420P16", "RGB24", "RGBS", "GRAYS"] for delta in [-1, 1]],
    case("analyse.zero_mask", mask="zero"),
    case("analyse.weight128", mask="weighted"),
    case("analyse.no_mask_border", width=80, height=80, grid=[9, 9], mask=None),
    case("analyse.no_mask_small", mask=None),
    case("analyse.outside_mask", width=16, height=16, grid=[3, 3], mask="zero"),
    case("analyse.zoom", params=dict(zoom=True)),
    case("analyse.rotation", params=dict(rot=True)),
    case("analyse.zoom_rotation", params=dict(zoom=True, rot=True)),
    case("analyse.aspect", params=dict(pixaspect=1.5, zoom=True, rot=True)),
    case("analyse.fields_tff", params=dict(fields=True, tff=True)),
    case("analyse.fields_property", params=dict(fields=True)),
    case("analyse.missing_negative_error", missing=True, params=dict(error=-1)),
    case("analyse.missing", missing=True),
    case("analyse.scene", sad=401),
    case("analyse.temporal_positive", delta=1, frame_vectors=True),
    case("analyse.temporal_negative", delta=-1, frame_vectors=True),
    case("analyse.info", params=dict(info=True)),
    # Frame zero retains the creation descriptor. Later frames are independent
    # public fields; the filter must retain only creation d/T1/T2.
    *[case(f"analyse.current_delta_{label}", current_metadata=dict(DeltaFrame=value), frame_vectors=True)
      for label, value in [("positive", 1), ("zero", 0)]],
    case("analyse.current_delta_missing", current_missing=["DeltaFrame"], frame_vectors=True),
    case("analyse.current_delta_saved_positive", delta=1,
         current_metadata=dict(DeltaFrame=-1), frame_vectors=True),
    case("analyse.current_hpad", current_metadata=dict(HPad=17)),
    case("analyse.current_pel", current_metadata=dict(Pel=4)),
    case("analyse.current_pel_missing", current_missing=["Pel"], current_sad=[-1]*4),
    case("analyse.current_grid", current_metadata=dict(NBlkX=3, Width=24, RealWidth=24)),
    case("analyse.current_grid_short_arrays", current_metadata=dict(NBlkX=3, Width=24, RealWidth=24),
         current_array_count=4),
    case("analyse.current_grid_saved_t2", current_metadata=dict(NBlkX=3, Width=24, RealWidth=24),
         current_sad=[401, 401, 401, 0, 0, 0]),
    case("analyse.current_depth_saved_t1", current_metadata=dict(BitsPerSample=10), current_sad=[500]*4),
    case("analyse.ineligible_mask_error", missing=True, input_error_frames=dict(mask=[2]),
         expected_output_errors=[dict(member=0, frame=2, sentinel=MASK_FAILURE_SENTINEL)]),
    case("analyse.scene_mask_error", sad=401, input_error_frames=dict(mask=[2]),
         expected_output_errors=[dict(member=0, frame=2, sentinel=MASK_FAILURE_SENTINEL)]),
    *[case(f"compensate.{fmt}.mode{mode}", "DepanCompensate", format=fmt,
           params=dict(subpixel=mode), width=16, height=12)
      for fmt in ["GRAY8", "YUV420P10", "YUV444P16"] for mode in [0, 1, 2]],
    *[case(f"compensate.offset{offset}", "DepanCompensate", params=dict(offset=offset),
           frame_motion=True, width=16, height=12)
      for offset in [-1, 1.5, -1.5, 2.5]],
    case("compensate.aspect", "DepanCompensate", params=dict(pixaspect=1.5), width=16, height=12),
    case("compensate.fields_tff", "DepanCompensate", params=dict(fields=True, tff=False), width=16, height=12),
    case("compensate.fields_property", "DepanCompensate", params=dict(fields=True), width=16, height=12),
    case("compensate.no_matchfields", "DepanCompensate", params=dict(fields=True, matchfields=False),
         omit_field=True, width=16, height=12),
    case("compensate.invalid_identity", "DepanCompensate", motion=[3.0, 2.0, 0.0, 1.0, 0],
         params=dict(subpixel=1), width=16, height=12),
    case("compensate.rotation_zoom", "DepanCompensate", motion=[0.5, -0.5, 3.0, 1.02, 1],
         params=dict(offset=-1.5), frame_motion=True, width=16, height=12),
    case("compensate.mirror", "DepanCompensate", motion=[7.0, -3.0, 0.0, 1.0, 1],
         params=dict(mirror=15), width=16, height=12),
    case("compensate.blur_left", "DepanCompensate", format="YUV420P10", motion=[-2.5, 0.0, 0.0, 1.0, 1],
         params=dict(subpixel=1, mirror=4, blur=3), width=16, height=12),
    case("compensate.blur_right", "DepanCompensate", motion=[2.5, 0.0, 0.0, 1.0, 1],
         params=dict(subpixel=1, mirror=8, blur=3), width=16, height=12),
    case("compensate.tiny_cubic", "DepanCompensate", width=1, height=2),
    case("compensate.tiny_nearest", "DepanCompensate", width=1, height=1, params=dict(subpixel=0)),
    case("compensate.tiny_bilinear", "DepanCompensate", width=1, height=1, params=dict(subpixel=1)),
    case("compensate.info", "DepanCompensate", params=dict(info=True)),
    case("compensate.zero_bypass", "DepanCompensate", params=dict(offset=0), missing=True),
    case("compensate.invalid_stops_reads", "DepanCompensate", params=dict(offset=1.5),
         invalid_frames=[1], malformed_frames=[2], requests=[[0, 2], [0, 2]]),
]


def packed_vector(x, y):
    value = (x & 0xffffffff) | ((y & 0xffffffff) << 32)
    return value - (1 << 64) if value >= 1 << 63 else value


def prepare(vs, core, spec):
    clip = core.std.BlankClip(width=spec["width"], height=spec["height"], format=getattr(vs, spec["format"]),
                              length=spec["length"], fpsnum=24000, fpsden=1001)

    def paint(n, f):
        out = f.copy()
        for k in range(out.format.num_planes):
            data = out[k]
            for y in range(data.shape[0]):
                for x in range(data.shape[1]):
                    value = (17*x + 29*y + 31*n + 11*k) % 192 + 16
                    if out.format.sample_type == vs.FLOAT:
                        value = (value - (128 if k and out.format.color_family == vs.YUV else 0)) / 256.0
                    else:
                        value <<= out.format.bits_per_sample - 8
                    data[y, x] = value
        out.props["TestMarker"] = [7, 9, n]
        out.props["TestData"] = b"blackbox\x00phase5"
        out.props["TestFloat"] = [1.5, -0.0]
        out.props["_Range"] = 1
        out.props["_Matrix"] = 1
        out.props["_SceneChangePrev"] = 7
        out.props["_SceneChangeNext"] = 9
        out.props["_DurationNum"] = 7
        out.props["_DurationDen"] = 13
        if not spec.get("omit_field"):
            out.props["_Field"] = 1 - n % 2
        out.props["Depan_dx"] = [1000.0 + n, -3.0]
        out.props["Depan_dy"] = 4.0
        out.props["Depan_rot"] = 0.0
        out.props["Depan_zoom"] = 1.0
        out.props["Depan_goodmotion"] = [7, n]
        out.props["DepanAnalyse_info"] = f"inherited analysis {n}".encode("ascii")
        out.props["DepanCompensate_info"] = f"inherited compensation {n}".encode("ascii")
        return out

    inputs = {"clip": core.std.ModifyFrame(clip, clip, paint)}
    # The carrier deliberately differs from the copied/rendered video. These
    # pixels are recorded as input evidence but are never filter operands.
    carrier = core.std.BlankClip(width=4, height=2, format=vs.RGBS, length=spec["length"], fpsnum=1, fpsden=1)
    if spec["operation"] == "DepanAnalyse":
        nx, ny = spec["grid"]
        metadata = dict(Width=nx*8, Height=ny*8, RealWidth=nx*8, RealHeight=ny*8,
                        HPad=16, VPad=16, Pel=2, Levels=1, Chroma=0, XRatioUV=1, YRatioUV=1,
                        BlkSizeX=8, BlkSizeY=8, OverlapX=0, OverlapY=0, NBlkX=nx, NBlkY=ny,
                        DeltaFrame=spec["delta"], BitsPerSample=8)
        source = core.std.SetFrameProps(carrier, **{"MVUtensilsAnalysis" + key: value
                                                   for key, value in metadata.items()})

        def vectors(n, f):
            out = f.copy()
            current = dict(metadata)
            if n != 0:
                current.update(spec.get("current_metadata", {}))
                for key, value in current.items():
                    out.props["MVUtensilsAnalysis" + key] = value
                for key in spec.get("current_missing", []):
                    del out.props["MVUtensilsAnalysis" + key]
            if not spec["missing"]:
                pattern = [(1, -3), (0, 0), (4, 2), (-2, 1)]
                values = []
                count = current["NBlkX"] * current["NBlkY"]
                if n != 0:
                    count = spec.get("current_array_count", count)
                for i in range(count):
                    x, y = pattern[i % len(pattern)]
                    if spec.get("frame_vectors"):
                        x += n
                        y -= n
                    values.append(packed_vector(x, y))
                out.props["MVUtensilsAnalysisVectors"] = values
                out.props["MVUtensilsAnalysisSAD"] = (spec.get("current_sad", [spec["sad"]]*count)
                                                       if n != 0 else [spec["sad"]]*count)
            out.props["_Field"] = n % 2  # Opposite carrier parity must not select the output parity.
            return out

        inputs["vectors"] = core.std.ModifyFrame(source, source, vectors)
        if spec["mask"] is not None:
            value = {"zero": 0, "one": 1, "weighted": 128}[spec["mask"]]
            inputs["mask"] = core.std.BlankClip(width=spec["width"], height=spec["height"], format=vs.GRAY8,
                                                length=spec["length"], color=[value])
            if spec.get("input_error_frames", {}).get("mask"):
                def failing_mask(n, f):
                    if n in spec["input_error_frames"]["mask"]:
                        raise vs.Error(MASK_FAILURE_SENTINEL)
                    return f.copy()
                mask = inputs["mask"]
                inputs["mask"] = core.std.ModifyFrame(mask, mask, failing_mask)
    else:
        def motion(n, f):
            out = f.copy()
            if spec["missing"]:
                return out
            dx, dy, rotation, zoom, good = spec["motion"]
            if spec.get("frame_motion"):
                dx += n * 0.25
                dy -= n * 0.125
            if n in spec.get("invalid_frames", []):
                good = 0
            for name, value in zip(["dx", "dy", "rot", "zoom"], [dx, dy, rotation, zoom]):
                out.props["Depan_" + name] = [float(value), -17.0]
            out.props["Depan_goodmotion"] = [good, 0]
            if n in spec.get("malformed_frames", []):
                del out.props["Depan_rot"]
            return out

        inputs["data"] = core.std.ModifyFrame(carrier, carrier, motion)
    return inputs


def build(plugin, spec, inputs):
    if spec["operation"] == "DepanAnalyse":
        arguments = dict(spec["params"])
        if "mask" in inputs:
            arguments["mask"] = inputs["mask"]
        return [plugin.DepanAnalyse(inputs["clip"], inputs["vectors"], **arguments)]
    return [plugin.DepanCompensate(inputs["clip"], inputs["data"], **spec["params"])]
