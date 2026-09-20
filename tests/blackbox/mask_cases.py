"""Phase-3 mask fixtures made only from public analysis properties."""


def case(name, operation="VectorLengthMask", *, bits=8, params=None, **extra):
    occlusion = operation == "OcclusionMask"
    sad = operation == "SADMask"
    result = dict(id="mask." + name, phase=3, operation=operation, bits=bits,
                  length=3, members=1, prefix="MVUtensils", grid=[2, 1] if occlusion else [1, 1],
                  block=[4, 4] if occlusion else [8, 8], overlap=[0, 0], pel=1,
                  delta=-1 if occlusion else 1, motion=[[4, 0], [0, 0]] if occlusion else [[0, 0] if sad else [4, 0]],
                  sad=[0, 0] if occlusion else [8 << (min(bits, 16) - 8) if sad else 0],
                  params=dict(ml=80 if occlusion else 1 if sad else 8, gamma=1 if occlusion or sad else 2),
                  requests=[[0, n] for n in [2, 0, 1, 2, 0]])
    result["params"].update(params or {})
    result.update(extra)
    return result


CASES = [
    *[case(f"{operation}.bits{bits}", operation, bits=bits)
      for operation in ["VectorLengthMask", "SADMask", "OcclusionMask"] for bits in [8, 10, 16, 32]],
    case("magnitude.prefix", prefix="Other"),
    case("magnitude.time_zero", params=dict(time=0)),
    case("magnitude.halfpel", pel=2, motion=[[8, 0]]),
    *[case(f"sad.projection_time{time}", "SADMask", grid=[3, 1], block=[4, 4],
           motion=[[0, 0], [4, 0], [0, 0]], sad=[0, 4, 8], params=dict(time=time)) for time in [0, 100]],
    case("occlusion.positive", "OcclusionMask", delta=1),
    case("occlusion.time_zero_positive", "OcclusionMask", delta=1, params=dict(time=0)),
    case("occlusion.time_zero_negative", "OcclusionMask", params=dict(time=0)),
    case("occlusion.empty_interval", "OcclusionMask", motion=[[8, 0], [0, 0]]),
    case("occlusion.zero_delta", "OcclusionMask", delta=0),
    case("magnitude.missing", missing="AnalysisVectors", params=dict(scval=12.5)),
    case("sad.float_missing", "SADMask", bits=32, missing="AnalysisSAD", params=dict(scval=1.25)),
    case("occlusion.wrong_length", "OcclusionMask", wrong_length=True, params=dict(scval=-1)),
    case("magnitude.invalid_current", invalid_current=True, params=dict(scval=9)),
    case("sad.levels_vary", "SADMask", levels_vary=True),
    case("magnitude.scene_equal", sad=[400]),
    case("sad.scene_cut", "SADMask", sad=[401], params=dict(scval=9)),
    case("occlusion.cropped", "OcclusionMask", actual=[6, 4]),
    case("magnitude.float_gamma_half", bits=32, params=dict(gamma=0.5)),
    case("sad.float_gamma_half", "SADMask", bits=32, params=dict(gamma=0.5)),
    case("occlusion.float_gamma_half", "OcclusionMask", bits=32, params=dict(ml=160, gamma=0.5)),
]


def prepare(vs, core, spec):
    carrier = core.std.BlankClip(width=4, height=4, format=vs.GRAY16, length=spec["length"],
                                 color=4321, fpsnum=24000, fpsden=1001)
    nx, ny = spec["grid"]
    bx, by = spec["block"]
    ox, oy = spec["overlap"]
    width, height = nx * (bx - ox) + ox, ny * (by - oy) + oy
    real_width, real_height = spec.get("actual", [width, height])
    scalars = dict(Width=width, Height=height, RealWidth=real_width, RealHeight=real_height,
                   HPad=8, VPad=8, Pel=spec["pel"], Levels=1, Chroma=0, XRatioUV=1, YRatioUV=1,
                   BlkSizeX=bx, BlkSizeY=by, OverlapX=ox, OverlapY=oy,
                   NBlkX=nx, NBlkY=ny, DeltaFrame=spec["delta"], BitsPerSample=spec["bits"])
    prefix = spec["prefix"]
    props = {prefix + "Analysis" + key: value for key, value in scalars.items()}
    packed = [(x & 0xffffffff) | ((y & 0xffffffff) << 32) for x, y in spec["motion"]]
    props[prefix + "AnalysisVectors"] = [value if value < (1 << 63) else value - (1 << 64) for value in packed]
    props[prefix + "AnalysisSAD"] = spec["sad"]
    if spec.get("missing"):
        del props[prefix + spec["missing"]]
    if spec.get("wrong_length"):
        props[prefix + "AnalysisVectors"] = [0] * (nx * ny + 1)
    props.update(TestMarker=[7, 9], TestData=b"mask\0input", TestFloat=[1.5, -0.0],
                 _SceneChangePrev=1, _SceneChangeNext=1, _Matrix=1, _ColorRange=0)
    field = core.std.SetFrameProps(carrier, **props)

    def update(n, f):
        out = f.copy()
        out.props["TestMarker"] = [7, 9, n]
        if spec.get("invalid_current") and n == 1:
            out.props[prefix + "AnalysisWidth"] = 0
        if spec.get("levels_vary"):
            out.props[prefix + "AnalysisLevels"] = n + 1
        return out

    return dict(clip=core.std.ModifyFrame(field, field, update))


def build(plugin, spec, inputs):
    return [getattr(plugin, spec["operation"])(inputs["clip"], prefix=spec["prefix"], **spec["params"])]
