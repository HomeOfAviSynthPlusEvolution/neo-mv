"""Deterministic public-API fixtures from approved docs/specs/phase-1.

This module does not import either implementation. Super payloads are never
interchanged or decoded. Case IDs and request order are part of the report.
"""

PREFIX = "MVUtensils"
SUPER_KEYS = tuple("Super" + name for name in (
    "Width", "Height", "RealWidth", "RealHeight", "HPad", "VPad", "Pel", "Levels", "Chroma",
    "BlkSizeX", "BlkSizeY", "OverlapX", "OverlapY", "BitsPerSample",
    "XRatioUV", "YRatioUV"))
ANALYSIS_KEYS = tuple("Analysis" + name for name in (
    "Width", "Height", "RealWidth", "RealHeight", "HPad", "VPad", "Pel",
    "Levels", "Chroma", "XRatioUV", "YRatioUV", "BlkSizeX", "BlkSizeY",
    "OverlapX", "OverlapY", "NBlkX", "NBlkY", "DeltaFrame", "BitsPerSample",
    "Vectors", "SAD"))


def case(case_id, operation, *, format="GRAY8", pattern="constant", pel=1,
         params=None, members=1, super_params=None):
    return dict(id=case_id, operation=operation, format=format, pattern=pattern,
                width=32, height=24, length=5, prefix=PREFIX,
                super_params={**dict(blksize=[8], overlap=[0], pad=[16], pel=pel,
                                     onelevel=True), **(super_params or {})},
                params=params or {}, members=members,
                # Includes first/last frames, nonsequential and duplicate reads.
                requests=[[member, frame] for frame in [2, 0, 4, 2]
                          for member in range(members)])


CASES = [
    case("super.gray8.pel1", "super"),
    case("super.yuv420p16.pel4", "super", format="YUV420P16", pel=4),
    case("super.yuv444ps.pel2", "super", format="YUV444PS", pel=2),
    *[case(f"analyse.translation.search{mode}", "analyse", pattern="translation",
           params=dict(delta=1, search=mode, pelsearch=2, pnew=0,
                       mvlambda=0, chroma=False)) for mode in range(6)],
    case("analyse.yuv420p16.pel2", "analyse", format="YUV420P16", pel=2),
    case("analyse.yuv420p8.pel2", "analyse", format="YUV420P8", pel=2),
    case("analyse.float.pel4", "analyse", format="YUV444PS", pel=4),
    case("many.radius2", "many", params=dict(radius=2, delta=1), members=4),
    case("recalculate.chain", "recalculate_chain", members=2),
    case("recalculate.public", "recalculate_public", params=dict(thsad=0)),
    case("scene.chain", "scene_chain"),
    case("scene.public", "scene_public", params=dict(thscd1=0, thscd2=50.0)),
]


def search0_translation(operation, distance):
    params = dict(search=0, mvlambda=0, pnew=0, chroma=False)
    if operation == "analyse":
        params.update(delta=1, pelsearch=distance)
    else:
        params.update(thsad=0, smooth=False, searchparam=distance, fields=False, satd=False)
    spec = case(f"{operation}.translation.search0.range{distance}", operation,
                pattern="translation", params=params)
    spec["public_field"] = dict(delta=1, seed=[0, 0])
    spec["expected_block"] = dict(frame=2, index=0,
                                  result=[8, 8, 64] if distance == 4 else [-1, 0, 294])
    return spec


# Approved search=0 image examples also serve as oracle-free host assertions.
SEARCH0_CASES = [search0_translation(op, distance)
                 for op in ["analyse", "recalculate_public"] for distance in range(1, 5)]


def search0_image(name, size, pad, seed, block, expected, *, patch=None, row=None):
    spec = case(f"recalculate.search0.{name}", "recalculate_public", pattern="search0_image",
                params=dict(thsad=0, smooth=False, search=0, searchparam=1, mvlambda=0,
                            pnew=0, chroma=False, meander=False, fields=False, satd=False),
                super_params=dict(blksize=[4], pad=[pad]))
    spec.update(width=size, height=size, length=2, requests=[[0, n] for n in [0, 1, 0]],
                public_field=dict(delta=1, seed=seed), reference_patch=patch, reference_row=row,
                expected_block=dict(frame=0, index=block, result=expected))
    return spec


SEARCH0_CASES += [
    search0_image("axial_diagonal", 16, 16, [4, 4], 0, [4, 3, 189], patch=[
        [11, 5, 15, 20, 5, 15], [8, 5, 7, 20, 11, 20], [9, 11, 16, 19, 3, 9],
        [19, 19, 19, 11, 3, 0], [18, 15, 11, 19, 17, 20], [2, 11, 8, 9, 7, 13]]),
    search0_image("diagonal_tie", 16, 16, [4, 4], 0, [5, 5, 30], patch=[
        [0, 10, 0, 0, 10, 0], [10, 10, 0, 0, 10, 10], [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0], [10, 10, 0, 0, 10, 10], [0, 10, 0, 0, 10, 0]]),
    search0_image("axial_tie", 16, 16, [4, 4], 0, [5, 4, 40],
                  row=[100, 100, 100, 0, 10, 0, 0, 10, 0, 100, 100, 100, 100, 100, 100, 100]),
    search0_image("upper_bound", 8, 1, [0, 0], 3, [0, 0, 40],
                  row=[100, 100, 100, 100, 10, 0, 0, 0]),
    search0_image("lower_bound", 8, 1, [0, 0], 0, [-1, 0, 80],
                  row=[0, 10, 10, 10, 100, 100, 100, 100]),
]
CASES += SEARCH0_CASES
BY_ID = {item["id"]: item for item in CASES}


def build(vs, core, plugin, spec):
    """Build a backend-local graph and return its source and output members."""
    fmt = getattr(vs, spec["format"])
    src = core.std.BlankClip(width=spec["width"], height=spec["height"],
                             format=fmt, length=spec["length"],
                             fpsnum=24000, fpsden=1001)

    def paint(n, f):
        result = f.copy()
        for plane in range(result.format.num_planes):
            data = result[plane]
            for y in range(data.shape[0]):
                for x in range(data.shape[1]):
                    if spec["pattern"] == "translation":
                        value = ((x + n) * 17 + y * 29) % 192 + 16
                    elif spec["pattern"] == "search0_image":
                        value = 0
                        if n == 1:
                            patch, row = spec["reference_patch"], spec["reference_row"]
                            value = row[x] if row is not None else 100
                            if patch is not None and 3 <= x <= 8 and 3 <= y <= 8:
                                value = patch[y - 3][x - 3]
                    else:
                        value = 32 + plane * 16
                    if result.format.sample_type == vs.FLOAT:
                        value = value / 256.0 if plane == 0 else (value - 128) / 256.0
                    else:
                        value <<= result.format.bits_per_sample - 8
                    data[y, x] = value
        result.props["TestMarker"] = [7, 9]
        result.props["_SceneChangePrev"] = 7
        result.props["_SceneChangeNext"] = 9
        return result

    src = core.std.ModifyFrame(src, src, paint)
    op, params = spec["operation"], spec["params"]
    prefix = spec["prefix"]

    def public_field():
        # Public, typed Analysis fixture; deliberately contains no Super payload.
        # These fixtures use GRAY8, square blocks, zero overlap and one level.
        assert spec["format"] == "GRAY8" and spec["super_params"]["overlap"] == [0]
        block = spec["super_params"]["blksize"][0]
        pad = spec["super_params"]["pad"][0]
        width, height = spec["width"], spec["height"]
        field = spec.get("public_field")
        metadata = dict(Width=width, Height=height, RealWidth=width, RealHeight=height,
                        HPad=pad, VPad=pad, Pel=spec["super_params"]["pel"], Levels=1, Chroma=0,
                        XRatioUV=1, YRatioUV=1, BlkSizeX=block, BlkSizeY=block,
                        OverlapX=0, OverlapY=0, NBlkX=width // block, NBlkY=height // block,
                        DeltaFrame=field["delta"] if field else -1, BitsPerSample=8)
        count = metadata["NBlkX"] * metadata["NBlkY"]
        dx, dy = field["seed"] if field else [0, 0]
        packed = (dx & 0xffffffff) | ((dy & 0xffffffff) << 32)
        if packed >= 1 << 63:
            packed -= 1 << 64

        def fill(n, f):
            result = f.copy()
            for key, value in metadata.items():
                result.props[prefix + "Analysis" + key] = value
            if field or n != 0:
                result.props[prefix + "AnalysisVectors"] = [packed] * count
                # Recalculate must measure new errors, not reuse these values.
                result.props[prefix + "AnalysisSAD"] = [0 if field else (999 if n % 2 else 0)] * count
            return result

        return core.std.ModifyFrame(src, src, fill)

    if op == "scene_public":
        out = plugin.SCDetection(src, public_field(), prefix=prefix, **params)
    else:
        sup = plugin.Super(src, prefix=prefix, **spec["super_params"])
        if op == "super":
            out = sup
        elif op == "analyse":
            out = plugin.Analyse(sup, prefix=prefix, **params)
        elif op == "many":
            out = plugin.AnalyseMany(sup, prefix=prefix, **params)
        elif op == "recalculate_chain":
            old = [plugin.Analyse(sup, delta=d, prefix=prefix) for d in [1, -1]]
            out = plugin.Recalculate(sup, old, prefix=prefix, **params)
        elif op == "recalculate_public":
            out = plugin.Recalculate(sup, public_field(), prefix=prefix, **params)
        elif op == "scene_chain":
            old = plugin.Analyse(sup, delta=-1, prefix=prefix)
            out = plugin.SCDetection(src, old, prefix=prefix, **params)
        else:
            raise ValueError(f"unknown operation: {op}")
    return src, list(out) if isinstance(out, (list, tuple)) else [out]
