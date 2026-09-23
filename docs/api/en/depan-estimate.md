# DepanEstimate

Estimate adjacent-frame translation and optional zoom from first-plane FFT correlation.

## Calling the function

VapourSynth: `core.neo_mv.DepanEstimate`. AviSynth: `neo_mv_DepanEstimate`. Parameter order:

```text
DepanEstimate(clip [, trust, winx, winy, wleft, wtop, dxmax, dymax, zoommax, stab, pixaspect, info, show, fields, tff])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `trust` | Float | `4.0` | Finite 0–100 confidence threshold for accepting motion. |
| `winx` | Integer | `0` | Window width; 0 chooses an automatic power of two up to 8192. Explicit width must be nonnegative and even, and fit the image. |
| `winy` | Integer | `0` | Window height; 0 chooses an automatic power of two up to 8192. Explicit height must be nonnegative and fit the image. |
| `wleft` | Integer | `-1` | Horizontal origin; a negative value selects automatic placement. |
| `wtop` | Integer | `-1` | Vertical origin; a negative value centers the window vertically. |
| `dxmax` | Integer | `-1` | Horizontal peak-search radius; negative uses one quarter of the effective window width. Must be less than half that width. |
| `dymax` | Integer | `-1` | Vertical peak-search radius; negative uses one quarter of the window height. Must be less than half that height. |
| `zoommax` | Float | `1.0` | Finite zoom acceptance control. Exactly 1 disables zoom estimation; other values use two horizontally separated half-width windows. |
| `stab` | Float | `1.0` | Finite displacement penalty in correlation-peak selection; negative values are accepted. |
| `pixaspect` | Float | `1.0` | Finite positive pixel aspect ratio; field mode adjusts the effective vertical aspect. |
| `info` | Boolean | `false` | Write diagnostic text and draw it using the host renderer. AviSynth uses propShow. |
| `show` | Boolean | `false` | Draw correlation surfaces over the visible image; motion properties are still produced. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |

### Automatic windows and display

Negative origins select automatic placement; zero window dimensions select the largest fitting power of two capped at 8192. With zoommax≠1, horizontal width is halved before placement. Automatic peak radii are one quarter of the effective window dimensions.

`show=true` draws correlation surfaces. `info=true` writes `DepanEstimate_info` and draws diagnostics. The function removes old `DepanEstimateFFT`, `DepanEstimateFFT2`, `DepanEstimateX`, `DepanEstimateY`, `DepanEstimateZoom`, `DepanEstimateGood`, and `DepanEstimateTrust` properties; these are not inputs.

## Input requirements

Fixed GRAY/YUV integer 8–16-bit or float32 clip. Only the first plane is analyzed. Windows must lie inside the image; required samples must be finite and integer samples must fit their declared depth.

## Return value and properties

Writes `Depan_dx`, `Depan_dy` (pixel displacement), `Depan_rot` (degrees), `Depan_zoom` (scale) and `Depan_goodmotion` (0/1). These names have no configurable prefix. Goodmotion=0 denotes an unusable estimate, not zero motion. The visible clip/timeline is retained except when display options draw on it.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
result = core.neo_mv.DepanEstimate(clip, winx=32, winy=32)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
result = neo_mv_DepanEstimate(clip, winx=32, winy=32)
return result
```

## Restrictions and common errors

Effective window width must be positive and even, and height positive. In zoom mode the width is halved and two separated windows must fit. Peak radii must be smaller than half their effective window dimensions. Required parity/FFT failures are errors, not invalid-motion fallback.

## Computation

See [DepanEstimate: computation](../../knowledge/en/depan-estimate.md) for formulas, rounding and numerical examples.

[API index](README.md)
