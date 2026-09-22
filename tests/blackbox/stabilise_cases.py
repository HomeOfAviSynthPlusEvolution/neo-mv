"""Phase-7 public motion fixtures, compared without numeric exemptions."""
import depan_cases


def case(name, *, params=None, **extra):
    result = dict(id='stabilise.' + name, phase=7, operation='DepanStabilise',
                  format='GRAY8', width=48, height=48, length=8, prefix='MVUtensils',
                  members=1, params=params or {}, requests=[[0, n] for n in [7, 0, 3, 1, 6, 2, 4, 5, 3]],
                  missing=False, motion=[0.25, -0.125, 0.0, 1.0, 1])
    result.update(extra)
    return result


CASES = [
    *[case(f'method{method}.{fmt}', format=fmt, params=dict(method=method))
      for method in [0, 1] for fmt in ['GRAY8', 'YUV420P10', 'YUV422P16', 'YUV444P8']],
    *[case(f'layers.mode{mode}', params=dict(prev=2, next=2, subpixel=mode, mirror=15, blur=3))
      for mode in [0, 1, 2]],
    case('inertial_zoom', params=dict(addzoom=True, initzoom=1.1)),
    case('window_zoom', params=dict(method=1, addzoom=True, initzoom=1.1)),
    case('window_radius1', params=dict(method=1, cutoff=6)),
    case('window_radius2', params=dict(method=1, cutoff=2.9)),
    case('scene_inertial', invalid_frames=[3]),
    case('scene_window', invalid_frames=[3, 6], params=dict(method=1)),
    case('scene_layers', invalid_frames=[3, 6], params=dict(prev=3, next=3)),
    case('fields', params=dict(fields=True, pixaspect=1.5)),
    case('fitlast', params=dict(fitlast=4)),
    case('hard_limit', motion=[20.0, 0.0, 0.0, 1.0, 1], params=dict(dxmax=-10.0)),
    case('soft_limit', motion=[20.0, 0.0, 0.0, 1.0, 1], params=dict(dxmax=10.0)),
    case('rotation_zoom', motion=[0.25, -0.125, 0.1, 1.002, 1], params=dict(addzoom=True)),
    case('info', width=320, params=dict(info=True)),
    case('identity_layers', motion=[0.0, 0.0, 0.0, 1.0, 1], params=dict(prev=1, subpixel=1)),
]


def prepare(vs, core, spec):
    inputs = depan_cases.prepare(vs, core, spec)
    inputs['clip'] = core.std.SetFrameProps(inputs['clip'], DepanStabilise_info=b'inherited stabilization')
    return inputs


def build(plugin, spec, inputs):
    return [plugin.DepanStabilise(inputs['clip'], inputs['data'], **spec['params'])]
