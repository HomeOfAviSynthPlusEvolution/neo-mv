"""Exercise the public AviSynth interface using DS2's pinned C API runner."""
import argparse
import pathlib
import subprocess

parser = argparse.ArgumentParser()
for option in ("runner", "plugin", "runtime", "backend", "work"):
    parser.add_argument("--" + option, required=True)
a = parser.parse_args()
work = pathlib.Path(a.work).resolve()
work.mkdir(parents=True, exist_ok=True)
plugin = pathlib.Path(a.plugin).resolve().as_posix()
header = f'LoadPlugin("{plugin}")\n'
fixture = """
c = BlankClip(width=64,height=48,length=12,fps=24,pixel_type="YV12",color_yuv=$204080)
s = neo_mv_Super(c,blksize=8,overlap=4,pad=32,pel=2)
v = neo_mv_AnalyseMany(s,radius=2,badrange=0)
b = v[0]
f = v[1]
d = neo_mv_DepanEstimate(c,winx=32,winy=32)
"""
count = 0

def run(name, body, frame=4, error=None, prefix=fixture, mode="--video", extra=()):
    global count
    path = work / (name + ".avs")
    path.write_text(header + prefix + body, encoding="utf-8")
    result = subprocess.run([a.runner, mode, str(path), "--backend", "c",
                             "--runtime", a.runtime, "--frame", str(frame), *extra],
                            capture_output=True, text=True, timeout=40)
    text = result.stdout + result.stderr
    if error:
        assert result.returncode != 0 and error in text, (name, text)
    else:
        assert result.returncode == 0, (name, text)
    count += 1
    return result.stdout

names = ["Super", "Analyse", "AnalyseMany", "Recalculate", "SCDetection", "Compensate",
         "Degrain", "VectorLengthMask", "SADMask", "OcclusionMask", "Flow", "FlowInter",
         "FlowFPS", "FlowBlur", "DepanAnalyse", "DepanCompensate", "DepanEstimate",
         "DepanStabilise", "KernelInfo"] + [f"Degrain{i}" for i in range(1, 26)]
run("registration", "\n".join(f'Assert(FunctionExists("neo_mv_{n}"),"missing {n}")' for n in names)
    + f'\nk=neo_mv_KernelInfo()\nAssert(k[0]=="{a.backend}")\nAssert(k[3]>=1)\nreturn c')

paths = {
    "super": "s", "analyse": "neo_mv_Analyse(s,delta=-1,badrange=0)",
    "many": "v[3]", "recalculate": "neo_mv_Recalculate(s,v)[2]",
    "recalculate-single": "neo_mv_Recalculate(s,b)[0]",
    "scene": "neo_mv_SCDetection(c,b)", "compensate": "neo_mv_Compensate(c,s,b)",
    "degrain": "neo_mv_Degrain(c,s,v,thsad=[400,400],limit=255.0,planes=[0,1,2])",
    "vector-mask": "neo_mv_VectorLengthMask(b)", "sad-mask": "neo_mv_SADMask(b)",
    "occlusion-mask": "neo_mv_OcclusionMask(b)", "flow": "neo_mv_Flow(c,s,b)",
    "inter": "neo_mv_FlowInter(c,s,[b,f])", "fps": "neo_mv_FlowFPS(c,s,[b,f],num=48,den=1)",
    "blur": "neo_mv_FlowBlur(c,s,[b,f])", "depan-analyse": "neo_mv_DepanAnalyse(c,b)",
    "depan-estimate": "d", "depan-compensate": "neo_mv_DepanCompensate(c,d,offset=1.5)",
    "depan-stabilise": "neo_mv_DepanStabilise(c,d)",
    "depan-window": "neo_mv_DepanStabilise(c,d,method=1)",
}
for name, expression in paths.items():
    run(name, "return " + expression)
    run(name + "-prefetch", "return (" + expression + ").Prefetch(4)")
for radius in range(1, 26):
    run(f"degrain{radius}", f"vv=neo_mv_AnalyseMany(s,radius={radius},badrange=0)\n"
        f"return neo_mv_Degrain{radius}(c,s,vv)")
for name in ("compensate", "degrain", "inter", "blur", "depan-stabilise"):
    for frame in (0, 11):
        run(f"{name}-edge{frame}", "return " + paths[name], frame=frame)

# Runtime property assertions check bundle order and custom-prefix propagation.
run("properties", '''
s2=neo_mv_Super(c,blksize=[8,8],overlap=[],pad=[32,32],prefix="Test")
v2=neo_mv_AnalyseMany(s2,radius=2,delta=2,prefix="Test",badrange=0)
return v2[2].ScriptClip("""Assert(propGetInt(last, "TestAnalysisDelta") == 4)
last""")
''')
run("recalculate-order", '''
r=neo_mv_Recalculate(s,v)
return r[3].ScriptClip("""Assert(propGetInt(last, "MVUtensilsAnalysisDelta") == -2)
last""")
''')
for name, expression in {
    "analyse": "neo_mv_DepanAnalyse(c,b,info=true)",
    "compensate": "neo_mv_DepanCompensate(c,d,info=true)",
    "estimate": "neo_mv_DepanEstimate(c,winx=32,winy=32,info=true)",
    "stabilise": "neo_mv_DepanStabilise(c,d,info=true)",
}.items():
    run("info-" + name, "return " + expression)

# Audio and parity come from the first clip, independent of vector inputs.
audio = """
c=AudioDub(c,Tone(length=0.5,samplerate=48000,channels=2)).AssumeTFF()
o=neo_mv_Compensate(c,s,b)
Assert(AudioRate(o)==AudioRate(c))
Assert(AudioChannels(o)==AudioChannels(c))
Assert(GetParity(o,4)==GetParity(c,4))
"""
original = run("audio-source", audio + "return c", mode="--audio")
forwarded = run("audio-output", audio + "return o.Prefetch(4)", mode="--audio")
assert original == forwarded

# Distinct temporal samples catch an adapter that returns the current frame or
# requests the wrong reference. Check pixels, including end-of-clip fallback.
temporal = """
c=BlankClip(width=32,height=32,length=1,pixel_type="Y8",color_yuv=$0A8080) \\
 + BlankClip(width=32,height=32,length=1,pixel_type="Y8",color_yuv=$1E8080) \\
 + BlankClip(width=32,height=32,length=1,pixel_type="Y8",color_yuv=$5A8080)
s=neo_mv_Super(c,blksize=8,overlap=4,pad=32,pel=1)
b=neo_mv_Analyse(s,delta=1,badrange=0)
"""
for frame, value in enumerate((30, 90, 90)):
    run(f"reference-pixels-{frame}",
        "return neo_mv_Compensate(c,s,b,thsad=16320,thscd1=16320).Prefetch(4)",
        frame=frame, prefix=temporal, extra=("--expect-y8-sum", str(32 * 32 * value)))

for name, body, error in [
    ("missing-block", "return neo_mv_Super(c,overlap=4)", "blksize"),
    ("bad-array", 'return neo_mv_Super(c,blksize=[8,"bad"],overlap=4)', "expected integer"),
    ("bad-radius", "return neo_mv_AnalyseMany(s,radius=0)", "radius"),
    ("large-bundle", "return neo_mv_AnalyseMany(s,radius=16384)", "radius"),
    ("empty-recalculate", "return neo_mv_Recalculate(s,[])", "nonempty"),
    ("bundle-rollback", "return neo_mv_Recalculate(s,[b,c])", "member[1]"),
    ("bad-degrain", "return neo_mv_Degrain1(c,s,v)", "exactly 2R"),
    ("bad-format", 'return neo_mv_Super(BlankClip(c,pixel_type="RGB32"),8,4)', "format"),
]:
    run(name, body, error=error)

# Exercise integer high depth and floating-point planes through the same bridge.
for pixel in ("Y8", "YUV420P10", "YUV420P16", "Y32", "YUV444PS"):
    setup = fixture.replace('pixel_type="YV12"', f'pixel_type="{pixel}"')
    # Global compensation is integer-only; it is not used in these graphs.
    setup = setup[:setup.index("d = ")]
    run("format-" + pixel, "return neo_mv_Compensate(c,s,b)", prefix=setup)
print(f"AviSynth {a.backend}: {count} cases passed; all 44 functions registered.")
