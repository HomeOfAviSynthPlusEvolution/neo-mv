"""One case and one implementation per process; only public API observations.

Each fresh process selects its backend before loading neo-mv, then queries the
loaded plugin's actual dispatch target. SIMD requests must not pass as scalar.
"""
import argparse
from enum import IntEnum
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import struct
import sys
import traceback

from cases import build
from catalog import BY_ID
import render_cases
import mask_cases
from protocol import ORDINARY_KEYS, SCHEMA, digest_file, digest_json, observation_keys


def configure_kernel(backend, requested, plugin=None):
    if backend == "mvu":
        if requested != "auto":
            raise ValueError("reference supports only its native automatic selection")
        return {"requested": "auto", "effective": "auto", "target": None}
    if requested not in ("scalar", "highway") or plugin is None:
        raise ValueError("candidate requires a loaded plugin and scalar or highway selection")
    info = plugin.KernelInfo()
    effective, target = info["backend"], info["target"]  # KernelInfo returns UTF-8 data as str.
    if not isinstance(effective, str) or not isinstance(target, str):
        raise ValueError("kernel identity must contain UTF-8 strings")
    if effective != requested or not target:
        raise ValueError(f"kernel mismatch: requested {requested}, got {info}")
    if requested == "highway" and target in ("scalar", "SCALAR", "EMU128"):
        raise ValueError(f"SIMD requested but got non-SIMD target {target}")
    if requested == "scalar" and target != "scalar":
        raise ValueError(f"scalar requested but got target {target}")
    return dict(requested=requested, effective=effective, target=target,
                selection="NEO_MV_KERNEL + loaded plugin KernelInfo")


def video_info(node):
    fmt = node.format
    return dict(width=node.width, height=node.height, length=node.num_frames,
                fps=[node.fps_num, node.fps_den],
                format=dict(family=int(fmt.color_family), sample_type=int(fmt.sample_type),
                            bits=fmt.bits_per_sample, bytes=fmt.bytes_per_sample,
                            subsampling=[fmt.subsampling_w, fmt.subsampling_h], planes=fmt.num_planes))


def property_value(value):
    items = list(value) if isinstance(value, (list, tuple)) else [value]
    if not items:
        raise ValueError("empty observed property requires a native type-aware reader")
    # VS exposes some native integer properties as IntEnum members. Retain the
    # exact integer payload and native property type, without coercing floats,
    # booleans, data strings or arbitrary int-convertible objects.
    if all(type(item) is int or isinstance(item, IntEnum) for item in items):
        return dict(type="int", count=len(items), values=[int(item) for item in items])
    kind = type(items[0])
    if any(type(item) is not kind for item in items):
        raise ValueError("mixed property types")
    if kind is float:
        return dict(type="float64", count=len(items), values=[struct.pack(">d", item).hex() for item in items])
    if kind is bytes:
        return dict(type="data", count=len(items), values=[item.hex() for item in items])
    raise ValueError(f"unsupported public property type: {kind.__name__}")


def snapshot(frame, keys):
    # memoryview.tobytes serializes logical samples, excluding host row padding.
    planes = []
    for index in range(frame.format.num_planes):
        view = frame[index]
        data = view.tobytes()
        planes.append(dict(width=view.shape[1], height=view.shape[0],
                           sample_bytes=frame.format.bytes_per_sample,
                           sha256=hashlib.sha256(data).hexdigest(), data=data.hex()))
    # Absence is a missing key, never normalized to zero or an empty array.
    # Test actual names, not mapping membership: newer VS Python bindings expose
    # deprecated aliases (e.g. _ColorRange) for keys absent from the native map.
    names = set(frame.props)
    props = {key: property_value(frame.props[key]) for key in keys if key in names}
    return dict(planes=planes, properties=props)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["neo", "mvu"], required=True)
    parser.add_argument("--kernel", required=True)
    parser.add_argument("--plugin", type=Path)
    parser.add_argument("--case", choices=BY_ID, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threads", type=int, choices=[1, 4], required=True)
    parser.add_argument("--vs-version", default="79")
    parser.add_argument("--mvu-version", default="8")
    args = parser.parse_args()
    spec = BY_ID[args.case]
    result = dict(schema=SCHEMA, case_id=args.case, case_sha256=digest_json(spec),
                  backend=args.backend, status="error", stage="environment")
    try:
        import vapoursynth as vs
        core = vs.core
        core.num_threads = args.threads
        package_vs = importlib.metadata.version("VapourSynth")
        package_mvu = importlib.metadata.version("vapoursynth-mvutensils")
        if package_vs != args.vs_version or package_mvu != args.mvu_version:
            raise ValueError(f"package mismatch: VS={package_vs}, MVU={package_mvu}")
        if core.core_version.release_major != int(args.vs_version) or core.core_version.release_minor != 0:
            raise ValueError(f"loaded core mismatch: {core.core_version}")
        if args.backend == "neo":
            if args.plugin is None:
                raise ValueError("candidate DLL path required")
            if os.environ.get("NEO_MV_KERNEL") != args.kernel:
                raise ValueError("worker must start with NEO_MV_KERNEL matching --kernel")
            core.std.LoadPlugin(path=str(args.plugin.resolve()))
            plugin = core.neomv
        else:
            plugin = core.mvu
            if plugin.version.major != int(args.mvu_version):
                raise ValueError(f"loaded MVU version mismatch: {plugin.version}")
        kernel = configure_kernel(args.backend, args.kernel, plugin)
        loaded = Path(plugin.plugin_path).resolve()
        loaded_hash = digest_file(loaded)
        if args.backend == "neo" and loaded_hash != digest_file(args.plugin):
            raise ValueError("loaded candidate binary differs from requested file")
        result["environment"] = dict(python=sys.executable, python_version=platform.python_version(),
            system=platform.platform(), machine=platform.machine(), processor=platform.processor(),
            vs_package=package_vs, mvu_package=package_mvu, core=str(core),
            plugin_path=str(loaded), plugin_sha256=loaded_hash,
            plugin_version=str(plugin.version), threads=core.num_threads, kernel=kernel)
        keys = observation_keys(spec)
        prepared = None
        fixture = {2: render_cases, 3: mask_cases}.get(spec.get("phase"))
        if fixture is not None:
            result["stage"] = "input"
            prepared = fixture.prepare(vs, core, spec)
            result["inputs"] = []
            result["auxiliary_inputs"] = []
            for name, node in prepared.items():
                frames = []
                for n in range(spec["length"]):
                    with node.get_frame(n) as frame:
                        frames.append(dict(frame=n, **snapshot(frame, keys)))
                if name == "clip":
                    result["input_video"] = video_info(node)
                    result["inputs"] = frames
                else:
                    result["auxiliary_inputs"].append(dict(name=name, video=video_info(node), frames=frames))
        result["stage"] = "creation"
        try:
            if prepared is None:
                source, outputs = build(vs, core, plugin, spec)
            else:
                source = prepared["clip"]
                outputs = fixture.build(plugin, spec, prepared)
        except vs.Error as error:
            if prepared is None:
                raise
            result.update(status="ok", stage="complete", outputs=[], records=[],
                          creation_error=dict(type=type(error).__name__, message=str(error)))
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
            return 0
        if len(outputs) != spec["members"]:
            raise ValueError(f"expected {spec['members']} outputs, got {len(outputs)}")
        result["outputs"] = [video_info(node) for node in outputs]
        result["stage"] = "input"
        if prepared is None:
            result["inputs"] = []
            for n in range(spec["length"]):
                with source.get_frame(n) as frame:
                    result["inputs"].append(dict(frame=n, **snapshot(frame, ORDINARY_KEYS)))
        result["stage"] = "frame"
        result["records"] = []
        for ordinal, (member, n) in enumerate(spec["requests"]):
            result["active_request"] = dict(request=ordinal, member=member, frame=n)
            try:
                acquired = outputs[member].get_frame(n)
            except vs.Error as error:
                result["records"].append(dict(**result["active_request"],
                    error=dict(type=type(error).__name__, message=str(error))))
            else:
                with acquired as frame:
                    observation = dict(**result["active_request"], **snapshot(frame, keys))
                    if spec.get("phase") == 3:
                        # Observe unexpected property presence without decoding
                        # or exporting unknown/private property payloads.
                        observation["property_names"] = sorted(frame.props)
                    result["records"].append(observation)
        result.pop("active_request", None)
        result["status"] = "ok"
        result["stage"] = "complete"
    except Exception as error:
        result["error"] = dict(type=type(error).__name__, message=str(error))
        traceback.print_exc()
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    return 0 if result["status"] == "ok" else 2


if __name__ == "__main__":
    sys.exit(main())
