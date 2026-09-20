"""Backend-independent result validation and exact public-output comparison."""
import hashlib
import json
import math
import struct

from cases import ANALYSIS_KEYS, SUPER_KEYS

SCHEMA = 2
ORDINARY_KEYS = ("TestMarker", "TestData", "TestFloat", "_Field", "_SceneChangePrev", "_SceneChangeNext",
                 "_DurationNum", "_DurationDen")
DEPAN_KEYS = ("Depan_dx", "Depan_dy", "Depan_rot", "Depan_zoom", "Depan_goodmotion",
              "DepanAnalyse_info", "DepanCompensate_info")
DEPAN_FLOAT_KEYS = ("Depan_dx", "Depan_dy", "Depan_rot", "Depan_zoom")
DEPAN_FLOAT_ABSOLUTE_TOLERANCE = 1e-4
DEPAN_FLOAT_RELATIVE_TOLERANCE = 1e-5


def observation_keys(spec):
    keys = list(ORDINARY_KEYS) + [spec["prefix"] + suffix for suffix in SUPER_KEYS + ANALYSIS_KEYS]
    if spec.get("phase") in (3, 4, 5):
        keys += ["_Range", "_ColorRange", "_Matrix"]
    if spec.get("phase") == 5:
        keys += list(DEPAN_KEYS)
    return keys


def digest_file(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def validate_snapshot(record):
    if not record.get("planes") or not isinstance(record.get("properties"), dict):
        raise ValueError("incomplete frame observation")
    for plane in record["planes"]:
        if any(type(plane.get(key)) is not int or plane[key] <= 0 for key in ("width", "height", "sample_bytes")):
            raise ValueError("invalid plane geometry")
        data = bytes.fromhex(plane["data"])
        if len(data) != plane["width"] * plane["height"] * plane["sample_bytes"]:
            raise ValueError("truncated logical plane")
        if hashlib.sha256(data).hexdigest() != plane.get("sha256"):
            raise ValueError("plane digest mismatch")
    for value in record["properties"].values():
        kind = value.get("type")
        if kind not in ("int", "float64", "data") or not isinstance(value.get("values"), list):
            raise ValueError("invalid typed property")
        if type(value.get("count")) is not int or value["count"] <= 0 or value["count"] != len(value["values"]):
            raise ValueError("invalid property count")
        for item in value["values"]:
            if kind == "int":
                if type(item) is not int or not -(2**63) <= item < 2**63:
                    raise ValueError("invalid int64 property")
            else:
                if not isinstance(item, str):
                    raise ValueError("invalid encoded property")
                encoded = bytes.fromhex(item)
                if kind == "float64" and len(encoded) != 8:
                    raise ValueError("invalid encoded property")


def validate_result(result, spec, backend):
    """Fail closed: absent, duplicate or truncated observations are not equality."""
    expected_errors = spec.get("expected_output_errors", [])
    error_targets = set()
    for expected in expected_errors:
        if set(expected) != {"member", "frame", "sentinel"} or \
                spec.get("phase") != 5 or spec.get("operation") != "DepanAnalyse" or \
                type(expected["member"]) is not int or type(expected["frame"]) is not int or \
                not isinstance(expected["sentinel"], str) or not expected["sentinel"] or \
                [expected["member"], expected["frame"]] not in spec["requests"] or \
                expected["frame"] not in spec.get("input_error_frames", {}).get("mask", []):
            raise ValueError("invalid injected output error contract")
        target = (expected["member"], expected["frame"])
        if target in error_targets:
            raise ValueError("duplicate injected output error contract")
        error_targets.add(target)
    if result.get("schema") != SCHEMA or result.get("case_id") != spec["id"]:
        raise ValueError("wrong result schema or case ID")
    if result.get("backend") != backend or result.get("status") != "ok":
        raise ValueError("backend did not produce a successful result")
    if result.get("case_sha256") != digest_json(spec):
        raise ValueError("worker used a different case definition")
    creation_error = result.get("creation_error")
    if creation_error is not None:
        validate_error(creation_error)
        if spec.get("phase") not in (2, 3, 4, 5) or result.get("outputs") != [] or result.get("records") != []:
            raise ValueError("invalid creation failure observation")
    elif len(result.get("outputs", [])) != spec["members"]:
        raise ValueError("wrong output member count")
    records = result.get("records", [])
    if not creation_error and len(records) != len(spec["requests"]):
        raise ValueError("missing or extra frame observations")
    for ordinal, (record, request) in enumerate(zip(records, spec["requests"])):
        if [record.get("member"), record.get("frame")] != request or record.get("request") != ordinal:
            raise ValueError("wrong frame/member/request identity")
        if "error" in record:
            validate_error(record["error"])
            if "planes" in record or "properties" in record:
                raise ValueError("error record also contains successful pixels")
        else:
            validate_snapshot(record)
            if spec.get("phase") in (3, 4, 5):
                names = record.get("property_names")
                if not isinstance(names, list) or any(not isinstance(name, str) for name in names) or \
                        names != sorted(set(names)) or \
                        set(record["properties"]) != set(names).intersection(observation_keys(spec)):
                    raise ValueError("missing or invalid output property inventory")
    inputs = result.get("inputs", [])
    if [item.get("frame") for item in inputs] != list(range(spec["length"])):
        raise ValueError("missing or duplicated source frames")
    for item in inputs:
        validate_snapshot(item)
    if spec.get("phase") in (2, 3, 4, 5):
        if not result.get("input_video"):
            raise ValueError("missing source video metadata")
    if spec.get("phase") in (2, 4, 5) or (spec.get("phase") == 3 and spec.get("operation") == "Flow"):
        auxiliary = result.get("auxiliary_inputs", [])
        if spec.get("phase") == 5:
            expected = (["vectors"] + (["mask"] if spec["mask"] is not None else [])) \
                if spec["operation"] == "DepanAnalyse" else ["data"]
        else:
            expected = ["super_source"] + ["vectors" + str(i) for i in range(len(spec["deltas"]))]
        if [item.get("name") for item in auxiliary] != expected:
            raise ValueError("missing auxiliary input observations")
        for item in auxiliary:
            if not item.get("video") or [f.get("frame") for f in item.get("frames", [])] != list(range(spec["length"])):
                raise ValueError("incomplete auxiliary video")
            for frame in item["frames"]:
                declared_error = frame["frame"] in spec.get("input_error_frames", {}).get(item["name"], [])
                if declared_error:
                    validate_error(frame.get("error"))
                    if "planes" in frame or "properties" in frame:
                        raise ValueError("failed input also contains successful pixels")
                else:
                    if "error" in frame:
                        raise ValueError("undeclared auxiliary input failure")
                    validate_snapshot(frame)
    env = result.get("environment", {})
    if not env.get("plugin_sha256") or not env.get("core") or not env.get("kernel"):
        raise ValueError("missing runtime provenance")
    if "reference_request" in env:
        requested = env["reference_request"]
        if backend != "mvu" or requested.get("path") != env.get("plugin_path") or \
                requested.get("sha256") != env["plugin_sha256"] or \
                requested.get("selection") != "explicit; autoload disabled":
            raise ValueError("explicit reference identity differs from loaded binary")
    if spec.get("phase") == 5 and spec["params"].get("info"):
        renderer = env.get("text_renderer", {})
        if renderer.get("entry") != "text.FrameProps" or \
                renderer.get("arguments") != dict(props=[spec["operation"] + "_info"]) or \
                not renderer.get("plugin_version"):
            raise ValueError("missing or incorrect text renderer provenance")
        if renderer.get("plugin_path"):
            if not renderer.get("plugin_sha256") or renderer.get("builtin_core") is not None:
                raise ValueError("incomplete external text renderer identity")
        elif not renderer.get("builtin_core") or renderer.get("plugin_sha256") is not None:
            raise ValueError("incomplete built-in text renderer identity")


def validate_error(error):
    if not isinstance(error, dict) or not isinstance(error.get("type"), str) or not error["type"] or \
            not isinstance(error.get("message"), str) or not error["message"]:
        raise ValueError("incomplete filter error observation")


def first_difference(left, right, path=""):
    """Keep the full observations on disk; return one small, actionable diff."""
    def short(value):
        return value if not isinstance(value, str) or len(value) <= 120 else value[:120] + "..."

    if type(left) is not type(right):
        return dict(path=path, reference=short(left), candidate=short(right))
    if isinstance(left, dict):
        for key in sorted(left.keys() | right.keys()):
            child = path + "/" + key
            if key not in left or key not in right:
                return dict(path=child, reference="present" if key in left else "missing",
                            candidate="present" if key in right else "missing")
            difference = first_difference(left[key], right[key], child)
            if difference:
                return difference
    elif isinstance(left, list):
        if len(left) != len(right):
            return dict(path=path + "/length", reference=len(left), candidate=len(right))
        for index, (a, b) in enumerate(zip(left, right)):
            difference = first_difference(a, b, path + "/" + str(index))
            if difference:
                return difference
    elif left != right:
        return dict(path=path, reference=short(left), candidate=short(right))
    return None


def mask_range_observations(reference, candidate, spec):
    """Project only the approved MVU 8 / VS R79 mask discrepancy for comparison.

    Returned records are copies; worker observations are never rewritten.
    Omitted spec keeps the comparator completely strict.
    """
    left, right = reference["records"], candidate["records"]
    if not spec or spec.get("phase") != 3 or spec.get("operation") not in (
            "VectorLengthMask", "SADMask", "OcclusionMask") or \
            reference.get("backend") != "mvu" or candidate.get("backend") != "neo" or \
            reference.get("environment", {}).get("mvu_package") != "8" or \
            any(r.get("environment", {}).get("vs_package") != "79" for r in (reference, candidate)):
        return left, right, []
    expected_left = dict(type="int", count=1, values=[0])
    expected_right = dict(type="int", count=1, values=[1])
    left, right, known = list(left), list(right), []
    for i, (a, b) in enumerate(zip(left, right)):
        av, bv = a.get("properties", {}).get("_Range"), b.get("properties", {}).get("_Range")
        if "_Range" not in a.get("property_names", []) or "_Range" not in b.get("property_names", []) or \
                first_difference(av, expected_left) or first_difference(bv, expected_right):
            continue
        known.append(dict(rule="mvu8-vs79-mask-range", path=f"/records/{i}/properties/_Range",
                          reference=av, candidate=bv,
                          request={key: a[key] for key in ("request", "member", "frame")}))
        left[i] = dict(a, properties={k: v for k, v in a["properties"].items() if k != "_Range"})
        right[i] = dict(b, properties={k: v for k, v in b["properties"].items() if k != "_Range"})
    return left, right, known


def depan_float_observations(reference, candidate, spec, left, right):
    """Project only finite motion values within the fixed, explicitly enabled bound.

    Original records, native types, counts and bit strings remain in reports.
    Other fields (including diagnostic text) are still compared exactly.
    """
    kernel = candidate.get("environment", {}).get("kernel", {})
    if not spec or spec.get("phase") != 5 or spec.get("operation") != "DepanAnalyse" or \
            reference.get("backend") != "mvu" or candidate.get("backend") != "neo" or \
            kernel.get("effective") != "highway" or not kernel.get("target") or \
            kernel["target"] in ("scalar", "SCALAR", "EMU128"):
        return left, right, [], []
    right, tolerated, rejected = list(right), [], []
    for ordinal, (a, b) in enumerate(zip(left, right)):
        if "error" in a or "error" in b:
            continue
        properties = dict(b.get("properties", {}))
        for key in DEPAN_FLOAT_KEYS:
            av, bv = a.get("properties", {}).get(key), properties.get(key)
            if not isinstance(av, dict) or not isinstance(bv, dict) or \
                    av.get("type") != "float64" or bv.get("type") != "float64" or \
                    type(av.get("count")) is not int or av["count"] <= 0 or av["count"] != bv.get("count") or \
                    not isinstance(av.get("values"), list) or not isinstance(bv.get("values"), list) or \
                    len(av["values"]) != av["count"] or len(bv["values"]) != bv["count"]:
                continue
            values = list(bv["values"])
            for index, (abits, bbits) in enumerate(zip(av["values"], bv["values"])):
                if abits == bbits:
                    continue
                try:
                    af = struct.unpack(">d", bytes.fromhex(abits))[0]
                    bf = struct.unpack(">d", bytes.fromhex(bbits))[0]
                except (TypeError, ValueError, struct.error):
                    continue
                if not math.isfinite(af) or not math.isfinite(bf):
                    continue
                absolute_difference = abs(af - bf)
                limit = DEPAN_FLOAT_ABSOLUTE_TOLERANCE + DEPAN_FLOAT_RELATIVE_TOLERANCE * max(abs(af), abs(bf))
                observation = dict(path=f"/records/{ordinal}/properties/{key}/values/{index}",
                    request={name: a[name] for name in ("request", "member", "frame")},
                    reference_bits=abits, candidate_bits=bbits, reference=af, candidate=bf,
                    absolute_difference=absolute_difference if math.isfinite(absolute_difference) else "overflow",
                    limit=limit)
                if absolute_difference <= limit:
                    tolerated.append(observation)
                    values[index] = abits
                else:
                    rejected.append(observation)
            properties[key] = dict(bv, values=values)
        right[ordinal] = dict(b, properties=properties)
    return left, right, tolerated, rejected


def compare(reference, candidate, spec=None, *, depan_float_tolerance=False):
    # Do not compare DLL identity/host addresses/private payloads. Do compare the
    # actual generated source before assigning any difference to an algorithm.
    difference = first_difference(reference.get("environment", {}).get("text_renderer"),
                                  candidate.get("environment", {}).get("text_renderer"), "/environment/text_renderer")
    if difference:
        return "input_mismatch", difference
    difference = first_difference(reference.get("input_video"), candidate.get("input_video"), "/input_video")
    if difference:
        return "input_mismatch", difference
    difference = first_difference(reference["inputs"], candidate["inputs"], "/inputs")
    if difference:
        return "input_mismatch", difference
    difference = first_difference(reference.get("auxiliary_inputs", []), candidate.get("auxiliary_inputs", []),
                                  "/auxiliary_inputs")
    if difference:
        return "input_mismatch", difference
    if "creation_error" in reference or "creation_error" in candidate:
        # These are positive compatibility fixtures. Even matching rejections
        # are red; preserve both messages without claiming their causes match.
        return "difference", dict(path="/creation", reference=reference.get("creation_error", "success"),
                                  candidate=candidate.get("creation_error", "success"))
    left_records, right_records, known = mask_range_observations(reference, candidate, spec)
    left_records, right_records = list(left_records), list(right_records)
    for index, (left, right) in enumerate(zip(reference["records"], candidate["records"])):
        expected = next((item for item in (spec or {}).get("expected_output_errors", [])
                         if (item["member"], item["frame"]) == (left["member"], left["frame"])), None)
        if expected is not None:
            sentinel = expected["sentinel"]
            matches = all((record["member"], record["frame"]) == (expected["member"], expected["frame"])
                          and isinstance(record.get("error", {}).get("message"), str)
                          and sentinel in record["error"]["message"] for record in (left, right))
            if matches:
                # Only declared negative fixtures may accept their injected
                # failure. Keep original errors intact in the worker reports;
                # differing host prefixes are not part of this contract.
                left_records[index] = dict(left_records[index], error=dict(expected_sentinel=sentinel))
                right_records[index] = dict(right_records[index], error=dict(expected_sentinel=sentinel))
                continue
        if expected is not None or "error" in left or "error" in right:
            difference = dict(path=f"/records/{index}/error", reference=left.get("error", "success"),
                              candidate=right.get("error", "success"),
                              request={key: left[key] for key in ("request", "member", "frame")})
            if known:
                difference["known_differences"] = known
            if expected is not None:
                difference["expected_error"] = expected
            return "difference", difference
    tolerated, rejected = [], []
    if depan_float_tolerance:
        left_records, right_records, tolerated, rejected = depan_float_observations(
            reference, candidate, spec, left_records, right_records)
    for field, left, right in (("outputs", reference["outputs"], candidate["outputs"]),
                               ("records", left_records, right_records)):
        difference = first_difference(left, right, "/" + field)
        if difference:
            if field == "records":
                index = int(difference["path"].split("/")[2]) if difference["path"].split("/")[2].isdigit() else None
                if index is not None:
                    difference["request"] = {key: reference[field][index][key]
                                             for key in ("request", "member", "frame")}
            if known:
                difference["known_differences"] = known
            if tolerated:
                difference["tolerated_differences"] = tolerated
            if rejected:
                difference["rejected_float_differences"] = rejected
            return "difference", difference
    if known:
        return "known_difference", dict(known_differences=known)
    if tolerated:
        return "within_tolerance", dict(tolerated_differences=tolerated,
            absolute_tolerance=DEPAN_FLOAT_ABSOLUTE_TOLERANCE,
            relative_tolerance=DEPAN_FLOAT_RELATIVE_TOLERANCE)
    return "pass", None
