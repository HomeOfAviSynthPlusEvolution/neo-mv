"""Backend-independent result validation and exact public-output comparison."""
import hashlib
import json

SCHEMA = 2


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
    if result.get("schema") != SCHEMA or result.get("case_id") != spec["id"]:
        raise ValueError("wrong result schema or case ID")
    if result.get("backend") != backend or result.get("status") != "ok":
        raise ValueError("backend did not produce a successful result")
    if result.get("case_sha256") != digest_json(spec):
        raise ValueError("worker used a different case definition")
    creation_error = result.get("creation_error")
    if creation_error is not None:
        validate_error(creation_error)
        if spec.get("phase") != 2 or result.get("outputs") != [] or result.get("records") != []:
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
    inputs = result.get("inputs", [])
    if [item.get("frame") for item in inputs] != list(range(spec["length"])):
        raise ValueError("missing or duplicated source frames")
    for item in inputs:
        validate_snapshot(item)
    if spec.get("phase") == 2:
        if not result.get("input_video"):
            raise ValueError("missing source video metadata")
        auxiliary = result.get("auxiliary_inputs", [])
        expected = ["super_source"] + ["vectors" + str(i) for i in range(len(spec["deltas"]))]
        if [item.get("name") for item in auxiliary] != expected:
            raise ValueError("missing auxiliary input observations")
        for item in auxiliary:
            if not item.get("video") or [f.get("frame") for f in item.get("frames", [])] != list(range(spec["length"])):
                raise ValueError("incomplete auxiliary video")
            for frame in item["frames"]:
                validate_snapshot(frame)
    env = result.get("environment", {})
    if not env.get("plugin_sha256") or not env.get("core") or not env.get("kernel"):
        raise ValueError("missing runtime provenance")


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


def compare(reference, candidate):
    # Do not compare DLL identity/host addresses/private payloads. Do compare the
    # actual generated source before assigning any difference to an algorithm.
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
    for index, (left, right) in enumerate(zip(reference["records"], candidate["records"])):
        if "error" in left or "error" in right:
            return "difference", dict(path=f"/records/{index}/error", reference=left.get("error", "success"),
                                      candidate=right.get("error", "success"),
                                      request={key: left[key] for key in ("request", "member", "frame")})
    for field in ("outputs", "records"):
        difference = first_difference(reference[field], candidate[field], "/" + field)
        if difference:
            if field == "records":
                index = int(difference["path"].split("/")[2]) if difference["path"].split("/")[2].isdigit() else None
                if index is not None:
                    difference["request"] = {key: reference[field][index][key]
                                             for key in ("request", "member", "frame")}
            return "difference", difference
    return "pass", None
