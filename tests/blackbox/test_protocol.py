"""Test the oracle transport without loading any video plugin."""
import copy
import hashlib
import unittest

from cases import CASES
from protocol import SCHEMA, compare, digest_json, validate_result
from worker import configure_kernel
from render_cases import CASES as RENDER_CASES


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.spec = CASES[0]
        frame = dict(planes=[dict(width=1, height=1, sample_bytes=1, data="00",
                                 sha256=hashlib.sha256(b"\0").hexdigest())],
                     properties={"MVUtensilsAnalysisVectors": dict(type="int", count=1, values=[0])})
        self.result = dict(schema=SCHEMA, case_id=self.spec["id"], backend="mvu", status="ok",
                           case_sha256=digest_json(self.spec), outputs=[{}],
                           environment=dict(core="R79", plugin_sha256="hash", kernel={"effective": "auto"}),
                           inputs=[dict(frame=n, **copy.deepcopy(frame)) for n in range(self.spec["length"])],
                           records=[dict(request=i, member=m, frame=n, **copy.deepcopy(frame))
                                    for i, (m, n) in enumerate(self.spec["requests"])])

    def test_complete_identity_and_exact_comparison(self):
        validate_result(self.result, self.spec, "mvu")
        candidate = copy.deepcopy(self.result)
        candidate["environment"]["plugin_sha256"] = "different implementation"
        self.assertEqual(compare(self.result, candidate), ("pass", None))

    def test_empty_truncated_and_duplicate_observations_fail(self):
        for mutate in (
            lambda r: r.update(records=[]),
            lambda r: r["records"].pop(),
            lambda r: r["records"].__setitem__(1, r["records"][0]),
            lambda r: r.update(outputs=[]),
            lambda r: r.update(inputs=[]),
            lambda r: r.update(case_sha256="other case"),
            lambda r: r.update(status="error"),
            lambda r: r["records"][0]["planes"][0].update(data=""),
            lambda r: r["records"][0]["planes"][0].update(sha256="wrong"),
            lambda r: r["records"][0]["properties"]["MVUtensilsAnalysisVectors"].update(count=2),
            lambda r: r["records"][0]["properties"]["MVUtensilsAnalysisVectors"].update(values=[2**63]),
        ):
            with self.subTest(mutation=mutate):
                result = copy.deepcopy(self.result)
                mutate(result)
                with self.assertRaises(ValueError):
                    validate_result(result, self.spec, "mvu")

    def test_missing_arrays_are_not_zero_vectors(self):
        candidate = copy.deepcopy(self.result)
        candidate["records"][0]["properties"].clear()
        status, diff = compare(self.result, candidate)
        self.assertEqual(status, "difference")
        self.assertEqual(diff["request"], dict(request=0, member=0, frame=2))
        self.assertEqual(diff["candidate"], "missing")

    def test_input_difference_is_not_an_algorithm_difference(self):
        candidate = copy.deepcopy(self.result)
        candidate["inputs"][0]["planes"][0]["data"] = "01"
        self.assertEqual(compare(self.result, candidate)[0], "input_mismatch")

    def test_signed_64bit_values_are_not_rounded(self):
        candidate = copy.deepcopy(self.result)
        candidate["records"][0]["properties"]["MVUtensilsAnalysisVectors"]["values"] = [-8589934589]
        self.assertEqual(compare(self.result, candidate)[1]["candidate"], -8589934589)

    def test_simd_cannot_pass_as_scalar(self):
        class Plugin:
            def __init__(self, backend, target):
                self.info = dict(backend=backend, target=target)
            def KernelInfo(self):
                return self.info
        for backend, target in [("scalar", "scalar"), ("highway", "SCALAR"),
                                ("highway", "EMU128"), ("highway", "")]:
            with self.subTest(backend=backend, target=target), self.assertRaises(ValueError):
                configure_kernel("neo", "highway", Plugin(backend, target))
        self.assertEqual(configure_kernel("neo", "highway", Plugin("highway", "AVX2"))["target"], "AVX2")
        self.assertEqual(configure_kernel("neo", "scalar", Plugin("scalar", "scalar"))["effective"], "scalar")
        with self.assertRaises(ValueError):
            configure_kernel("neo", "scalar", Plugin("highway", "AVX2"))

    def test_corrupt_encoded_properties_fail(self):
        for kind, value in [("data", "not hex"), ("float64", "00")]:
            with self.subTest(kind=kind):
                result = copy.deepcopy(self.result)
                result["records"][0]["properties"]["Marker"] = dict(type=kind, count=1, values=[value])
                with self.assertRaises(ValueError):
                    validate_result(result, self.spec, "mvu")

    def render_result(self):
        spec = RENDER_CASES[0]
        result = copy.deepcopy(self.result)
        result.update(case_id=spec["id"], case_sha256=digest_json(spec))
        result["input_video"] = {"width": 1}
        sample = {k: copy.deepcopy(v) for k, v in result["records"][0].items()
                  if k not in ("member", "frame", "request")}
        result["records"] = [dict(request=i, member=m, frame=n, **copy.deepcopy(sample))
                              for i, (m, n) in enumerate(spec["requests"])]
        result["auxiliary_inputs"] = [dict(name=name, video={"width": 1}, frames=copy.deepcopy(result["inputs"]))
                                      for name in ["super_source", "vectors0"]]
        return spec, result

    def test_auxiliary_inputs_are_required_and_compared_first(self):
        spec, result = self.render_result()
        validate_result(result, spec, "mvu")
        missing = copy.deepcopy(result)
        missing["auxiliary_inputs"].pop()
        with self.assertRaises(ValueError):
            validate_result(missing, spec, "mvu")
        changed = copy.deepcopy(result)
        changed["auxiliary_inputs"][1]["frames"][0]["properties"].clear()
        self.assertEqual(compare(result, changed)[0], "input_mismatch")
        changed = copy.deepcopy(result)
        changed["input_video"]["width"] = 2
        self.assertEqual(compare(result, changed)[0], "input_mismatch")
        del changed["input_video"]
        with self.assertRaises(ValueError):
            validate_result(changed, spec, "mvu")

    def test_observed_errors_stay_red_including_matching_rejections(self):
        spec, good = self.render_result()
        creation = copy.deepcopy(good)
        creation.update(outputs=[], records=[], creation_error=dict(type="Error", message="original detail"))
        validate_result(creation, spec, "mvu")
        for pair in [(creation, good), (good, creation), (creation, creation)]:
            self.assertEqual(compare(*pair)[0], "difference")
        failed = copy.deepcopy(good)
        failed["records"][0] = dict(request=0, member=0, frame=2,
                                     error=dict(type="Error", message="upstream detail"))
        validate_result(failed, spec, "mvu")
        for pair in [(failed, good), (good, failed), (failed, failed)]:
            self.assertEqual(compare(*pair)[0], "difference")
        failed["records"][0]["error"]["message"] = ""
        with self.assertRaises(ValueError):
            validate_result(failed, spec, "mvu")

    def test_one_float_bit_is_a_difference(self):
        reference, candidate = copy.deepcopy(self.result), copy.deepcopy(self.result)
        # Adjacent IEEE-754 float32 values: 1.0 and 1.0 + 2**-23.
        for result, bits in [(reference, 0x3f800000), (candidate, 0x3f800001)]:
            data = bits.to_bytes(4, "little")
            result["records"][0]["planes"][0].update(sample_bytes=4, data=data.hex(),
                                                       sha256=hashlib.sha256(data).hexdigest())
            validate_result(result, self.spec, "mvu")
        self.assertEqual(compare(reference, candidate)[0], "difference")
        self.assertEqual(compare(reference, reference), ("pass", None))


if __name__ == "__main__":
    unittest.main()
