"""Test the oracle transport without loading any video plugin."""
import copy
import hashlib
import unittest

from cases import CASES
from protocol import SCHEMA, compare, digest_json, validate_result
from worker import configure_kernel


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


if __name__ == "__main__":
    unittest.main()
