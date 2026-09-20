"""Test the oracle transport without loading any video plugin."""
import copy
from enum import Enum, IntEnum
import hashlib
import unittest
from types import SimpleNamespace

from cases import CASES
from protocol import SCHEMA, compare, digest_json, validate_result
from worker import configure_kernel, property_value, snapshot
from render_cases import CASES as RENDER_CASES
from mask_cases import CASES as MASK_CASES
from flow_cases import CASES as FLOW_CASES
from interpolation_cases import CASES as INTERPOLATION_CASES
from depan_cases import CASES as DEPAN_CASES


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

    def test_native_integer_enum_binding_preserves_exact_values(self):
        class Range(IntEnum):
            LIMITED = 0
            FULL = 1
            LARGE = 2**60 + 1
        class Other(Enum):
            NUMBER = 1
        self.assertEqual(property_value(Range.FULL), dict(type="int", count=1, values=[1]))
        self.assertEqual(property_value([Range.LIMITED, 1, Range.LARGE]),
                         dict(type="int", count=3, values=[0, 1, 2**60 + 1]))
        for item in property_value([Range.LIMITED, Range.FULL])["values"]:
            self.assertIs(type(item), int)
        for value in [True, Other.NUMBER, [Range.FULL, 1.0], [Range.FULL, b"1"]]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                property_value(value)
        self.assertEqual(property_value(1.0)["type"], "float64")

    def test_utf8_data_property(self):
        self.assertEqual(property_value("zoom=1.00000"), property_value(b"zoom=1.00000"))
        self.assertEqual(property_value(["a", "\u4e2d"]), dict(type="data", count=2, values=["61", "e4b8ad"]))

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

    def test_snapshot_does_not_invent_deprecated_property_aliases(self):
        class AliasedProperties(dict):
            def __contains__(self, key):
                return key == "_ColorRange" or super().__contains__(key)

            def __getitem__(self, key):
                return super().__getitem__("_Range" if key == "_ColorRange" else key)

        frame = SimpleNamespace(format=SimpleNamespace(num_planes=0), props=AliasedProperties(_Range=1))
        self.assertIn("_ColorRange", frame.props)
        self.assertEqual(snapshot(frame, ["_Range", "_ColorRange"])["properties"],
                         {"_Range": dict(type="int", count=1, values=[1])})

    def mask_result(self):
        spec = MASK_CASES[0]
        result = copy.deepcopy(self.result)
        result.update(case_id=spec["id"], case_sha256=digest_json(spec),
                      input_video=dict(width=4, height=4, bits=16),
                      outputs=[dict(width=8, height=8, bits=8)])
        result["inputs"] = result["inputs"][:spec["length"]]
        sample = {key: copy.deepcopy(value) for key, value in result["records"][0].items()
                  if key not in ("member", "frame", "request")}
        sample.update(properties={"_Range": dict(type="int", count=1, values=[1])},
                      property_names=["_Range"])
        result["records"] = [dict(request=i, member=m, frame=n, **copy.deepcopy(sample))
                              for i, (m, n) in enumerate(spec["requests"])]
        return spec, result

    def test_mask_output_format_is_independent_of_carrier(self):
        spec, result = self.mask_result()
        validate_result(result, spec, "mvu")
        self.assertEqual(compare(result, result), ("pass", None))
        changed = copy.deepcopy(result)
        changed["outputs"][0]["bits"] = 16
        self.assertEqual(compare(result, changed)[0], "difference")
        self.assertEqual(compare(result, changed)[1]["path"], "/outputs/0/bits")
        changed = copy.deepcopy(result)
        del changed["input_video"]
        with self.assertRaises(ValueError):
            validate_result(changed, spec, "mvu")

    def test_mask_property_inventory_observes_unknown_keys_without_payloads(self):
        spec, result = self.mask_result()
        for names in [None, [], ["_Range", "_Range"], [42]]:
            changed = copy.deepcopy(result)
            changed["records"][0]["property_names"] = names
            with self.assertRaises(ValueError):
                validate_result(changed, spec, "mvu")
        changed = copy.deepcopy(result)
        changed["records"][0]["property_names"] = ["UnexpectedOpaqueProperty", "_Range"]
        validate_result(changed, spec, "mvu")
        self.assertEqual(compare(result, changed)[0], "difference")
        self.assertIn("/property_names", compare(result, changed)[1]["path"])
        self.assertNotIn("UnexpectedOpaqueProperty", changed["records"][0]["properties"])
        for key in ["_Range", "TestMarker", spec["prefix"] + "AnalysisVectors"]:
            changed = copy.deepcopy(result)
            changed["records"][0]["property_names"] = sorted({"_Range", key})
            changed["records"][0]["properties"].pop(key, None)
            with self.subTest(missing=key), self.assertRaises(ValueError):
                validate_result(changed, spec, "mvu")

    def test_mask_creation_errors_are_observed_and_remain_red(self):
        spec, result = self.mask_result()
        failed = copy.deepcopy(result)
        failed.update(outputs=[], records=[], creation_error=dict(type="Error", message="mask creation detail"))
        validate_result(failed, spec, "mvu")
        self.assertEqual(compare(failed, result)[0], "difference")
        self.assertEqual(compare(failed, failed)[0], "difference")

    def mask_range_pair(self):
        spec, candidate = self.mask_result()
        candidate["backend"] = "neo"
        candidate["environment"]["vs_package"] = "79"
        reference = copy.deepcopy(candidate)
        reference["backend"] = "mvu"
        reference["environment"]["mvu_package"] = "8"
        for record in reference["records"]:
            record["properties"]["_Range"]["values"] = [0]
        return spec, reference, candidate

    def test_exact_mask_range_exception_retains_raw_observations(self):
        spec, reference, candidate = self.mask_range_pair()
        before = copy.deepcopy((reference, candidate))
        self.assertEqual(compare(reference, candidate)[0], "difference")
        status, detail = compare(reference, candidate, spec)
        self.assertEqual(status, "known_difference")
        self.assertEqual(len(detail["known_differences"]), len(spec["requests"]))
        for i, observed in enumerate(detail["known_differences"]):
            self.assertEqual(observed["rule"], "mvu8-vs79-mask-range")
            self.assertEqual(observed["reference"], dict(type="int", count=1, values=[0]))
            self.assertEqual(observed["candidate"], dict(type="int", count=1, values=[1]))
            self.assertEqual(observed["request"]["request"], i)
        self.assertEqual((reference, candidate), before)
        self.assertEqual(compare(candidate, candidate, spec), ("pass", None))

    def test_mask_range_exception_has_narrow_scope(self):
        spec, reference, candidate = self.mask_range_pair()
        for changed_spec in [None, dict(spec, phase=2), dict(spec, operation="Flow")]:
            self.assertEqual(compare(reference, candidate, changed_spec)[0], "difference")
        for side, key, value in [(0, "mvu_package", "9"), (0, "vs_package", "80"),
                                 (1, "vs_package", "80")]:
            pair = copy.deepcopy([reference, candidate])
            pair[side]["environment"][key] = value
            self.assertEqual(compare(*pair, spec)[0], "difference")
        self.assertEqual(compare(candidate, reference, spec)[0], "difference")

    def test_mask_range_wrong_payload_and_missing_name_stay_red(self):
        spec, reference, candidate = self.mask_range_pair()
        for wrong in [dict(type="int", count=1, values=[2]),
                      dict(type="int", count=2, values=[1, 1]),
                      dict(type="float64", count=1, values=["3ff0000000000000"]),
                      dict(type="int", count=True, values=[True]), None]:
            changed = copy.deepcopy(candidate)
            if wrong is None:
                changed["records"][-1]["properties"].pop("_Range")
            else:
                changed["records"][-1]["properties"]["_Range"] = wrong
            self.assertEqual(compare(reference, changed, spec)[0], "difference")
        changed = copy.deepcopy(candidate)
        changed["records"][-1]["property_names"] = []
        self.assertEqual(compare(reference, changed, spec)[0], "difference")

    def test_range_exception_does_not_hide_other_differences(self):
        spec, reference, candidate = self.mask_range_pair()
        for mutate in [lambda r: r["outputs"][0].update(bits=16),
                       lambda r: r["records"][-1]["properties"].update(TestMarker=dict(type="int", count=1, values=[7])),
                       lambda r: r["records"][-1]["property_names"].append("Unexpected"),
                       lambda r: r["records"][-1]["planes"][0].update(data="01"),
                       lambda r: r["records"][-1].update(frame=99)]:
            changed = copy.deepcopy(candidate)
            mutate(changed)
            status, detail = compare(reference, changed, spec)
            self.assertEqual(status, "difference")
            self.assertIn("known_differences", detail)
        changed = copy.deepcopy(candidate)
        changed["inputs"][0]["properties"]["_Range"] = dict(type="int", count=1, values=[1])
        self.assertEqual(compare(reference, changed, spec)[0], "input_mismatch")
        changed = copy.deepcopy(candidate)
        changed["records"][-1] = dict(request=4, member=0, frame=0, error=dict(type="Error", message="failed"))
        status, detail = compare(reference, changed, spec)
        self.assertEqual(status, "difference")
        self.assertEqual(len(detail["known_differences"]), len(spec["requests"]) - 1)

    def flow_result(self):
        _, result = self.render_result()
        spec = FLOW_CASES[0]
        result.update(case_id=spec["id"], case_sha256=digest_json(spec))
        for record in result["records"]:
            record["property_names"] = sorted(record["properties"])
        return spec, result

    def test_flow_requires_and_compares_reference_source_and_vector_inputs(self):
        spec, result = self.flow_result()
        validate_result(result, spec, "mvu")
        self.assertEqual(compare(result, result, spec), ("pass", None))
        for mutate in [lambda r: r.pop("auxiliary_inputs"),
                       lambda r: r["auxiliary_inputs"].reverse(),
                       lambda r: r["auxiliary_inputs"][1]["frames"].pop(),
                       lambda r: r["auxiliary_inputs"][0].pop("video")]:
            changed = copy.deepcopy(result)
            mutate(changed)
            with self.assertRaises(ValueError):
                validate_result(changed, spec, "mvu")
        for index in (0, 1):
            changed = copy.deepcopy(result)
            changed["auxiliary_inputs"][index]["frames"][0]["properties"].clear()
            self.assertEqual(compare(result, changed, spec)[0], "input_mismatch")

    def test_flow_preserves_range_comparison_without_mask_exception(self):
        spec, reference = self.flow_result()
        reference["environment"].update(vs_package="79", mvu_package="8")
        candidate = copy.deepcopy(reference)
        candidate["backend"] = "neo"
        for result, value in [(reference, 0), (candidate, 1)]:
            for record in result["records"]:
                record["properties"]["_Range"] = dict(type="int", count=1, values=[value])
                record["property_names"] = sorted(record["properties"])
            validate_result(result, spec, result["backend"])
        status, detail = compare(reference, candidate, spec)
        self.assertEqual(status, "difference")
        self.assertNotIn("known_differences", detail)
        changed = copy.deepcopy(candidate)
        changed["records"][0]["properties"].pop("_Range")
        with self.assertRaises(ValueError):
            validate_result(changed, spec, "neo")

    def interpolation_result(self):
        _, result = self.render_result()
        spec = next(item for item in INTERPOLATION_CASES if item["operation"] == "FlowFPS")
        result.update(case_id=spec["id"], case_sha256=digest_json(spec),
                      outputs=[dict(width=32, height=24, length=10, fps=[48000, 1001])])
        extra = copy.deepcopy(result["auxiliary_inputs"][-1])
        extra["name"] = "vectors1"
        result["auxiliary_inputs"].append(extra)
        sample = {key: copy.deepcopy(value) for key, value in result["records"][0].items()
                  if key not in ("request", "member", "frame")}
        sample["properties"].update(_DurationNum=dict(type="int", count=1, values=[1001]),
                                     _DurationDen=dict(type="int", count=1, values=[48000]),
                                     _Range=dict(type="int", count=1, values=[1]))
        sample["property_names"] = sorted(sample["properties"])
        result["records"] = [dict(request=i, member=m, frame=n, **copy.deepcopy(sample))
                              for i, (m, n) in enumerate(spec["requests"])]
        return spec, result

    def test_interpolation_requires_both_vector_nodes_and_inventory(self):
        spec, result = self.interpolation_result()
        validate_result(result, spec, "mvu")
        for mutate in [lambda r: r["auxiliary_inputs"].pop(),
                       lambda r: r["auxiliary_inputs"][-1]["frames"].pop(),
                       lambda r: r["records"][0].pop("property_names"),
                       lambda r: r["records"][0]["properties"].pop("_DurationNum")]:
            changed = copy.deepcopy(result)
            mutate(changed)
            with self.assertRaises(ValueError):
                validate_result(changed, spec, "mvu")

    def test_interpolation_duration_rate_pixels_and_range_are_strict(self):
        spec, reference = self.interpolation_result()
        reference["environment"].update(vs_package="79", mvu_package="8")
        for mutate in [lambda r: r["outputs"][0].update(length=11),
                       lambda r: r["outputs"][0].update(fps=[48000, 1000]),
                       lambda r: r["records"][0]["properties"]["_DurationDen"].update(values=[24000]),
                       lambda r: r["records"][0]["properties"]["_Range"].update(values=[0]),
                       lambda r: r["records"][0]["planes"][0].update(data="01")]:
            candidate = copy.deepcopy(reference)
            candidate["backend"] = "neo"
            mutate(candidate)
            status, detail = compare(reference, candidate, spec)
            self.assertEqual(status, "difference")
            self.assertNotIn("known_differences", detail)
        failed = copy.deepcopy(reference)
        failed.update(outputs=[], records=[], creation_error=dict(type="Error", message="phase four detail"))
        validate_result(failed, spec, "mvu")
        self.assertEqual(compare(failed, failed, spec)[0], "difference")

    def test_interpolation_inventory_has_valid_bounded_requests(self):
        from fractions import Fraction
        self.assertGreaterEqual(len(INTERPOLATION_CASES), 30)
        self.assertLessEqual(len(INTERPOLATION_CASES), 50)
        self.assertEqual({item["operation"] for item in INTERPOLATION_CASES}, {"FlowInter", "FlowFPS", "FlowBlur"})
        for spec in INTERPOLATION_CASES:
            with self.subTest(case=spec["id"]):
                self.assertEqual(spec["phase"], 4)
                self.assertGreater(spec["deltas"][0], 0)
                self.assertEqual(spec["deltas"][0], -spec["deltas"][1])
                length = spec["length"]
                if spec["operation"] == "FlowFPS":
                    source = Fraction(*spec.get("fps", [24000, 1001]))
                    num, den = spec["params"].get("num", 25), spec["params"].get("den", 1)
                    target = Fraction(num, den) if num and den else 2*source
                    length = int(length * target / source)
                for member, n in spec["requests"]:
                    self.assertEqual(member, 0)
                    self.assertGreaterEqual(n, 0)
                    self.assertLess(n, length)

    def depan_result(self, spec=None):
        _, result = self.render_result()
        spec = spec or DEPAN_CASES[0]
        result.update(case_id=spec["id"], case_sha256=digest_json(spec))
        names = (["vectors"] + (["mask"] if spec["mask"] is not None else [])) \
            if spec["operation"] == "DepanAnalyse" else ["data"]
        result["auxiliary_inputs"] = [dict(name=name, video={"width": 1}, frames=copy.deepcopy(result["inputs"]))
                                      for name in names]
        sample = {key: copy.deepcopy(value) for key, value in result["records"][0].items()
                  if key not in ("request", "member", "frame")}
        sample["properties"].update(
            Depan_dx=dict(type="float64", count=1, values=["3fe0000000000000"]),
            Depan_dy=dict(type="float64", count=1, values=["0000000000000000"]),
            Depan_rot=dict(type="float64", count=1, values=["8000000000000000"]),
            Depan_zoom=dict(type="float64", count=1, values=["3ff0000000000000"]),
            Depan_goodmotion=dict(type="int", count=1, values=[1]),
            DepanAnalyse_info=dict(type="data", count=1, values=[b"inherited analysis".hex()]),
            DepanCompensate_info=dict(type="data", count=1, values=[b"inherited compensation".hex()]))
        sample["property_names"] = sorted(sample["properties"])
        result["records"] = [dict(request=i, member=m, frame=n, **copy.deepcopy(sample))
                              for i, (m, n) in enumerate(spec["requests"])]
        return spec, result

    def test_depan_public_motion_signed_zero_and_diagnostics_are_strict(self):
        spec, reference = self.depan_result()
        validate_result(reference, spec, "mvu")
        for key, value in [("Depan_dx", "3fe0000000000001"), ("Depan_rot", "0000000000000000"),
                           ("DepanAnalyse_info", b"different diagnostic".hex())]:
            candidate = copy.deepcopy(reference)
            candidate["records"][0]["properties"][key]["values"] = [value]
            status, detail = compare(reference, candidate, spec)
            self.assertEqual(status, "difference")
            self.assertNotIn("known_differences", detail)
        missing = copy.deepcopy(reference)
        del missing["records"][0]["properties"]["Depan_zoom"]
        with self.assertRaises(ValueError):
            validate_result(missing, spec, "mvu")

    def test_depan_requires_exact_auxiliary_inputs(self):
        for operation in ["DepanAnalyse", "DepanCompensate"]:
            spec, reference = self.depan_result(next(item for item in DEPAN_CASES if item["operation"] == operation))
            validate_result(reference, spec, "mvu")
            missing = copy.deepcopy(reference)
            missing["auxiliary_inputs"].pop()
            with self.assertRaises(ValueError):
                validate_result(missing, spec, "mvu")
            changed = copy.deepcopy(reference)
            changed["auxiliary_inputs"][-1]["frames"][0]["properties"].clear()
            self.assertEqual(compare(reference, changed, spec)[0], "input_mismatch")
            failed = copy.deepcopy(reference)
            failed.update(outputs=[], records=[], creation_error=dict(type="Error", message="Depan creation detail"))
            validate_result(failed, spec, "mvu")
            self.assertEqual(compare(failed, failed, spec)[0], "difference")

    def test_depan_info_binds_external_or_builtin_renderer_and_defaults(self):
        spec, reference = self.depan_result(next(item for item in DEPAN_CASES if item["params"].get("info")))
        with self.assertRaises(ValueError):
            validate_result(reference, spec, "mvu")
        reference["environment"]["text_renderer"] = dict(plugin_path="text.dll", plugin_sha256="renderer hash",
            plugin_version="1", builtin_core=None, entry="text.FrameProps",
            arguments=dict(props=[spec["operation"] + "_info"]))
        validate_result(reference, spec, "mvu")
        changed = copy.deepcopy(reference)
        changed["environment"]["text_renderer"]["plugin_sha256"] = "other renderer"
        self.assertEqual(compare(reference, changed, spec)[0], "input_mismatch")
        changed = copy.deepcopy(reference)
        changed["environment"]["text_renderer"]["arguments"]["scale"] = 2
        with self.assertRaises(ValueError):
            validate_result(changed, spec, "mvu")
        builtin = copy.deepcopy(reference)
        builtin["environment"]["text_renderer"].update(plugin_path=None, plugin_sha256=None, builtin_core="R79")
        validate_result(builtin, spec, "mvu")

    def test_depan_fixture_inventory_is_bounded_and_names_all_public_entries(self):
        self.assertGreaterEqual(len(DEPAN_CASES), 40)
        self.assertLessEqual(len(DEPAN_CASES), 60)
        self.assertEqual({item["operation"] for item in DEPAN_CASES}, {"DepanAnalyse", "DepanCompensate"})
        for spec in DEPAN_CASES:
            with self.subTest(case=spec["id"]):
                self.assertEqual(spec["phase"], 5)
                if spec["operation"] == "DepanAnalyse":
                    self.assertIn(spec["delta"], (-1, 1))
                for member, n in spec["requests"]:
                    self.assertEqual(member, 0)
                    self.assertGreaterEqual(n, 0)
                    self.assertLess(n, spec["length"])


if __name__ == "__main__":
    unittest.main()
