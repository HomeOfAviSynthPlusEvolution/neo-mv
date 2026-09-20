"""Run deterministic compatibility cases against installed release binaries.

No benchmarks, timings, source checkout or package installation are performed.
Use --list to inspect cases. Reports are stored in a fresh directory on each run.
Exit status: 0 = all pass; 1 = observed differences; 2 = incomplete/failed run.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from catalog import BY_ID, CASES
from protocol import SCHEMA, compare, digest_file, validate_result


def git_state(root):
    def git(*args):
        process = subprocess.run(["git", "-c", "safe.directory=" + root.as_posix(),
                                  "-C", str(root), *args], capture_output=True,
                                 text=True, timeout=10)
        return process.stdout.strip() if process.returncode == 0 else None
    try:
        return dict(commit=git("rev-parse", "HEAD"), status=git("status", "--porcelain"))
    except (OSError, subprocess.TimeoutExpired):
        return dict(commit=None, status=None)


def execute(spec, backend, args, directory):
    output = directory / f"{spec['id']}.{backend}.json"
    command = [sys.executable, str(Path(__file__).with_name("worker.py")),
               "--backend", backend, "--case", spec["id"], "--output", str(output),
               "--threads", str(args.threads), "--vs-version", args.vs_version,
               "--mvu-version", args.mvu_version,
               "--kernel", args.neo_kernel if backend == "neo" else "auto"]
    if backend == "neo":
        command += ["--plugin", str(args.plugin)]
    environment = os.environ.copy()
    if backend == "neo":
        # Set before process startup: Windows CRTs can cache separate environments.
        environment["NEO_MV_KERNEL"] = args.neo_kernel
    # File redirection avoids buffering arbitrary host diagnostics in memory.
    with output.with_suffix(".stdout.log").open("wb") as stdout, \
         output.with_suffix(".stderr.log").open("wb") as stderr:
        try:
            process = subprocess.run(command, stdout=stdout, stderr=stderr, timeout=args.timeout, env=environment)
        except subprocess.TimeoutExpired:
            return dict(status="timeout", command=command), None
        except OSError as error:
            return dict(status="launch_error", message=str(error), command=command), None
    if not output.is_file():
        return dict(status="missing_result", returncode=process.returncode, command=command), None
    try:
        result = json.loads(output.read_text(encoding="utf-8"))
        if process.returncode != 0:
            return dict(status="worker_error", returncode=process.returncode,
                        stage=result.get("stage"), error=result.get("error"), command=command), None
        validate_result(result, spec, backend)
    except (ValueError, TypeError, KeyError, AttributeError) as error:
        return dict(status="invalid_result", message=str(error), command=command), None
    return dict(status="ok", result=output.name, command=command), result


def write_report(directory, report):
    (directory / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    counts = Counter(item["status"] for item in report["cases"])
    lines = ["# Binary compatibility report", "", f"Status: {report['status']}",
             f"Completed: {len(report['cases'])}/{len(report['selected_cases'])}",
             f"Counts: {dict(counts)}", "", "| Case | Result |", "| --- | --- |"]
    lines += [f"| {item['id']} | {item['status']} |" for item in report["cases"]]
    for item in report["cases"]:
        if item.get("difference"):
            lines += ["", "## " + item["id"], "", "```json",
                      json.dumps(item["difference"], indent=2), "```"]
    lines += ["", "This is a binary compatibility result, not a specification verdict or performance measurement.",
              "Worker JSON and logs retain the full observations and errors. No reference private payload is exported."]
    (directory / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, help="candidate plugin built from neo-mv")
    parser.add_argument("--case", action="append", choices=BY_ID, dest="cases", help="repeat to select cases")
    parser.add_argument("--list", action="store_true", help="list cases without importing VS")
    parser.add_argument("--phase", type=int, choices=[1, 2, 3], help="select one development phase")
    parser.add_argument("--threads", type=int, choices=[1, 4], default=1)
    parser.add_argument("--timeout", type=float, default=30, help="per backend/case timeout in seconds")
    parser.add_argument("--output-root", type=Path, default=Path(__file__).resolve().parents[2] / "build" / "blackbox")
    parser.add_argument("--vs-version", default="79")
    parser.add_argument("--mvu-version", default="8")
    parser.add_argument("--neo-kernel", choices=["scalar", "highway"], default="scalar",
                        help="select and query the loaded candidate backend; highway requires a real SIMD target")
    args = parser.parse_args()
    if args.cases and len(args.cases) != len(set(args.cases)):
        parser.error("duplicate case selection")
    selected = [BY_ID[key] for key in args.cases] if args.cases else CASES
    if args.phase is not None:
        selected = [spec for spec in selected if spec.get("phase", 1) == args.phase]
    if not selected:
        parser.error("no cases selected")
    if args.list:
        for spec in selected:
            print(spec["id"])
        return 0
    if args.plugin is None or not args.plugin.is_file():
        parser.error("--plugin must name an existing candidate binary")
    if not 0 < args.timeout <= 300:
        parser.error("--timeout must be in (0,300]")
    args.plugin = args.plugin.resolve()
    args.output_root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="run-", dir=args.output_root.resolve()))
    report = dict(schema=SCHEMA, status="incomplete", started_utc=datetime.now(timezone.utc).isoformat(),
                  git=git_state(Path(__file__).resolve().parents[2]),
                  candidate=dict(path=str(args.plugin), sha256=digest_file(args.plugin), kernel=args.neo_kernel),
                  expected=dict(vs=args.vs_version, mvu=args.mvu_version), threads=args.threads,
                  harness_sha256={p.name: digest_file(p) for p in sorted(Path(__file__).parent.glob("*.py"))},
                  selected_cases=selected, cases=[])
    print(f"Report directory: {directory}", flush=True)
    write_report(directory, report)
    for spec in selected:
        workers, results = {}, {}
        # Serial subprocesses, each with <=4 VS threads. No shared graphs/results.
        for backend in ("mvu", "neo"):
            workers[backend], results[backend] = execute(spec, backend, args, directory)
        if any(value is None for value in results.values()):
            status, difference = "execution_error", None
        else:
            status, difference = compare(results["mvu"], results["neo"])
        report["cases"].append(dict(id=spec["id"], status=status, difference=difference, workers=workers))
        write_report(directory, report)
        print(f"{spec['id']}: {status}", flush=True)
    statuses = {item["status"] for item in report["cases"]}
    code = 2 if statuses & {"execution_error", "input_mismatch"} else 1 if "difference" in statuses else 0
    report["status"] = "pass" if code == 0 else "error" if code == 2 else "difference"
    write_report(directory, report)
    return code


if __name__ == "__main__":
    sys.exit(main())
