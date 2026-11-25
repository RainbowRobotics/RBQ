#!/usr/bin/env python3
"""Checks the RBA-1 arm examples' command lines - no robot or simulator needed, a few seconds.

Every example reads its arguments, and with RBQ_ARM_DRY_RUN=1 in the environment prints the call it would make
("[dry-run] <target> <call>", arm_helper DryRun / dry_run) and exits before connecting. This runs:

  cases     the table in arm_examples_cases.txt - arguments, exit code, what the output must (not) say
  examples  every command line the examples themselves show - in -h and in the hint printed without arguments -
            and 00_arm_help's list: each must run (exit 0, a [dry-run] line). A help text that shows a command
            the example refuses fails here
  parity    each of the above on the C++ and on the Python example: the two must print the same

Usage:
  python3 rbq_sdk/scripts/arm_examples_check.py [--cpp-bin DIR] [--python PY] [--only cpp|py] [-v]
    --cpp-bin  the built C++ examples (default rbq_sdk/cpp/example/bin-x86_64 - rbq_sdk/cpp/example/scripts/build.bash)
    --python   the Python that runs the Python examples; it needs cyclonedds (default: this one).
               Without cyclonedds the Python half is skipped - pip install cyclonedds==11.0.1
    -v         print every command and its output

Exit code 0 when everything passed.
"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

SDK = Path(__file__).resolve().parent.parent
CASES = Path(__file__).resolve().parent / "arm_examples_cases.txt"
PY_DIR = SDK / "python" / "example" / "arm"
EXAMPLES = ["00_arm_help", "01_arm_state", "02_arm_power", "03_arm_gripper", "04_arm_move_j", "05_arm_move_l",
            "06_arm_high_level", "07_arm_low_level", "08_arm_torque"]
# a command line an example shows: optional "python3", optional "./", the example, then its arguments
COMMAND = re.compile(r"^(?:python3\s+)?(?:\./)?(0[0-8]_arm_\w+?)(?:\.py)?(?:\s+(.*))?$")


class Runner:
    def __init__(self, cpp_bin: Path, python: str, langs, verbose: bool):
        self.cpp_bin, self.python, self.langs, self.verbose = cpp_bin, python, langs, verbose
        self.env = {**os.environ, "RBQ_ARM_DRY_RUN": "1", "LD_LIBRARY_PATH": str(cpp_bin),
                    "PYTHONPATH": str(SDK / "python"), "PYTHONDONTWRITEBYTECODE": "1"}
        self.cache = {}

    def run(self, lang: str, example: str, args):
        """(exit code, output with the program name made the same for both languages)"""
        key = (lang, example, tuple(args))
        if key not in self.cache:
            if lang == "cpp":
                cmd, cwd = [f"./{example}", *args], self.cpp_bin
            else:
                cmd, cwd = [self.python, f"{example}.py", *args], PY_DIR
            try:
                p = subprocess.run(cmd, cwd=cwd, env=self.env, capture_output=True, text=True, timeout=15)
                out, code = p.stdout + p.stderr, p.returncode
            except subprocess.TimeoutExpired:
                out, code = "(timed out after 15 s - did it try to connect?)", -1
            out = re.sub(r"python3 (0[0-8]_arm_\w+|my_arm)\.py", r"\1", out)
            out = re.sub(r"(?<![\w/])\./(0[0-8]_arm_\w+|my_arm)", r"\1", out)
            out = re.sub(r"(0[0-8]_arm_\w+|my_arm)\.py", r"\1", out)
            out = "\n".join(l for l in out.splitlines() if "is not multicast-capable" not in l)
            out = re.sub(r" {2,}", "  ", out)   # column widths differ with the longer "python3 x.py" names
            self.cache[key] = (code, out)
            if self.verbose:
                print(f"  $ {' '.join(cmd)}  -> {code}\n" + "".join(f"    | {l}\n" for l in out.splitlines()))
        return self.cache[key]


def read_cases():
    cases = []
    for n, line in enumerate(CASES.read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        cols = [c.strip() for c in line.split("|")]
        if len(cols) < 3:
            sys.exit(f"{CASES.name}:{n}: needs example | arguments | exit code")
        example = next((e for e in EXAMPLES if e.startswith(cols[0] + "_")), None)
        if example is None:
            sys.exit(f"{CASES.name}:{n}: no example {cols[0]}")
        cases.append((n, example, cols[1].split(), int(cols[2]), [c for c in cols[3:] if c]))
    return cases


def shown_commands(runner: Runner, lang: str):
    """Every runnable command line an example prints in -h or without arguments: [(where, example, args)]"""
    found = []
    for example in EXAMPLES:
        outs = [("no arguments", runner.run(lang, example, [])[1])]
        if example != "00_arm_help":
            outs.append(("-h", runner.run(lang, example, ["-h"])[1]))
        for where, out in outs:
            for line in out.splitlines():
                # "04_arm_move_j sim 1                          to Ready" - the command ends at the gap before the note
                m = COMMAND.match(re.split(r"\s{2,}", line.strip())[0])
                if not m or re.search(r"[\[\]<>|()]|\.\.\.", m.group(2) or ""):
                    continue   # not a command, a usage form ("[target] <pose>") or prose ("01_arm_state shows (x ...")
                found.append((f"{example} {where}", m.group(1), (m.group(2) or "").split()))
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cpp-bin", default=str(SDK / "cpp" / "example" / "bin-x86_64"))
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--only", choices=["cpp", "py"])
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    langs = [a.only] if a.only else ["cpp", "py"]
    cpp_bin = Path(a.cpp_bin).resolve()
    if "cpp" in langs and not (cpp_bin / "04_arm_move_j").exists():
        sys.exit(f"no C++ examples in {cpp_bin} - build them: bash rbq_sdk/cpp/example/scripts/build.bash (or --only py)")
    if "py" in langs and subprocess.run([a.python, "-c", "import cyclonedds"], capture_output=True).returncode != 0:
        print(f"[skip] Python examples: {a.python} has no cyclonedds - pip install cyclonedds==11.0.1, or --python <venv>/bin/python")
        langs.remove("py")
        if not langs:
            return 1
    runner = Runner(cpp_bin, a.python, langs, a.verbose)
    failures = []

    def fail(what, why, out=""):
        failures.append(f"FAIL {what}: {why}" + "".join(f"\n       | {l}" for l in out.splitlines()))

    # cases
    cases = read_cases()
    for n, example, args, code_want, checks in cases:
        for lang in langs:
            code, out = runner.run(lang, example, args)
            what = f"[{lang}] {example} {' '.join(args)}  ({CASES.name}:{n})"
            if code != code_want:
                fail(what, f"exit {code}, want {code_want}", out)
            for c in checks:
                if c.startswith("!") and c[1:] in out:
                    fail(what, f"says '{c[1:]}'", out)
                elif not c.startswith("!") and c not in out:
                    fail(what, f"does not say '{c}'", out)

    # the commands the examples show must run
    shown = {}
    for lang in langs:
        for where, example, args in shown_commands(runner, lang):
            shown.setdefault((example, tuple(args)), []).append(f"{lang} {where}")
            code, out = runner.run(lang, example, args)
            if code != 0 or "[dry-run]" not in out:
                fail(f"[{lang}] {example} {' '.join(args)}  (shown in {where})", f"exit {code}, no [dry-run] line", out)

    # C++ and Python print the same
    parity = 0
    if len(langs) == 2:
        for example, args in sorted({(e, tuple(a)) for _, e, a, _, _ in cases} | set(shown)):
            c, p = runner.run("cpp", example, list(args)), runner.run("py", example, list(args))
            parity += 1
            if c != p:
                fail(f"{example} {' '.join(args)}", "C++ and Python differ",
                     f"C++ exit {c[0]}:\n{c[1]}\nPython exit {p[0]}:\n{p[1]}")

    for f in failures:
        print(f)
    print(f"\n{len(cases)} cases x {'+'.join(langs)}, {len(shown)} shown commands, {parity} parity checks - "
          f"{'all passed' if not failures else f'{len(failures)} failed'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
