#!/usr/bin/env python3
"""Build bs_v11 on Windows/Linux x64 without CMake or third-party Python packages.

Examples (run with `python`, or `python3` where installed):
    python tools/build.py --baseline
    python tools/build.py --cxx clang++ --build-dir build/clang --run-tests
    python tools/build.py --debug --sanitize address,undefined --run-tests
    python tools/build.py --native --lto --baseline --build-dir build/native

Release uses C++20, O3, NDEBUG, pthread, no fast-math and no FP contraction.
The default x86-64/generic CPU target is portable across x64 machines. --native
is explicitly machine-specific; --lto requires working compiler/linker support.
Baseline and application receive identical options. Test assertions remain on.
No PGO or arbitrary extra flags are accepted; CXXFLAGS/LDFLAGS are intentionally
not imported, so they cannot silently change floating-point semantics. Use CMake
for MSVC. CXX may name one GCC/Clang executable (not a shell command or flags).
"""

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
TEST_NAMES = ("geometry_tests", "solver_tests", "tail_cache_tests", "progress_tests", "heuristic_tests", "r2_tests", "r3_tests", "coverage_tests", "equal_radius_tests", "novelty_tests", "goal_finish_tests", "prerequisite_tests")


def command_text(command):
    if os.name == "nt":
        return subprocess.list2cmdline([str(arg) for arg in command])
    return shlex.join([str(arg) for arg in command])


def run(command, **kwargs):
    command = [str(arg) for arg in command]
    print("+ " + command_text(command), flush=True)
    return subprocess.run(command, check=True, **kwargs)


def compiler_path(requested):
    candidates = [requested] if requested else [os.environ.get("CXX"), "g++", "clang++"]
    for candidate in candidates:
        if candidate:
            found = shutil.which(candidate)
            if found:
                return str(Path(found).resolve())
            if requested or candidate == os.environ.get("CXX"):
                raise RuntimeError(
                    "Compiler not found: {!r}. Pass --cxx with one GCC/Clang executable; "
                    "do not include flags.".format(candidate)
                )
    raise RuntimeError("No g++ or clang++ found on PATH. Install an x64 C++20 toolchain or pass --cxx.")


def check_driver(compiler):
    result = subprocess.run(
        [compiler, "-dM", "-E", "-x", "c++", "-"],
        input="", text=True, encoding="utf-8", errors="replace",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30,
    )
    macros = result.stdout
    if result.returncode or not ("__GNUC__" in macros or "__clang__" in macros):
        raise RuntimeError("This script requires a GCC/Clang GNU-style driver; use CMake for MSVC.\n" + result.stderr)
    if "__x86_64__" not in macros and "_M_X64" not in macros:
        raise RuntimeError("The selected compiler does not target x64; select an x86_64 toolchain.")
    return "Clang" if "__clang__" in macros else "GCC"


def check_toolchain(compiler, flags, build_dir, check_assertions):
    """Compile, link AND run a tiny C++20/thread program with the requested flags.

    In particular MinGW often accepts sanitizer flags but lacks their libraries.
    Never silently drop a requested mode or emit an uninstrumented executable.
    """
    source = """#include <bit>
#include <cstdint>
#include <thread>
#if !defined(__x86_64__) && !defined(_M_X64)
#error An x64 compiler is required
#endif
#ifdef __FAST_MATH__
#error Fast math must remain disabled
#endif
static_assert(sizeof(void*) == 8);
static_assert(std::bit_cast<std::uint64_t>(1.0) == 0x3ff0000000000000ULL);
"""
    if check_assertions:
        source += "#ifdef NDEBUG\n#error Test assertions must remain enabled\n#endif\n"
    source += "int main() { int n = 0; std::thread t([&] { n = 1; }); t.join(); return n != 1; }\n"
    with tempfile.TemporaryDirectory(prefix="toolchain-check-", dir=build_dir) as temporary:
        directory = Path(temporary)
        cpp = directory / "probe.cpp"
        exe = directory / ("probe.exe" if os.name == "nt" else "probe")
        cpp.write_text(source, encoding="utf-8")
        command = [compiler, *flags, "-Werror", str(cpp), "-o", str(exe)]
        result = subprocess.run(
            command, cwd=ROOT, text=True, encoding="utf-8", errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120,
        )
        if result.returncode:
            raise RuntimeError(
                "Requested C++20/threads/optimization/sanitizer flags are not supported "
                "by this compiler and linker. No flags were silently removed.\n"
                + command_text(command) + "\n" + result.stdout
            )
        result = subprocess.run(
            [str(exe)], cwd=directory, text=True, encoding="utf-8", errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30,
        )
        if result.returncode:
            raise RuntimeError(
                "Toolchain runtime check failed (exit {}). Check compiler/sanitizer "
                "runtime DLLs or libraries.\n{}".format(result.returncode, result.stdout)
            )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cxx", help="GCC/Clang executable; default: CXX, g++, then clang++")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build", help="output directory (default: project/build)")
    parser.add_argument("--baseline", action="store_true", help="also build baseline_v9 using exactly the same flags")
    parser.add_argument("--debug", action="store_true", help="use O0/g3 and assertions instead of O3/NDEBUG")
    parser.add_argument("--native", action="store_true", help="opt in to -march=native; NOT portable across x64 CPUs")
    parser.add_argument("--lto", action="store_true", help="opt in to checked -flto compiler/linker support")
    parser.add_argument(
        "--sanitize", choices=("none", "address", "undefined", "address,undefined", "thread"), default="none",
        help="require sanitizer compile/link/runtime support (default: none; normally combine with --debug)",
    )
    parser.add_argument("--tests", action="store_true", help="also build existing geometry/solver/tail_cache/progress/heuristic C++ tests")
    parser.add_argument("--run-tests", action="store_true", help="build and run C++ tests; fail if none exist")
    return parser.parse_args()


def main():
    args = parse_args()
    compiler = compiler_path(args.cxx)
    family = check_driver(compiler)
    print("Compiler: {} ({})".format(compiler, family), flush=True)
    build_dir = args.build_dir.resolve()
    build_dir.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++20", "-Wall", "-Wextra", "-Wpedantic"]
    flags += ["-O0", "-g3", "-UNDEBUG"] if args.debug else ["-O3", "-DNDEBUG"]
    flags += ["-march=native"] if args.native else ["-march=x86-64", "-mtune=generic"]
    if args.lto:
        flags += ["-flto"]
    if args.sanitize != "none":
        flags += ["-fsanitize=" + args.sanitize, "-fno-omit-frame-pointer", "-fno-sanitize-recover=all"]
    # Keep these last, including on the link invocation, even in native/LTO mode.
    flags += ["-pthread", "-fno-fast-math", "-ffp-contract=off"]
    print("Mode: {} / {} / LTO={} / sanitizers={}".format(
        "debug" if args.debug else "release",
        "native (NOT portable)" if args.native else "portable x64",
        args.lto, args.sanitize,
    ), flush=True)

    # heuristic.hpp is header-only; main.cpp already includes it.
    targets = [("bs_v11", [ROOT / "src/main.cpp", ROOT / "src/reporting.cpp"], False)]
    if args.baseline:
        targets.append(("baseline_v9", [ROOT / "benchmarks/baseline_v9.cpp"], False))
    test_targets = []
    if args.tests or args.run_tests:
        for name in TEST_NAMES:
            source = ROOT / "tests" / (name + ".cpp")
            if source.is_file():
                test_targets.append((name, [source, ROOT / "src/reporting.cpp"], True))
            else:
                print("Skipping {}: {} is not present.".format(name, source), flush=True)
        if args.run_tests and not test_targets:
            raise RuntimeError("--run-tests requested, but no recognized C++ test sources exist.")
        targets.extend(test_targets)
    for name, sources, is_test in targets:
        for source in sources:
            if not source.is_file():
                raise RuntimeError("Missing source for {}: {}".format(name, source))

    check_toolchain(compiler, flags, build_dir, args.debug)
    if test_targets:
        check_toolchain(compiler, [*flags, "-UNDEBUG"], build_dir, True)
    test_executables = []
    suffix = ".exe" if os.name == "nt" else ""
    for name, sources, is_test in targets:
        executable = build_dir / (name + suffix)
        target_flags = [*flags, "-UNDEBUG"] if is_test else flags
        run([compiler, *target_flags, "-I", str(ROOT / "src"), *sources, "-o", executable], cwd=ROOT)
        print("Built " + str(executable), flush=True)
        if is_test:
            test_executables.append(executable)
    if args.run_tests:
        for executable in test_executables:
            run([executable], cwd=ROOT, timeout=120)
        print("Passed {} C++ test executable(s).".format(len(test_executables)), flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print("Build failed: " + str(error), file=sys.stderr)
        sys.exit(1)
