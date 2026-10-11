#!/usr/bin/env python3
"""Headless AdvantageKit replay: runs the CURRENT robot code against a recorded log's inputs and
writes <log>_sim.wpilog. Then `analyze.py compare <log>_sim.wpilog` shows what the change did.

    python replay.py logs/akit_26-10-03_18-00-00.wpilog            # build, then replay
    python replay.py LOG --out C:/tmp/after_fix.wpilog --no-build

Why not the Sim GUI or `gradlew simulateJava`: AdvantageKit refuses to replay while any HAL sim
extension (the GUI, DS socket) is loaded. This runs the built jar directly with none loaded, using
the env vars LoggingStartup reads (AKIT_LOG_PATH, ROBOT_REPLAY_OUTPUT).
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]


def find_jdk() -> Path:
    """The WPILib JDK for this project's year. System Java is often too new for Gradle 8.11."""
    year = "2026"
    m = re.search(r'edu\.wpi\.first\.GradleRIO"\s*version\s*"(\d{4})', (REPO / "build.gradle").read_text())
    if m:
        year = m.group(1)
    candidates = []
    if platform.system() == "Windows":
        candidates.append(Path(os.environ.get("PUBLIC", r"C:\Users\Public")) / "wpilib" / year / "jdk")
    else:
        candidates.append(Path.home() / "wpilib" / year / "jdk")
    if os.environ.get("JAVA_HOME"):
        candidates.append(Path(os.environ["JAVA_HOME"]))
    for c in candidates:
        if (c / "bin").is_dir():
            return c
    sys.exit(f"No JDK found. Install WPILib {year} or set JAVA_HOME to its jdk folder.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="recorded .wpilog to replay (from the robot or a sim run)")
    ap.add_argument("--out", help="output log (default: <log>_sim.wpilog next to the input)")
    ap.add_argument("--no-build", action="store_true", help="reuse the existing jar")
    ap.add_argument("--timeout", type=float, default=600, help="seconds before giving up")
    args = ap.parse_args()

    log = Path(args.log).resolve()
    if not log.is_file():
        sys.exit(f"Log not found: {log}")
    out = Path(args.out).resolve() if args.out else log.with_name(log.stem + "_sim.wpilog")
    jdk = find_jdk()
    env = dict(os.environ, JAVA_HOME=str(jdk))
    jni = REPO / "build" / "jni" / "release"

    if not args.no_build:
        gradlew = REPO / ("gradlew.bat" if platform.system() == "Windows" else "gradlew")
        print(f"Building with {jdk} ...", flush=True)
        r = subprocess.run([str(gradlew), "jar", "extractReleaseNative", "-q"], cwd=REPO, env=env,
                           shell=platform.system() == "Windows")
        if r.returncode != 0:
            sys.exit("Build failed; fix compile errors first (see output above).")

    jars = sorted((REPO / "build" / "libs").glob("*.jar"))
    if not jars or not jni.is_dir():
        sys.exit("No built jar/natives. Run without --no-build.")

    env.pop("HALSIM_EXTENSIONS", None)
    env["AKIT_LOG_PATH"] = str(log)
    env["ROBOT_REPLAY_OUTPUT"] = str(out)
    env["PATH"] = str(jni) + os.pathsep + env.get("PATH", "")
    if platform.system() == "Darwin":
        env["DYLD_LIBRARY_PATH"] = str(jni)
    elif platform.system() == "Linux":
        env["LD_LIBRARY_PATH"] = str(jni)
    if out.exists():
        out.unlink()

    java = jdk / "bin" / ("java.exe" if platform.system() == "Windows" else "java")
    print(f"Replaying {log.name} ...", flush=True)
    try:
        r = subprocess.run([str(java), f"-Djava.library.path={jni}", "-jar", str(jars[0])],
                           cwd=REPO, env=env, capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        sys.exit(f"Replay did not finish within {args.timeout:.0f}s.")
    text = (r.stdout or "") + (r.stderr or "")
    problems = [ln for ln in text.splitlines()
                if ("Error" in ln or "Exception" in ln) and "PathPlanner config" not in ln]
    if not out.exists() or out.stat().st_size < 1000:
        print(text[-3000:])
        sys.exit("Replay produced no output log (see robot output above).")
    for ln in problems[:20]:
        print("robot:", ln)
    print(f"Replay written to {out}")
    print(f"Next: python {Path(__file__).with_name('analyze.py')} compare \"{out}\"")


if __name__ == "__main__":
    main()
