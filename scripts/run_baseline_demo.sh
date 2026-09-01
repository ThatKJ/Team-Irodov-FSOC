#!/usr/bin/env bash
#
# Reproducible SIH26169 FSOC baseline demo (v1_baseline, frozen).
#
# Builds if needed, runs the Step-10 baseline acceptance suite, runs the static
# and sinusoidal demo scenarios, regenerates the Step-9 annotated-frame
# evidence, and prints where every artifact landed. Nothing here is destructive
# and no machine-specific paths are hardcoded.
#
#   ./scripts/run_baseline_demo.sh          # build only if the binaries are missing
#   FSOC_DEMO_REBUILD=1 ./scripts/run_baseline_demo.sh   # force a reconfigure + build

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

build_dir="build/debug"
rebuild="${FSOC_DEMO_REBUILD:-0}"

if [[ "${rebuild}" == "1" || ! -x "${build_dir}/step10_validation_smoke" || ! -x "${build_dir}/fsoc_demo" ]]; then
    echo "== configure + build (Ninja, Debug) =="
    cmake --preset debug
    cmake --build --preset debug
else
    echo "== using existing build in ${build_dir} (set FSOC_DEMO_REBUILD=1 to force) =="
fi

mkdir -p generated/demo

echo
echo "==================================================================="
echo " Step-10 baseline acceptance  (must end: STEP 10 BASELINE ACCEPTANCE: PASS)"
echo "==================================================================="
"${build_dir}/step10_validation_smoke"

echo
echo "==================================================================="
echo " Demo scenario: static  (initial coarse alignment)"
echo "==================================================================="
"${build_dir}/fsoc_demo" static --csv generated/demo/static_demo.csv

echo
echo "==================================================================="
echo " Demo scenario: sinusoidal  (closed-loop tracking of a moving target)"
echo "==================================================================="
"${build_dir}/fsoc_demo" sinusoidal --csv generated/demo/sinusoidal_demo.csv

echo
echo "==================================================================="
echo " Step-9 engineering visualization evidence (annotated PNG frames)"
echo "==================================================================="
"${build_dir}/step9_visualization_smoke"

echo
echo "==================================================================="
echo " Artifacts"
echo "==================================================================="
echo "  generated/step10/       baseline acceptance report + per-scenario CSV + PNG"
echo "  generated/demo/         demo-runner telemetry CSV exports (static, sinusoidal)"
echo "  generated/step9/        annotated camera-view PNG frames (+ optional .mp4)"
echo
echo "Done. The v1_baseline engine is unchanged; this script only packages it."
