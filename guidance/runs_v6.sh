#!/bin/sh
set -e
PY=../.venv/bin/python
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-20}
$PY -m ksp_guidance.mc --designs 400 --per 8 --landing baseline --seed 2026 --out runs/v6_baseline.npz
$PY -m ksp_guidance.mc --designs 400 --per 8 --landing baseline --seed 2026 --model-errors --out runs/v6_baseline_model_errors.npz
echo DONE
