#!/bin/sh
# Full Monte Carlo: same seed => same boosters and dispersions for each landing law (paired).
set -e
PY=../.venv/bin/python
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-20}
$PY -m ksp_guidance.mc --designs 400 --per 8 --landing baseline --seed 2026 --out runs/baseline.npz
$PY -m ksp_guidance.mc --designs 400 --per 8 --landing gfold --seed 2026 --out runs/gfold.npz
$PY -m ksp_guidance.mc --designs 400 --per 8 --landing baseline --seed 2026 --model-errors --out runs/baseline_model_errors.npz
