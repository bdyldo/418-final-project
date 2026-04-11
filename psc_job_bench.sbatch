#!/bin/bash
#SBATCH -p RM
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=128
#SBATCH -o outputs/slurm_%j.out

set -euo pipefail

cd /jet/home/jpark45/15418/418-final-project

mkdir -p outputs benchmark_results benchmark_svgs

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-128}"
export OMP_PROC_BIND=true
export OMP_PLACES=cores

TARGETS="${TARGETS:-svg_few_8}"
N_VALUE="${N:-${SLURM_CPUS_PER_TASK:-128}}"

echo "Building benchmark target..."
rm -f benchmark_runner grid_demo correctness_tests benchmark_case_tool
make benchmark_runner

echo "Using N=${N_VALUE}"
echo "Running make targets: ${TARGETS}"

for target in ${TARGETS}; do
  echo
  echo "=== make ${target} N=${N_VALUE} ==="
  make "${target}" N="${N_VALUE}"
done
