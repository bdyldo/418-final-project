#!/bin/bash
#SBATCH -p RM
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH -o outputs/slurm_%j.out

set -euo pipefail

cd /jet/home/jpark45/15418/418-final-project

mkdir -p outputs benchmark_results benchmark_svgs_seq

TARGETS=(
  svg_few_8_seq
  svg_few_16_seq
  svg_medium_16_seq
  svg_medium_32_seq
  svg_abundant_64_seq
  svg_abundant_128_seq
)

echo "Building sequential benchmark target..."
rm -f benchmark_runner_seq grid_demo correctness_tests benchmark_case_tool
make benchmark_runner_seq

echo "Running sequential benchmark targets:"
printf '  %s\n' "${TARGETS[@]}"

for target in "${TARGETS[@]}"; do
  echo
  echo "=== make ${target} ==="
  make "${target}"
done
