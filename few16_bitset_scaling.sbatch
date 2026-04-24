#!/bin/bash
#SBATCH -p RM
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=128
#SBATCH -o /jet/home/jpark45/15418/418-final-project/outputs/slurm_%j.out

set -euo pipefail

cd /jet/home/jpark45/15418/418-final-project

mkdir -p outputs benchmark_results

export OMP_PROC_BIND=true
export OMP_PLACES=cores

RESULT_FILE="benchmark_results/few_16_bitset_scaling_${SLURM_JOB_ID}.txt"
THREAD_COUNTS=(1 2 4 8 16 32 64 128)

echo "Building benchmark target..."
rm -f benchmark_runner grid_demo correctness_tests benchmark_case_tool
make benchmark_runner

{
  echo "few_16 bitset scaling run"
  echo "planner=bitset-wavefront-mapf"
  echo "result_label=few_16_bitset"
  echo "job_id=${SLURM_JOB_ID}"
  echo "host=$(hostname)"
  echo "cpus_per_task=${SLURM_CPUS_PER_TASK:-128}"
  echo "thread_counts=${THREAD_COUNTS[*]}"
  echo
} > "${RESULT_FILE}"

for n in "${THREAD_COUNTS[@]}"; do
  export OMP_NUM_THREADS="${n}"
  {
    echo "=== few_16 bitset with N=${n} ==="
    ./benchmark_runner few_16 --planner bitset -N "${n}"
    echo
  } | tee -a "${RESULT_FILE}"
done

echo "Combined results written to ${RESULT_FILE}"
