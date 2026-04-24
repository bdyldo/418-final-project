#!/bin/bash
#SBATCH -p RM
#SBATCH -t 02:00:00
#SBATCH -N 1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=128
#SBATCH -o outputs/slurm_%j.out

set -euo pipefail

cd /jet/home/jpark45/15418/418-final-project

mkdir -p outputs benchmark_results

export OMP_PROC_BIND=true
export OMP_PLACES=cores

BEAM_WIDTH=8
THREAD_COUNTS=(16 32 64 128)
RESULT_FILE="benchmark_results/small_64_parallel_beam${BEAM_WIDTH}_scaling_${SLURM_JOB_ID}.txt"

echo "Building benchmark target..."
rm -f benchmark_runner grid_demo correctness_tests benchmark_case_tool
make benchmark_runner

{
  echo "small_64 parallel scaling run"
  echo "planner=parallel-speculative"
  echo "beam_width=${BEAM_WIDTH}"
  echo "job_id=${SLURM_JOB_ID}"
  echo "host=$(hostname)"
  echo "cpus_per_task=${SLURM_CPUS_PER_TASK:-128}"
  echo "thread_counts=${THREAD_COUNTS[*]}"
  echo
} > "${RESULT_FILE}"

for n in "${THREAD_COUNTS[@]}"; do
  export OMP_NUM_THREADS="${n}"
  {
    echo "=== small_64 parallel with N=${n} ==="
    ./benchmark_runner small_64 --planner parallel --beam-width "${BEAM_WIDTH}" -N "${n}"
    echo
  } | tee -a "${RESULT_FILE}"
done

echo "Combined results written to ${RESULT_FILE}"
