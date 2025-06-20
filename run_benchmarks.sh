#!/bin/bash

BENCHMARKS=(
  "0_unoptimized/build/benchmark"
  "1_parallel_jacobian/build/benchmark"
  "2_parallel_system_solve/build/benchmark"
)

OUTPUT_FILES=(
  "benchmark_0_unoptimized"
  "benchmark_1_parallel_jacobian"
  "benchmark_2_parallel_system_solve"
)

OUTPUT_DIR="."

BENCHMARK_OPTS="--benchmark_format=csv"

mkdir -p "$OUTPUT_DIR"

for i in "${!BENCHMARKS[@]}"; do
  echo "Uruchamiam benchmark: ${BENCHMARKS[i]}"
  "${BENCHMARKS[i]}" $BENCHMARK_OPTS > "${OUTPUT_DIR}/${OUTPUT_FILES[i]}.csv"
  echo "Wynik zapisano do: ${OUTPUT_DIR}/${OUTPUT_FILES[i]}.csv"
done

echo "Wszystkie benchmarki zostały wykonane."
