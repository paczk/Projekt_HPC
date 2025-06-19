#include "benchmark/benchmark.h"
#include "multibody_solver.hpp"

void Unoptimized(benchmark::State& state)
{
    MultibodySystem sys;

    const auto n = state.range(0);

    for(int i = 0; i < n; ++i)
    {
        Body body1(i + 1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        
        sys.addBody(body1);
    }
    
    for(int i = 0; i < n; ++i)
    {
        FixedPositionConstraint constraint1(i + 1, i + 1, Eigen::Vector3d(0.0, 0.0, 0.0));
        FixedOrientationConstraint constraint3(i + n, i + 1, Eigen::Vector3d(0.0, 0.0, 1.0));

        sys.addConstraint(constraint1);
        sys.addConstraint(constraint3);
    }

    for(auto _ : state)
    {
        auto output = multibody_solver(sys, 0.0);
    }

    
}

BENCHMARK(Unoptimized)->RangeMultiplier(2)->Range(2, 2 << 16)->Unit(benchmark::kSecond);

BENCHMARK_MAIN();