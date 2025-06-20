#include <vector>
#include <random>
#include "benchmark/benchmark.h"
#include "multibody_solver.hpp"

Eigen::Vector3d distance(double t)
                    {
                        
                        return Eigen::Vector3d(0.0, 0.0, cos(t));
                    };

void MultibodySolverBenchmark(benchmark::State& state)
{
    
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist;

    std::vector<MultibodySystem> systems;

    auto n_platforms= state.range(0);
    auto n_leg_parts = state.range(1);
    const auto max_threads = state.range(2);
    const auto guard = oneapi::tbb::global_control{
      oneapi::tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(max_threads)};

    const auto block_size = state.range(3);

    double platform_size_x = 1000.0;
    double platform_size_y = 1000.0;

    for(int i = 1; i <= n_platforms; i++)
    {
        MultibodySystem sys;

        Body platform{1, 0.0, 0.0, 0.5 * n_leg_parts, 1.0, 0.0, 0.0, 0.0};
        sys.addBody(platform);
        QuaternionConstraint orientation_constraint_pl{0, 1};
        sys.addConstraint(orientation_constraint_pl);

        for(long int j = 1; j <= 2; j++)
        {
            double x = dist(gen, std::uniform_real_distribution<>::param_type(0.0, platform_size_x));
            double y = dist(gen, std::uniform_real_distribution<>::param_type(0.0, platform_size_y));

            for(long int k = 1; k <= n_leg_parts; k++)
            {
                long int segment_id = j * 1'000'000 + k;

                if(k % 2 == 1) 
                {
                    Body leg_segment{segment_id, x, y + 0.5, (k-1) * 0.5, 0.9659, 0.2588, 0, 0};
                    sys.addBody(leg_segment);
                }
                else 
                {
                    Body leg_segment{segment_id, x, y + 0.5, (k-1) * 0.5, 0.2588, 0.9659, 0, 0};
                    sys.addBody(leg_segment);
                }
                

                //FixedParameterConstraint x_constant{segment_id, segment_id, 0};
                //sys.addConstraint(x_constant);

                QuaternionConstraint orientation_constraint{segment_id + 10'000'000, segment_id};
                sys.addConstraint(orientation_constraint);

                if(k == 1)
                {
                    Eigen::Vector3d ground_point(x,y,0.0);
                    Eigen::Vector3d body2_point(0.0, -0.5, 0.0);
                    RevoluteConstraint to_ground_constraint{segment_id + 3'000'000, 0, segment_id, ground_point, body2_point, 
                                                           Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0)};
                    sys.addConstraint(to_ground_constraint);
                }
                else
                {
                    Eigen::Vector3d body1_point(0.0, 0.5, 0.0);
                    Eigen::Vector3d body2_point(0.0, -0.5, 0.0);
                    RevoluteConstraint between_segments_constraint{segment_id + 3'000'000, segment_id - 1, segment_id, body1_point, body2_point, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0)};
                    sys.addConstraint(between_segments_constraint);

                    
                    Eigen::Vector3d point1(0.0, -0.5, 0.0);
                    Eigen::Vector3d point2(0.0, 0.5, 0.0);
                    DistanceConstraint distance_constraint{segment_id + 6'000'000, segment_id - 1, segment_id, point1, point2, distance};
                    sys.addConstraint(distance_constraint);
                }  
            }

            Eigen::Vector3d end_body_point(0.0, 0.5, 0.0);
            Eigen::Vector3d platform_point(x, y, 0.0);
            RevoluteConstraint platform_constraint{j, j * 1'000'000 + n_leg_parts, 1, end_body_point, platform_point, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0)};
            sys.addConstraint(platform_constraint);
        }

        systems.push_back(sys);
        if(i == 1)
        {
            platform_size_x = platform_size_x / n_platforms;
            platform_size_y = platform_size_y / n_platforms;
        }
    }

    for(auto _ : state)
    {
        for(auto& system : systems)
        {
            auto output = multibody_solver(system, 0.0, block_size);
        }
    }
    state.SetItemsProcessed(state.iterations() * n_platforms * n_leg_parts);

    
}

BENCHMARK(MultibodySolverBenchmark)->Unit(benchmark::kSecond)
    ->ArgsProduct
    ({
        {4, 8, 16, 24, 48},  // Number of platforms in total
        {benchmark::CreateRange(2, 128, 2)}, // Number of legs' parts
        {4, 8, 16, 24}, // Number of threads
        {7, 14, 28, 56, 70} // Block size for Jacobian
    })
    ->UseRealTime()->MeasureProcessCPUTime()->Name("Par Jacobian + right-hand side (#platforms, #leg parts, #threads, block size)");


BENCHMARK_MAIN();