#include <vector>
#include <iostream>
#include <random>

#include "multibody_solver.hpp"

int main() 
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist;

    std::vector<MultibodySystem> systems;

    auto n_platforms= 8;
    auto n_leg_parts = 10;

    double platform_size_x = 1000.0;
    double platform_size_y = 1000.0;

    for(int i = 1; i <= n_platforms; i++)
    {
        MultibodySystem sys;

        Body platform{1, 0.0, 0.0, 1.0 * n_leg_parts, 1.0, 0.0, 0.0, 0.0};
        sys.addBody(platform);

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
                

                FixedParameterConstraint x_constant{segment_id, segment_id, 0};
                sys.addConstraint(x_constant);

                
        }

        systems.push_back(sys);
        if(i == 1)
        {
            platform_size_x = platform_size_x / n_platforms;
            platform_size_y = platform_size_y / n_platforms;
        }
    }}


        for(auto& system : systems)
        {
            auto output = multibody_solver(system, 0.0);
        }
    std::cout << "Multibody system solved successfully!" << std::endl;

    return 0;
}