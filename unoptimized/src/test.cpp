#include <vector>
#include <iostream>

#include "multibody_solver.hpp"

int main() 
{
    // Create a multibody solver instance
    MultibodySystem sys;

    // Define some bodies and joints
    Body body1(1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    Body body2(2, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    
    
    FixedPositionConstraint constraint1(1, 1, Eigen::Vector3d(0.0, 0.0, 0.0));
    FixedPositionConstraint constraint2(2, 2, Eigen::Vector3d(1.0, 0.0, 0.0));
    FixedOrientationConstraint constraint3(3, 1, Eigen::Vector3d(0.0, 0.0, 1.0));
    FixedOrientationConstraint constraint4(4, 2, Eigen::Vector3d(0.0, 1.0, 0.0));

    // Add bodies and joints to the solver
    sys.addBody(body1);
    sys.addBody(body2);
    sys.addConstraint(constraint1);
    sys.addConstraint(constraint2);
    sys.addConstraint(constraint3);
    sys.addConstraint(constraint4);

    // Solve the multibody system
    auto output = multibody_solver(sys, 0.0);
    // Output results
    std::cout << "Multibody system solved successfully!" << std::endl;

    return 0;
}