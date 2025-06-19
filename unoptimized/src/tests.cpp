#include <vector>
#include <iostream>

#include "multibody_solver.hpp"

int main() 
{
    // Create a multibody solver instance
    MultibodySystem sys;

    // Define some bodies and joints
    //Body body1(1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    //Body body2(2, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    

    for(int i = 0; i < 2; ++i)
    {
        Body body1(i + 1, 0.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0);
        
        sys.addBody(body1);
    }
    
    //FixedPositionConstraint constraint1(1, 1, Eigen::Vector3d(0.0, 0.0, 0.0));
    //FixedPositionConstraint constraint2(2, 2, Eigen::Vector3d(1.0, 0.0, 0.0));
    //FixedOrientationConstraint constraint3(3, 1, Eigen::Vector3d(0.0, 0.0, 1.0));
    //FixedOrientationConstraint constraint4(4, 2, Eigen::Vector3d(0.0, 1.0, 0.0));

    for(int i = 0; i < 2; ++i)
    {
        Eigen::Vector3d p1(0.0,0.0,0.0);
        FixedPositionConstraint constraint1(i, i+1, Eigen::Vector3d(1.0, 10.0, 1.0));
        FixedOrientationConstraint constraint3(i+10, i+1, Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        //BallJointConstraint constraint3(i+10,0,i+1,Eigen::Vector3d(1.0,1.0,1.0),Eigen::Vector3d(1.0,1.0,1.0));
        sys.addConstraint(constraint1);
        sys.addConstraint(constraint3);
    }

    // Add bodies and joints to the solver
    //sys.addBody(body1);
    //sys.addBody(body2);
    //sys.addConstraint(constraint1);
    //sys.addConstraint(constraint2);
    //sys.addConstraint(constraint3);
    //sys.addConstraint(constraint4);

    // Solve the multibody system
    auto output = multibody_solver(sys, 0.0);
    auto qq = output[0].getQ();
    // Output results
    std::cout << "Multibody system solved successfully!" << qq(1) << std::endl;

    return 0;
}