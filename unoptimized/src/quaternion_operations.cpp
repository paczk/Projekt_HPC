#include <eigen3/Eigen/Dense>
#include "quaternion_operations.hpp"

Eigen::MatrixXd L(Eigen::Map<const Eigen::VectorXd> e)
{
    Eigen::MatrixXd L(3, 4);
    
    L(1, 0) = -e(1);
    L(1, 1) = e(0);
    L(1, 2) = -e(3);
    L(1, 3) = e(2);
    
    L(2, 0) = -e(2);
    L(2, 1) = e(3);
    L(2, 2) = e(0);
    L(2, 3) = -e(1);
    
    L(3, 0) = -e(3);
    L(3, 1) = -e(2);
    L(3, 2) = e(1);
    L(3, 3) = e(0);

    return L;
}

Eigen::MatrixXd E(Eigen::Map<const Eigen::VectorXd> e)
{
    Eigen::MatrixXd L(3, 4);
    
    L(1, 0) = -e(1);
    L(1, 1) = e(0);
    L(1, 2) = e(3);
    L(1, 3) = -e(2);
    
    L(2, 0) = -e(2);
    L(2, 1) = -e(3);
    L(2, 2) = e(0);
    L(2, 3) = e(1);
    
    L(3, 0) = -e(3);
    L(3, 1) = e(2);
    L(3, 2) = -e(1);
    L(3, 3) = e(0);

    return L;
}

Eigen::Matrix3d R(Eigen::Map<const Eigen::VectorXd> e)
{
    Eigen::Matrix3d R;

    R = L(e).transpose() * R(e);
}