#ifndef QUATERNION_OPERATIONS
#define QUATERNION_OPERATIONS

#include <eigen3/Eigen/Dense>

Eigen::MatrixXd L(Eigen::Map<const Eigen::VectorXd> e);

Eigen::MatrixXd E(Eigen::Map<const Eigen::VectorXd> e);

Eigen::Matrix3d R(Eigen::Map<const Eigen::VectorXd> e);

#endif