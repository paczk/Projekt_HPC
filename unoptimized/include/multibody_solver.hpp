#ifndef MULTIBODY_SOLVER_HPP
#define MULTIBODY_SOLVER_HPP

#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>

#include "multibody_system.hpp"

Eigen::MatrixXd multibody_jacobian(MultibodySystem mbs, State state);

bool bicgstab_solver(const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& b,
                     Eigen::VectorXd& x,
                     int max_iter = 1000,
                     double tol = 1e-8);

State newton_solver(MultibodySystem mbs, State state);

std::vector<State> multibody_solver(MultibodySystem& mbs, double end_time);

#endif