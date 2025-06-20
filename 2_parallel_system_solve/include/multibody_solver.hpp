#ifndef MULTIBODY_SOLVER_HPP
#define MULTIBODY_SOLVER_HPP

#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include <oneapi/tbb.h>

#include "multibody_system.hpp"
#include <Eigen/Sparse>
using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

SparseMatrix multibody_jacobian(MultibodySystem mbs, State state, int block_size = 7);

State newton_solver(MultibodySystem mbs, State state, int block_size = 7);

std::vector<State> multibody_solver(MultibodySystem& mbs, double end_time, int block_size = 7);

#endif