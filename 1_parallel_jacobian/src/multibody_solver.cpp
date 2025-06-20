#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include <oneapi/tbb.h>

#include<multibody_system.hpp>
#include<multibody_solver.hpp>
#include <Eigen/Sparse>
using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

SparseMatrix multibody_jacobian(MultibodySystem mbs, State state, int block_size)
{
    const int constraints_number = mbs.getNumConstraints();
    const int cols = mbs.getNumBodies() * 7;

    std::vector<Triplet> triplets;

    const Eigen::VectorXd q = state.getQ();
    const double t = state.getTime();
    const std::vector<long int> body_ids = mbs.getBodyIds();
    const auto& constraints = mbs.getConstraints();

    std::vector<std::vector<Triplet>> local_triplets(q.size());

    const auto range = oneapi::tbb::blocked_range<Eigen::Index>{0, q.size(), 14};

    oneapi::tbb::parallel_for(range, [&](const auto& r)
    {
        for (Eigen::Index i = r.begin(); i != r.end(); ++i)
        {
            Eigen::VectorXd q_h = q;
            q_h(i) += 1e-4;

            int constraint_index = 0;
            for (const auto& constraint : constraints)
            {
                Eigen::VectorXd diff =
                    constraint->ConstrainingFunctions(q_h, t, body_ids)
                - constraint->ConstrainingFunctions(q,   t, body_ids);

                Eigen::VectorXd partial_jacobi = diff / 1e-4;
                int eq_num = constraint->equations_number();

                for (int row = 0; row < eq_num; ++row)
                {
                    local_triplets[i].emplace_back(constraint_index + row, i, partial_jacobi(row));
                }

                constraint_index += eq_num;
            }
        }
    });

    for (const auto& vec : local_triplets)
    triplets.insert(triplets.end(), vec.begin(), vec.end());

    SparseMatrix J(constraints_number, cols);
    J.setFromTriplets(triplets.begin(), triplets.end());

    return J;
}



State newton_solver(MultibodySystem mbs, State state)
{
    auto constraints_number = mbs.getNumConstraints();
    Eigen::VectorXd b(constraints_number);
    b.setZero();
    auto q = state.getQ();
    auto t = state.getTime();
    std::vector<long int> body_ids = mbs.getBodyIds();
    auto constraints = mbs.getConstraints();
    auto new_q = q;
    //std::cout<< new_q.transpose() << "\n";
    double norm;
    int iter = 0;
    int i = 0;
    for(auto& con : constraints)
    {
        b.segment(i, con->equations_number()) = -con->ConstrainingFunctions(new_q, t, body_ids);
        i += con->equations_number();
    }

    norm = b.dot(b);

    while(norm > 1e-12)
    {
        //std::cout << b.transpose() << "\n";
        Eigen::SparseMatrix<double> J = multibody_jacobian(mbs, state);   
        //std::cout << "calculated Jacobian\n";

        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
        solver.compute(J);

        if (solver.info() != Eigen::Success) {
            std::cerr << "Decomposition failed!\n";
        }

        Eigen::VectorXd delta_q = solver.solve(b);

        if (solver.info() != Eigen::Success) {
            std::cerr << "Solving failed!\n";
        }
        
        //std::cout << "Solved linear problem\n";
        
        new_q += delta_q;

        i = 0;
        for(auto& con : constraints)
        {
            b.segment(i, con->equations_number()) = -con->ConstrainingFunctions(new_q, t, body_ids);
            i += con->equations_number();
        }

        norm = b.dot(b);
        iter++;
        //std::cout << iter << ". newton iteration done\n";
        //std::cout << "Norm: " << norm << "\n";
        
        if(iter > 1000)
        {
            std::cerr << "Newton solver did not converge after 1000 iterations.\n";
            break;
        }

        if(norm > 1e20)
        {
            std::cerr << "Newton solver diverged, norm is too high: " << norm << "\n";
            break;
        }
    }
    
    return State{new_q, t};
}

std::vector<State> multibody_solver(MultibodySystem& mbs, double end_time)
{
    Eigen::VectorXd q(mbs.getNumBodies() * 7);
    auto body_ids = mbs.getBodyIds();
    
    for(int i = 0; i < mbs.getNumBodies(); i++)
    {
        auto part_q = mbs.getBodyParameters(body_ids[i]);
        q.segment(i*7,7) = part_q;
    }

    State state{q, 0};
    std::vector<State> states;
    for(double t = 0; t <= end_time; t +=0.1)
    {
        states.push_back(newton_solver(mbs,state));
    }

    return states;
}