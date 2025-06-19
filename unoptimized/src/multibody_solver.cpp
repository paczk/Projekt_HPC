#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>

#include<multibody_system.hpp>
#include<multibody_solver.hpp>

Eigen::MatrixXd multibody_jacobian(MultibodySystem mbs, State state)
{
    auto constraints_number = mbs.getNumConstraints();
    Eigen::MatrixXd J(constraints_number, mbs.getNumBodies() * 7);
    J.setZero();
    Eigen::VectorXd q = state.getQ();
    auto t = state.getTime();
    std::vector<long int> body_ids = mbs.getBodyIds();
    auto constraints = mbs.getConstraints();

    for(int i = 0; i < q.size(); i++)
    {
        Eigen::MatrixXd q_h = q;
        q_h(i) += 1;
        int constraint_index = 0;
        for(auto& constraint : constraints)
        {
            Eigen::VectorXd partial_jacobi = (constraint->ConstrainingFunctions(q_h, t, body_ids) - constraint->ConstrainingFunctions(q, t, body_ids))/ 1.0;
            J.col(i).segment(constraint_index, constraint->equations_number()) = partial_jacobi;
            constraint_index += constraint->equations_number();
        }
    }

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
    double norm = 1;
    int iter = 0;
    do {
        int i = 0;
        for(auto& con : constraints)
        {
            b.segment(i, con->equations_number()) = -con->ConstrainingFunctions(new_q, t, body_ids);
            
            i += con->equations_number();
        }
        //std::cout << b.transpose() << "\n";
        Eigen::MatrixXd J = multibody_jacobian(mbs, state);   
        

        std::cout << "calculated Jacobian\n";
        
        Eigen::VectorXd delta_q = J.householderQr().solve(b);

        std::cout << delta_q.transpose() << "\n";
        
        std::cout << "Solved linear problem\n";
        
        new_q += delta_q;

        i = 0;
        for(auto& con : constraints)
        {
            //b.segment(i, con->equations_number()) = -con->ConstrainingFunctions(new_q, t, body_ids);
            i += con->equations_number();
        }

        norm = b.dot(b);
        iter++;
        std::cout << norm << ". newton iteration done\n";
        
    } while(norm > 1e-12);
    
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