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
    auto q = state.getQ();
    auto t = state.getTime();
    std::vector<long int> body_ids = mbs.getBodyIds();
    auto constraints = mbs.getConstraints();

    for(int i = 0; i < q.size(); i++)
    {
        auto q_h = q;
        q_h(i) += 1e-4;
        int constraint_index = 0;
        for(auto& constraint : constraints)
        {
            auto partial_jacobi = (constraint->ConstrainingFunctions(q_h, t, body_ids) - constraint->ConstrainingFunctions(q, t, body_ids))/ 1e-4;
            J.col(i).segment(constraint_index, constraint->equations_number()) = partial_jacobi;
            constraint_index += constraint->equations_number();
        }
    }

    return J;
}

bool bicgstab_solver(const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& b,
                     Eigen::VectorXd& x,
                     int max_iter,
                     double tol)
{
    const int n = A.rows();
    Eigen::VectorXd r = b - A * x;
    Eigen::VectorXd r0 = r;
    Eigen::VectorXd p = r;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd s(n), t(n);

    double rho_old = 1.0, alpha = 1.0, omega = 1.0;
    double rho_new, beta;

    for (int iter = 0; iter < max_iter; ++iter) {
        rho_new = r0.dot(r);
        if (std::abs(rho_new) < 1e-15) return false;

        beta = (rho_new / rho_old) * (alpha / omega);

        for (int i = 0; i < n; ++i)
            p[i] = r[i] + beta * (p[i] - omega * v[i]);

        v = A * p;
        alpha = rho_new / r0.dot(v);

        for (int i = 0; i < n; ++i)
            s[i] = r[i] - alpha * v[i];

        if (s.norm() < tol) {
            x += alpha * p;
            return true;
        }

        t = A * s;
        omega = t.dot(s) / t.dot(t);
        if (std::abs(omega) < 1e-15) return false;
        
        for (int i = 0; i < n; ++i)
            x[i] += alpha * p[i] + omega * s[i];

        for (int i = 0; i < n; ++i)
            r[i] = s[i] - omega * t[i];

        if (r.norm() < tol) return true;

        rho_old = rho_new;
    }

    return false;
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
    double norm;
    int iter = 0;
    do {
        for(int i = 0; i < constraints_number; i++)
        {
            b.segment(i, constraints[i]->equations_number()) = -constraints[i]->ConstrainingFunctions(q, t, body_ids);
        }

        Eigen::MatrixXd J = multibody_jacobian(mbs, state);

        std::cout << "calculated Jacobian\n";
        
        Eigen::VectorXd delta_q(mbs.getNumBodies() * 7);
        
        if(!bicgstab_solver(J, b, delta_q))
            return State{new_q, t};
        
        std::cout << "Solved linear problem\n";
        
        new_q += delta_q;

        for(int i = 0; i < constraints_number; i++)
        {
            b.segment(i, constraints[i]->equations_number()) = constraints[i]->ConstrainingFunctions(q, t, body_ids);
        }

        norm = b.dot(b);
        iter++;
        std::cout << iter << ". newton iteration done\n";
    } while(norm > 1e-8);
    
    return State{new_q, t};
}

std::vector<State> multibody_solver(MultibodySystem mbs, double end_time)
{
    Eigen::VectorXd q(mbs.getNumBodies() * 7);
    auto body_ids = mbs.getBodyIds();
    for(int i = 0; i < mbs.getNumBodies(); i++)
    {
        auto part_q = mbs.getBodyParameters(body_ids[i]);
        q.segment(body_ids[i],7) = part_q;
    }
    State state{q, 0};
    std::vector<State> states;
    for(double t = 0; t < end_time; t +=0.1)
    {
        states.push_back(newton_solver(mbs,state));
    }

    return states;
}