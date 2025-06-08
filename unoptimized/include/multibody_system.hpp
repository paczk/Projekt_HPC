#ifndef MULTIBODY_SYSTEM
#define MULTIBODY_SYSTEM

#include <vector>
#include "bodies.hpp"
#include "constraints.hpp"
#include <memory>

class MultibodySystem
{
    public:
        MultibodySystem();

        void addBody(Body& body);

        void addConstraint(const Constraint& constraint);

        int getNumBodies();

        int getNumConstraints();

        const std::vector<Body>& getBodies() const;
        const std::vector<int>& getBodyIds() const;
        const std::vector<std::shared_ptr<Constraint>>& getConstraints() const;
    
    private:
        std::vector<Body> bodies;
        std::vector<int> body_ids;
        std::vector<std::shared_ptr<Constraint>> constraints;
};

class State
{
    public:
        State(const Eigen::VectorXd& q, double t);

        const Eigen::VectorXd& getQ() const;

        double getTime() const;
        
    private:
        Eigen::VectorXd q; // positions and orientations of bodies
        double t; // time
};

#endif