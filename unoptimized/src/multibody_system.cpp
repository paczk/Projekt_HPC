#include <vector>
#include "bodies.hpp"
#include "constraints.hpp"
#include "multibody_system.hpp"
#include <memory>

class MultibodySystem
{
    public:
        MultibodySystem() = default;

        void addBody(Body& body) 
        {
            bodies.push_back(body);
            body_ids.push_back(body.getId());
        }

        void addConstraint(const Constraint& constraint) 
        {
            constraints.push_back(constraint);
        }

        int getNumBodies()
        {
            return bodies.size();
        }

        int getNumConstraints() 
        {
            int total_constraints = 0;
            for(auto& constraint : constraints)
            {
                total_constraints += constraint->equations_number();
            }
            return total_constraints;
        }

        const std::vector<Body>& getBodies() const 
        {
            return bodies;
        }

        const std::vector<int>& getBodyIds() const 
        {
            return body_ids;
        }

        const std::vector<std::shared_ptr<Constraint>>& getConstraints() const 
        {
            return constraints;
        }

        const Eigen::VectorXd& getBodyParameters(int id)
        {
            auto it = std::find(body_ids.begin(), body_ids.end(), id);
            int i = std::distance(body_ids.begin(), it);
            return bodies[i].getPosition();
        }
    
    private:
        std::vector<Body> bodies;
        std::vector<int> body_ids;
        std::vector<std::shared_ptr<Constraint>> constraints;
};

class State
{
    public:
        State(const Eigen::VectorXd& q, double t) : q(q), t(t) {}

        const Eigen::VectorXd& getQ() const {
            return q;
        }

        double getTime() const {
            return t;
        }
        
    private:
        Eigen::VectorXd q; // positions and orientations of bodies
        double t; // time
};