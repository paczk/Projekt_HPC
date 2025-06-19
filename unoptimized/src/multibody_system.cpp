#include "multibody_system.hpp"
#include <algorithm>
#include <memory>

MultibodySystem::MultibodySystem() = default;

void MultibodySystem::addBody(const Body& body)
{
    bodies.push_back(body);
    body_ids.push_back(body.getId());
}

void MultibodySystem::addConstraint(const Constraint& constraint) {
    constraints.push_back(constraint.clone());
}

int MultibodySystem::getNumBodies() const
{
    return static_cast<int>(bodies.size());
}

int MultibodySystem::getNumConstraints() const
{
    int total_constraints = 0;
    for (const auto& constraint : constraints)
    {
        total_constraints += constraint->equations_number();
    }
    return total_constraints;
}

const std::vector<Body>& MultibodySystem::getBodies() const
{
    return bodies;
}

const std::vector<long int>& MultibodySystem::getBodyIds() const
{
    return body_ids;
}

const std::vector<std::shared_ptr<Constraint>>& MultibodySystem::getConstraints() const
{
    return constraints;
}

const Eigen::VectorXd& MultibodySystem::getBodyParameters(long int id) const
{
    auto it = std::find(body_ids.begin(), body_ids.end(), id);
    if (it == body_ids.end())
        throw std::runtime_error("Body ID not found");
    int i = static_cast<int>(std::distance(body_ids.begin(), it));
    return bodies[i].getPosition();
}

// State implementation

State::State(const Eigen::VectorXd& q, double t) : q(q), t(t) {}

const Eigen::VectorXd& State::getQ() const
{
    return q;
}

double State::getTime() const
{
    return t;
}
