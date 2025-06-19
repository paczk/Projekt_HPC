#include "constraints.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include "bodies.hpp"
#include "quaternion_operations.hpp"
#include <memory>
#include <iostream>

const Eigen::Matrix<double, 7, 1> ground {0, 0, 0, 1, 0, 0, 0};

Eigen::Map<const Eigen::VectorXd> get_body_position(const Eigen::VectorXd& q, long int id, const std::vector<long int>& body_ids)
{
    if(id == 0)
    {
        return Eigen::Map<const Eigen::VectorXd>(ground.data(), 3);
    }
    else
    {
        auto it = std::find(body_ids.begin(), body_ids.end(), id);
        int i = std::distance(body_ids.begin(), it);
        return Eigen::Map<const Eigen::VectorXd>(q.data() + i * 7, 3);
    }
}

Eigen::Map<const Eigen::VectorXd> get_body_rotation(const Eigen::VectorXd& q, long int id, const std::vector<long int>& body_ids)
{
    if(id == 0)
    {
        return Eigen::Map<const Eigen::VectorXd>(ground.data() + 3, 4);
    }
    else
    {
        auto it = std::find(body_ids.begin(), body_ids.end(), id);
        int i = std::distance(body_ids.begin(), it);
        return Eigen::Map<const Eigen::VectorXd>(q.data() + i * 7 + 3, 4);
    }
}

// Constraint base class
Constraint::Constraint(long int id, long int body1_id, long int body2_id)
    : id(id), body1_id(body1_id), body2_id(body2_id) {}

Eigen::VectorXd Constraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    return Eigen::VectorXd();
}

double Constraint::equations_number()
{
    return 0;
}

// DistanceConstraint
DistanceConstraint::DistanceConstraint(long int id, long int body1_id, long int body2_id, const Eigen::Vector3d& body1_point, 
                                       const Eigen::Vector3d& body2_point, 
                                       const Eigen::Vector3d (*distance)(double t))
    : Constraint(id, body1_id, body2_id), body1_point(body1_point), body2_point(body2_point), distance(distance) {}

std::shared_ptr<Constraint> DistanceConstraint::clone() const {
    return std::make_shared<DistanceConstraint>(*this);
}

Eigen::VectorXd DistanceConstraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    auto e1 = get_body_rotation(q, body1_id, body_ids);
    auto e2 = get_body_rotation(q, body2_id, body_ids);
    auto r1 = get_body_position(q, body1_id, body_ids);
    auto r2 = get_body_position(q, body2_id, body_ids);

    Eigen::Vector3d functions;

    Eigen::Vector3d dist;
    dist = distance(t);

    functions = r2 + R(e2) * body2_point - (r1 + R(e1) * body1_point) - dist;
    return functions;
}

double DistanceConstraint::equations_number()
{
    return 3;
}

// FixedParameterConstraint
FixedParameterConstraint::FixedParameterConstraint(long int id, long int body_id, int parameter_index)
    : Constraint(id, body_id, 0), parameter_index(parameter_index) {}

std::shared_ptr<Constraint> FixedParameterConstraint::clone() const {
    return std::make_shared<FixedParameterConstraint>(*this);
}

Eigen::VectorXd FixedParameterConstraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    Eigen::VectorXd functions(1);
    auto e = get_body_rotation(q, body1_id, body_ids);
    auto r = get_body_position(q, body1_id, body_ids);

    if (parameter_index < 3) {
        functions(0) = r(parameter_index);
    } else {
        functions(0) = e(parameter_index - 3);
    }
    return functions;
}

double FixedParameterConstraint::equations_number()
{
    return 1;
}

// FixedOrientationConstraint
FixedOrientationConstraint::FixedOrientationConstraint(long int id, long int body_id, const Eigen::Vector4d& orientation)
    : Constraint(id, body_id, 0), orientation(orientation) {}

std::shared_ptr<Constraint> FixedOrientationConstraint::clone() const {
    return std::make_shared<FixedOrientationConstraint>(*this);
}

Eigen::VectorXd FixedOrientationConstraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    Eigen::VectorXd functions(4);
    auto e = get_body_rotation(q, body1_id, body_ids);
    functions = e - orientation;
    return functions;
}

double FixedOrientationConstraint::equations_number()
{
    return 4;
}

// FixedPositionConstraint
FixedPositionConstraint::FixedPositionConstraint(long int id, long int body_id, const Eigen::Vector3d& position)
    : Constraint(id, body_id, 0), position(position) {}

std::shared_ptr<Constraint> FixedPositionConstraint::clone() const {
    return std::make_shared<FixedPositionConstraint>(*this);
}

Eigen::VectorXd FixedPositionConstraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    Eigen::VectorXd functions(3);
    auto r = get_body_position(q, body1_id, body_ids);
    functions = r - position;
    return functions;
}

double FixedPositionConstraint::equations_number()
{
    return 3;
}

// BallJointConstraint
BallJointConstraint::BallJointConstraint(long int id, long int body1_id, long int body2_id, const Eigen::Vector3d& body1_point, 
                                         const Eigen::Vector3d& body2_point)
    : Constraint(id, body1_id, body2_id), body1_point(body1_point), body2_point(body2_point) {}

std::shared_ptr<Constraint> BallJointConstraint::clone() const {
    return std::make_shared<BallJointConstraint>(*this);
}

Eigen::VectorXd BallJointConstraint::ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids)
{
    auto e1 = get_body_rotation(q, body1_id, body_ids);
    auto e2 = get_body_rotation(q, body2_id, body_ids);
    auto r1 = get_body_position(q, body1_id, body_ids);
    auto r2 = get_body_position(q, body2_id, body_ids);

    Eigen::VectorXd functions(3);
    functions = (r2 + R(e2) * body2_point) - (r1 + R(e1) * body1_point);
    return functions;
}

double BallJointConstraint::equations_number()
{
    return 3;
}
