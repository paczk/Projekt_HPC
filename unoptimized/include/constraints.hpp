#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "bodies.hpp"
#include <memory>
#include <iostream>

// Stała pozycja/rotacja dla ciała typu ground
extern const Eigen::Matrix<double, 7, 1> ground;

// Funkcje pomocnicze do mapowania stanu układu
Eigen::Map<const Eigen::VectorXd> get_body_position(const Eigen::VectorXd& q, long int id, const std::vector<long int>& body_ids);
Eigen::Map<const Eigen::VectorXd> get_body_rotation(const Eigen::VectorXd& q, long int id, const std::vector<long int>& body_ids);

// Klasa bazowa dla ograniczeń
class Constraint
{
public:
    Constraint(long int id, long int body1_id, long int body2_id);

    virtual std::shared_ptr<Constraint> clone() const = 0;

    virtual Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids);
    virtual double equations_number();

protected:
    long int id;
    long int body1_id;
    long int body2_id;
};

// Ograniczenie odległości między punktami na ciałach
class DistanceConstraint : public Constraint
{
public:
    DistanceConstraint(long int id, long int body1_id, long int body2_id, const Eigen::Vector3d& body1_point,
                       const Eigen::Vector3d& body2_point,
                       const Eigen::Vector3d (*distance)(double t));

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d body1_point;
    const Eigen::Vector3d body2_point;
    const Eigen::Vector3d (*distance)(double t);
};

// Ograniczenie na konkretny parametr pozycji lub orientacji
class FixedParameterConstraint : public Constraint
{
public:
    FixedParameterConstraint(long int id, long int body_id, int parameter_index);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids) override;
    double equations_number() override;

private:
    int parameter_index;
};

// Ograniczenie orientacji względem ustalonego wektora
class FixedOrientationConstraint : public Constraint
{
public:
    FixedOrientationConstraint(long int id, long int body_id, const Eigen::Vector4d& orientation);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector4d orientation;
};

// Ograniczenie pozycji względem stałego punktu
class FixedPositionConstraint : public Constraint
{
public:
    FixedPositionConstraint(long int id, long int body_id, const Eigen::Vector3d& position);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d position;
};

// Ograniczenie kuliste – wspólny punkt w przestrzeni
class BallJointConstraint : public Constraint
{
public:
    BallJointConstraint(long int id, long int body1_id, long int body2_id, const Eigen::Vector3d& body1_point,
                        const Eigen::Vector3d& body2_point);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<long int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d body1_point;
    const Eigen::Vector3d body2_point;
};

#endif // CONSTRAINTS_HPP
