#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "bodies.hpp"
#include <memory>

// Stała pozycja/rotacja dla ciała typu ground
extern const Eigen::Matrix<double, 7, 1> ground;

// Funkcje pomocnicze do mapowania stanu układu
Eigen::Map<const Eigen::VectorXd> get_body_position(const Eigen::VectorXd& q, int id, const std::vector<int>& body_ids);
Eigen::Map<const Eigen::VectorXd> get_body_rotation(const Eigen::VectorXd& q, int id, const std::vector<int>& body_ids);

// Klasa bazowa dla ograniczeń
class Constraint
{
public:
    Constraint(int id, int body1_id, int body2_id);

    virtual std::shared_ptr<Constraint> clone() const = 0;

    virtual Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids);
    virtual double equations_number();

protected:
    int id;
    int body1_id;
    int body2_id;
};

// Ograniczenie odległości między punktami na ciałach
class DistanceConstraint : public Constraint
{
public:
    DistanceConstraint(int id, int body1_id, int body2_id, const Eigen::Vector3d& body1_point,
                       const Eigen::Vector3d& body2_point,
                       const Eigen::Vector3d& (*distance)(double t));

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d& body1_point;
    const Eigen::Vector3d& body2_point;
    const Eigen::Vector3d& (*distance)(double t);
};

// Ograniczenie na konkretny parametr pozycji lub orientacji
class FixedParameterConstraint : public Constraint
{
public:
    FixedParameterConstraint(int id, int body_id, int parameter_index);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override;
    double equations_number() override;

private:
    int parameter_index;
};

// Ograniczenie orientacji względem ustalonego wektora
class FixedOrientationConstraint : public Constraint
{
public:
    FixedOrientationConstraint(int id, int body_id, const Eigen::Vector3d& orientation);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d& orientation;
};

// Ograniczenie pozycji względem stałego punktu
class FixedPositionConstraint : public Constraint
{
public:
    FixedPositionConstraint(int id, int body_id, const Eigen::Vector3d& position);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d& position;
};

// Ograniczenie kuliste – wspólny punkt w przestrzeni
class BallJointConstraint : public Constraint
{
public:
    BallJointConstraint(int id, int body1_id, int body2_id, const Eigen::Vector3d& body1_point,
                        const Eigen::Vector3d& body2_point);

    std::shared_ptr<Constraint> clone() const override;

    Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override;
    double equations_number() override;

private:
    const Eigen::Vector3d& body1_point;
    const Eigen::Vector3d& body2_point;
};

#endif // CONSTRAINTS_HPP
