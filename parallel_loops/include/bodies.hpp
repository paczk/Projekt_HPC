#ifndef BODIES_HPP
#define BODIES_HPP

#include <cmath>
#include <Eigen/Dense>

class Body
{
public:
    Body(long int id, double x, double y, double z, double e0, double e1, double e2, double e3);

    void setPosition(const Eigen::VectorXd& position);

    const Eigen::VectorXd& getPosition() const;

    long int getId() const;

private:
    long int id;
    Eigen::VectorXd q;
};

#endif // BODIES_HPP
