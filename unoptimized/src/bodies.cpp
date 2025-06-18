#include "bodies.hpp"
#include <cmath>
#include <Eigen/Dense>

Body::Body(int id, double x, double y, double z, double e0, double e1, double e2, double e3)
    : id(id), q(7)
{
    double norm = std::sqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3);
    e0 /= norm;
    e1 /= norm;
    e2 /= norm;
    e3 /= norm;
    q << x, y, z, e0, e1, e2, e3;
}

void Body::setPosition(const Eigen::VectorXd& position)
{
    q = position;
}

const Eigen::VectorXd& Body::getPosition() const
{
    return q;
}

int Body::getId() const
{
    return id;
}


