#ifndef BODIES_HPP
#define BODIES_HPP

#include <eigen3/Eigen/Dense>

class Body
{
public:
    Body(int id, double x, double y, double z, double e0, double e1, double e2, double e3);

    void setPosition(const Eigen::VectorXd& position);

    const Eigen::VectorXd& getPosition();

    int getId();

private:
    int id;
    Eigen::VectorXd q;
};

#endif // BODIES_HPP
