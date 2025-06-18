#include "bodies.hpp"

Body test_body1(1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
Body test_body2(2, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
Body test_body3(3, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
Body test_body4(4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Body test_body5(5, 1.0, 1.0, 1.0, 0.7071, 0.7071, 0.0, 0.0);
Body test_body6(6, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

int main()
{
Eigen::VectorXd test_position1(7);

test_position1 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

test_body2.setPosition(test_position1);

test_position1 = test_body3.getPosition();

auto id = test_body2.getId();


}; 