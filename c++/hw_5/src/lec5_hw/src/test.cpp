#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
Vector2d v;
v(0)=1;
v(1)=2;
for (int i=0; i<=1;i++)
{
MatrixXd m(3,3);
m(0,0) = 3;
m(1,0) = 2.5;
m(2,0) = 2.5;

MatrixXd n(3,2);
m.block<3,2>(0,1)= MatrixXd::Zero(3,2);
std::cout << "Here is the matrix m:\n" << m << std::endl;
}

// std:: v(2);
// v(0) = m;
// v(1) = m;
//std::cout << "Here is the vector v:\n" << v << std::endl;
}
