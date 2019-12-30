#include "euler_angles_to_transform.h"
#include <math.h>
#include <cmath>

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  const double pi = std::acos(-1);
  double c0 = std::cos(xzx(0)/180.0 * pi);
  double s0 = std::sin(xzx(0)/180.0 * pi);
  double c1 = std::cos(xzx(1)/180.0 * pi);
  double s1 = std::sin(xzx(1)/180.0 * pi);
  double c2 = std::cos(xzx(2)/180.0 * pi);
  double s2 = std::sin(xzx(2)/180.0 * pi);

  Eigen::Affine3d D;
  D.matrix() <<
    1,0,0,0,
    0,c0,-s0,0,
    0,s0,c0,0,
    0,0,0,1;
  Eigen::Affine3d C;
  C.matrix() <<
    c1,-s1,0,0,
    s1,c1,0,0,
    0,0,1,0,
    0,0,0,1;
  Eigen::Affine3d B;
  B.matrix() <<
    1,0,0,0,
    0,c2,-s2,0,
    0,s2,c2,0,
    0,0,0,1;
  Eigen::Affine3d A;
  A.matrix() << D.matrix()*C.matrix()*B.matrix();

  return A;
}
