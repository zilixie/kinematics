#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  double cur_step = max_step;
  Eigen::VectorXd new_z = z;
  new_z -= cur_step * dz;
  proj_z(new_z);

  if (Eigen::VectorXd::Zero(dz.size()) == dz) {
    return 0;
  }

  //printf("f(new_z)=%f  f(z)=%f\n", f(new_z),f(z));

  while(f(new_z) > f(z)) {

    cur_step = 0.5 * cur_step;
    new_z = z;
    new_z -= cur_step * dz;
    proj_z(new_z);
  }


  return cur_step;
}
