#include "projected_gradient_descent.h"
#include "line_search.h"

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  int iter = 0;
  double sigma;
  sigma = line_search(f, proj_z, z, grad_f(z), 1000);
  //printf("sigma: %f\n", sigma);
  while (iter < max_iters) {
    z -= sigma * grad_f(z);
    proj_z(z);
    iter++;
  }
}
