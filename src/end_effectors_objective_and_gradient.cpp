#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>
#include <algorithm>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd position = transformed_tips(copy, b);
    double ls = pow((position - xb0).norm(), 2);
    return ls;
  };

  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Eigen::MatrixXd J;
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd position = transformed_tips(copy, b);


    kinematics_jacobian(copy,b,J);
    Eigen::VectorXd df;
    df = J.transpose() * (2 * (position - xb0));

    return df;
  };

  proj_z = [&](Eigen::VectorXd & A)
  {
    assert(skeleton.size()*3 == A.size());
    for (int i=0;i<skeleton.size();i++) {
      Bone b = skeleton[i];
      for (int j=0; j<3; j++) {
        A(i*3 + j) = std::max(b.xzx_min(j), std::min(b.xzx_max(j), A(i*3 + j)));
      }
    }
  };
}
