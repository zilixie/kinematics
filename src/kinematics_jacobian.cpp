#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>


void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  double h = 0.0000001;

  Skeleton skeleton_h = skeleton;

  for (int i=0;i<skeleton.size(); i++) {
    for (int j=0; j<3; j++) {
      skeleton_h[i].xzx(j) += h;
      Eigen::VectorXd position = transformed_tips(skeleton, b);
      Eigen::VectorXd positionh = transformed_tips(skeleton_h, b);

      Eigen::VectorXd a = positionh - position;
      Eigen::VectorXd Jij = (1.0/h) * (positionh - position);
      J.col(i*3 + j) = Jij;
      skeleton_h[i].xzx(j) -= h;
    }
  }
}
