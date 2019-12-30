#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > T;
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(3*b.size());
  forward_kinematics(skeleton, T);

  for (int i=0; i<b.size(); i++) {
    double l = skeleton[b[i]].length;
    Eigen::Affine3d rest_pose_trans = skeleton[b[i]].rest_T;
    Eigen::Vector4d canonical_tail = {l,0,0,1};
    Eigen::Vector4d curr_tail = T[b[i]] * rest_pose_trans * canonical_tail;
    positions(i * 3 + 0) = curr_tail(0);
    positions(i * 3 + 1) = curr_tail(1);
    positions(i * 3 + 2) = curr_tail(2);
  }
  return positions;
}


