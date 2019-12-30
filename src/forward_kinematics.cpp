#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());

  //index to the bone
  std::vector<int> index;
  index.resize(T.size());
  for (int i = 0; i<index.size(); i++) {
    index[i] = -2;
  }

  for (int i=0; i<T.size(); i++) {
    for (int j=0; j<skeleton.size(); j++) {
      Eigen::Matrix4d rest_trans = skeleton[j].rest_T.matrix();
      Eigen::Matrix4d curr_trans = euler_angles_to_transform(skeleton[j].xzx).matrix();
      Eigen::Matrix4d inverse_rest_trans = rest_trans.inverse();
      Eigen::Affine3d t;

      if (skeleton[j].parent_index == -1 && index[j] == -2) {
        t.matrix() = Eigen::Affine3d::Identity().matrix() * rest_trans * curr_trans * inverse_rest_trans;
        T[j] = t;
        index[j] = 2;
        break;
      }
      if (index[skeleton[j].parent_index] == 2 && index[j] == -2) {
        Eigen::Affine3d p_rest_trans = T[skeleton[j].parent_index];
        t.matrix() = p_rest_trans.matrix() * rest_trans * curr_trans * inverse_rest_trans;
        T[j] = t;
        index[j] = 2;
        break;
      }
    }
  }
}
