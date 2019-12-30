#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  Eigen::Vector4d v = {0,0,0,0};
  for (int i=0; i<V.rows(); i++) {
    v = {V.row(i)(0), V.row(i)(1), V.row(i)(2), 1};
    Eigen::Vector4d sum_v = {0,0,0,0};
    for (int j=0; j<T.size(); j++) {
      if (skeleton[j].weight_index == -1) {
        double weight = 0;
        Eigen::Affine3d transform = T[j];
        sum_v = sum_v + weight * transform.matrix() * v;
      }
      if (skeleton[j].weight_index != -1) {
        double weight = W(i,skeleton[j].weight_index);
        Eigen::Affine3d transform = T[j];
        sum_v = sum_v + weight * transform.matrix() * v;
      }
    }
    U.row(i) = Eigen::Vector3d(sum_v(0), sum_v(1), sum_v(2));
  }
}
