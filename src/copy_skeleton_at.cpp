#include "copy_skeleton_at.h"
#include <iostream>

Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  Skeleton copy = skeleton;
  for (int i=0; i<skeleton.size(); i++) {
    for (int j=0;j<3;j++) {
      copy[i].xzx(j) = A(i*3+j);
    }
  }

  return copy;
}
