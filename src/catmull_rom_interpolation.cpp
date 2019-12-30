#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>
#include <algorithm> // std::max
#include <cmath>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  double xzx0_t = 0.0;
  double xzx1_t = 0.0;
  double xzx2_t = 0.0;

  double inf = std::numeric_limits<double>::infinity();

  std::vector<std::pair<double, Eigen::Vector3d> > kf;
  for (int i=0; i<keyframes.size(); i++) {
    if (i == 0) {
      std::pair<double, Eigen::Vector3d> p1 ((-1)*inf, keyframes[i].second);
      std::pair<double, Eigen::Vector3d> p2 (keyframes[i].first, keyframes[i].second);
      kf.push_back(p1);
      kf.push_back(p2);
    }
    else {
      if (i == keyframes.size() - 1) {
        std::pair<double, Eigen::Vector3d> p1 (keyframes[i].first, keyframes[i].second);
        std::pair<double, Eigen::Vector3d> p2 (inf, keyframes[i].second);
        kf.push_back(p1);
        kf.push_back(p2);
      }
      else {
        std::pair<double, Eigen::Vector3d> p (keyframes[i].first, keyframes[i].second);
        kf.push_back(p);
      }
    }
  }


  if (kf.size() != 0) {

    for (int i=0; i<kf.size()-3; i++) {
      double t0 = kf[i+0].first;
      double t1 = kf[i+1].first;
      double t2 = kf[i+2].first;
      double t3 = kf[i+3].first;

      Eigen::Vector3d a0 = kf[i+0].second;
      Eigen::Vector3d a1 = kf[i+1].second;
      Eigen::Vector3d a2 = kf[i+2].second;
      Eigen::Vector3d a3 = kf[i+3].second;

      if ((t > t0) && (t < t3) && (t > t1) && (t < t2)) {
        double new_t = t - t1;
        double T = new_t/(t2-t1);
        xzx0_t = 0.5 * ((2*a1(0)) + (-a0(0)+a2(0))* T +
                        (2*a0(0)-5*a1(0)+4*a2(0)-a3(0))*T*T + (-a0(0)+3*a1(0)-3*a2(0)+a3(0))*T*T*T);
        xzx1_t = 0.5 * ((2*a1(1)) + (-a0(1)+a2(1))* T +
                        (2*a0(1)-5*a1(1)+4*a2(1)-a3(1))*T*T + (-a0(1)+3*a1(1)-3*a2(1)+a3(1))*T*T*T);
        xzx2_t = 0.5 * ((2*a1(2)) + (-a0(2)+a2(2))* T +
                        (2*a0(2)-5*a1(2)+4*a2(2)-a3(2))*T*T + (-a0(2)+3*a1(2)-3*a2(2)+a3(2))*T*T*T);

        break;
      }
    }
  }

  return Eigen::Vector3d(xzx0_t,xzx1_t,xzx2_t);
}
