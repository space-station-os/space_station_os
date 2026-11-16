
#include <cmath>
#include <cstdio>

#include "ssos_gnc/common/cmg_kinematics.hpp"
#include "ssos_gnc/common/units.hpp"

using namespace ssos_gnc::common;

int main()
{
  CmgKinematics cmg;

  std::printf("CMG array: %d units, envelope %.1f N.m.s\n\n", kNumCmg, cmg.momentum_envelope());
  std::printf("  delta (all equal) [deg]   |h|/env    manipulability   min sigma\n");

  double worst_manip = 1e300;
  double worst_angle = 0.0;

  for (int deg = 0; deg <= 180; deg += 5) {
    Vector4 d;
    const double r = deg * kDegToRad;
    d << r, r, r, r;

    const double manip = cmg.manipulability(d);
    const double frac = cmg.momentum(d).norm() / cmg.momentum_envelope();
    const double sigma = cmg.min_singular_value(d);

    if (manip < worst_manip) {
      worst_manip = manip;
      worst_angle = deg;
    }

    if (deg % 15 == 0) {
      std::printf("%12d              %7.4f    %.6e   %9.3f\n", deg, frac, manip, sigma);
    }
  }

  std::printf(
    "\nleast well-conditioned sweep point: delta = %.0f deg, manipulability = %.6e\n",
    worst_angle, worst_manip);
  return 0;
}
