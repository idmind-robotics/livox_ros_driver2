// Self-check for the extrinsic transform built by LidarPubHandler.
//
// The rotation matrix it builds is reproduced here rather than linked, because
// LidarPubHandler owns an SDK point-cloud observer and cannot be constructed
// without a live SDK. Keep BuildRotation() identical to
// LidarPubHandler::SetLidarsExtParam() in src/comm/pub_handler.cpp.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "comm/comm.h"

using livox_ros::ExtParameter;
using livox_ros::ExtParameterDetailed;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-6;

/** The identity that LidarPubHandler starts from, before any config is applied. */
ExtParameterDetailed Identity() {
  return ExtParameterDetailed{{0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

ExtParameterDetailed BuildRotation(const ExtParameter &p) {
  ExtParameterDetailed e{};
  e.trans[0] = p.x;
  e.trans[1] = p.y;
  e.trans[2] = p.z;

  const double cos_roll = cos(p.roll * kPi / 180.0);
  const double cos_pitch = cos(p.pitch * kPi / 180.0);
  const double cos_yaw = cos(p.yaw * kPi / 180.0);
  const double sin_roll = sin(p.roll * kPi / 180.0);
  const double sin_pitch = sin(p.pitch * kPi / 180.0);
  const double sin_yaw = sin(p.yaw * kPi / 180.0);

  e.rotation[0][0] = cos_pitch * cos_yaw;
  e.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  e.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
  e.rotation[1][0] = cos_pitch * sin_yaw;
  e.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  e.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
  e.rotation[2][0] = -sin_pitch;
  e.rotation[2][1] = sin_roll * cos_pitch;
  e.rotation[2][2] = cos_roll * cos_pitch;
  return e;
}

/** Same arithmetic as ProcessCartesianHighPoint(): raw mm in, metres out. */
void Apply(const ExtParameterDetailed &e, const double in[3], double out[3]) {
  for (int r = 0; r < 3; ++r) {
    out[r] = (in[0] * e.rotation[r][0] + in[1] * e.rotation[r][1] +
              in[2] * e.rotation[r][2] + e.trans[r]) / 1000.0;
  }
}

void Check(const char *what, const double got[3], double x, double y, double z) {
  if (fabs(got[0] - x) > kEps || fabs(got[1] - y) > kEps || fabs(got[2] - z) > kEps) {
    printf("FAIL %s: got (%g, %g, %g), want (%g, %g, %g)\n",
           what, got[0], got[1], got[2], x, y, z);
    assert(false);
  }
}

}  // namespace

int main() {
  double out[3];

  // An unconfigured lidar must pass its points through untouched. The shipped
  // default had rotation[1] == {0, 1, 1}, which turned every point into y + z.
  const ExtParameterDetailed identity = Identity();
  const double up[3] = {0, 0, 1000};
  Apply(identity, up, out);
  Check("identity, z axis", out, 0, 0, 1.0);

  const double diag[3] = {1000, 2000, 3000};
  Apply(identity, diag, out);
  Check("identity, arbitrary point", out, 1.0, 2.0, 3.0);

  // yaw 90 deg maps +x onto +y.
  ExtParameter yaw90{};
  yaw90.yaw = 90.0f;
  const double fwd[3] = {1000, 0, 0};
  Apply(BuildRotation(yaw90), fwd, out);
  Check("yaw 90, x axis", out, 0, 1.0, 0);

  // Translation is in mm and lands in metres.
  ExtParameter shift{};
  shift.x = 1000;
  Apply(BuildRotation(shift), up, out);
  Check("translation x = 1000mm", out, 1.0, 0, 1.0);

  // A pure roll must leave the roll axis alone.
  ExtParameter roll90{};
  roll90.roll = 90.0f;
  Apply(BuildRotation(roll90), fwd, out);
  Check("roll 90, x axis", out, 1.0, 0, 0);

  printf("test_extrinsic: all checks passed\n");
  return 0;
}
