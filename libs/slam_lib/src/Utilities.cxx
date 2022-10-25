//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-06-16
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include "LidarSlam/Utilities.h"

#include <unordered_map>
#include <chrono>

namespace LidarSlam
{
namespace Utils
{
//==============================================================================
//   Geometry helpers
//==============================================================================

//------------------------------------------------------------------------------
Eigen::Matrix3d RPYtoRotationMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

//------------------------------------------------------------------------------
Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& rot)
{
  // `rpy = rot.eulerAngles(2, 1, 0).reverse()`             returns angles in range [-PI:PI]x[-PI:PI]x[0:PI].
  // `rpy = Eigen::EulerAnglesZYXd(rot).angles().reverse()` returns angles in range [-PI:PI]x[-PI:PI]x[-PI:PI].
  // But these are bad. For first range, yaw angle cannot be negative : this
  // leads to un-necessary non trivial RPY decomposition, and to unstable
  // optimization result as we are not optimizing around 0.
  // For second ranges, there exist several RPY decomposition for the same
  // rotation (one of them being non-trivial too). Therefore the optimization
  // may also be unstable by oscillating between them.
  // We prefer to output angles in range [-PI:PI]x[-PI/2:PI/2]x[-PI:PI] : we
  // allow negative values to avoid oscillation artefacts, and minimize the
  // pitch angle to fix representation.
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(rot(2, 1), rot(2, 2));
  rpy.y() = -std::asin(rot(2, 0));
  rpy.z() = std::atan2(rot(1, 0), rot(0, 0));
  return rpy;
}

//------------------------------------------------------------------------------
Eigen::Isometry3d PoseToIsometry(const Eigen::Vector6d& pose)
{
  return XYZRPYtoIsometry(pose(0), pose(1), pose(2), pose(3), pose(4), pose(5));
}

//------------------------------------------------------------------------------
Eigen::Isometry3d XYZRPYtoIsometry(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d transform;
  transform.linear() = RPYtoRotationMatrix(roll, pitch, yaw);  // Set rotation part
  transform.translation() = Eigen::Vector3d(x, y, z);          // Set translation part
  transform.makeAffine();                                      // Set the last row to [0 0 0 1]
  return transform;
}

//------------------------------------------------------------------------------
Eigen::Vector6d IsometryToXYZRPY(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d xyzrpy;
  xyzrpy << transform.translation(), RotationMatrixToRPY(transform.linear());
  return xyzrpy;
}

//------------------------------------------------------------------------------
Eigen::Vector6d IsometryToRPYXYZ(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d rpyxyz;
  rpyxyz << RotationMatrixToRPY(transform.linear()), transform.translation();
  return rpyxyz;
}

//------------------------------------------------------------------------------
std::string Capitalize(std::string st)
{
  st[0] = std::toupper(st[0]);
  return st;
}

//------------------------------------------------------------------------------
std::string Plural(std::string st)
{
  return st + 's';
}

//==============================================================================
//   Processing duration measurements
//==============================================================================

namespace Timer
{
  //----------------------------------------------------------------------------
  // Anonymous namespace to avoid accessing these variables from outside.
  namespace
  {
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> startTimestamps;
    std::unordered_map<std::string, double> totalDurations;
    std::unordered_map<std::string, unsigned int> totalCalls;
  } // end of anonymous namespace

  //----------------------------------------------------------------------------
  void Reset()
  {
    startTimestamps.clear();
    totalDurations.clear();
    totalCalls.clear();
  }

  //----------------------------------------------------------------------------
  void Init(const std::string& timer)
  {
    startTimestamps[timer] = std::chrono::steady_clock::now();
  }

  //----------------------------------------------------------------------------
  double Stop(const std::string& timer)
  {
    std::chrono::duration<double> chrono_s = std::chrono::steady_clock::now() - startTimestamps[timer];
    double duration = chrono_s.count();
    totalDurations[timer] += duration;
    totalCalls[timer]++;
    return duration;
  }

  //----------------------------------------------------------------------------
  void StopAndDisplay(const std::string& timer, int nbDigits)
  {
    const double currentDuration = Stop(timer);
    double meanDurationMs = totalDurations[timer] * 1000. / totalCalls[timer];
    SET_COUT_FIXED_PRECISION(nbDigits);
    PRINT_COLOR(CYAN, "  -> " << timer << " took : " << currentDuration * 1000. << " ms (average : " << meanDurationMs << " ms)");
    RESET_COUT_FIXED_PRECISION;
  }

  //----------------------------------------------------------------------------
  void Display(const std::string& timer, int nbDigits)
  {
    double meanDurationMs = totalDurations[timer] * 1000. / totalCalls[timer];
    SET_COUT_FIXED_PRECISION(nbDigits);
    PRINT_COLOR(CYAN, "  -> " << timer << " took in average : " << meanDurationMs << " ms");
    RESET_COUT_FIXED_PRECISION;
  }
}  // end of Timer namespace
}  // end of Utils namespace
}  // end of LidarSlam namespace