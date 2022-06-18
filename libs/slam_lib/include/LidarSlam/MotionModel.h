//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2019-04-08
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

#pragma once

#include <Eigen/Geometry>

namespace LidarSlam
{

/**
 * \struct LinearTransformInterpolator
 * \brief Linear interpolator to estimate an intermediate transform between two isometries.
 * 
 * At t=t0, the first isometry is returned, at t=t1 the second.
 * The translation will be interpolated linearly and the rotation spherical linearly.
 *
 * NOTE: if t0=t1 or H0=H1, this returns H0 to avoid numerical issues.
 */
template <typename T>
struct LinearTransformInterpolator
{
  // Useful types
  using Vector3T = Eigen::Matrix<T, 3, 1>;
  using QuaternionT = Eigen::Quaternion<T>;
  using Translation3T = Eigen::Translation<T, 3>;
  using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

  LinearTransformInterpolator()
    : LinearTransformInterpolator(Isometry3T::Identity(), Isometry3T::Identity())
  {}

  LinearTransformInterpolator(const Isometry3T& H0, const Isometry3T& H1, double t0 = 0., double t1 = 1.)
    : Time0(t0)
    , Time1(t1)
    , Rot0(H0.linear())
    , Rot1(H1.linear())
    , Trans0(H0.translation())
    , Trans1(H1.translation())
  {
    IsInvalid = !IsInterpolatorValid();
  }

  // Setters
  void SetH0(const Isometry3T& H0, double t0 = 0.)
  {
    Time0 = t0;
    Rot0 = QuaternionT(H0.linear());
    Trans0 = H0.translation();
    IsInvalid = !IsInterpolatorValid();
  }
  void SetH1(const Isometry3T& H1, double t1 = 1.)
  {
    Time1 = t1;
    Rot1 = QuaternionT(H1.linear());
    Trans1 = H1.translation();
    IsInvalid = !IsInterpolatorValid();
  }
  void SetTransforms(const Isometry3T& H0, const Isometry3T& H1)
  {
    SetH0(H0, Time0);
    SetH1(H1, Time1);
  }
  void SetTimes(double t0, double t1)
  {
    Time0 = t0;
    Time1 = t1;
    IsInvalid = !IsInterpolatorValid();
  }

  // Getters
  Isometry3T GetH0() const
  {
    return Translation3T(Trans0) * Rot0;
  }
  Isometry3T GetH1() const
  {
    return Translation3T(Trans1) * Rot1;
  }
  Isometry3T GetTransformRange() const
  {
    return GetH0().inverse() * GetH1();
  }
  double GetTime0() const
  {
    return Time0;
  }
  double GetTime1() const
  {
    return Time1;
  }
  double GetTimeRange() const
  {
    return Time1 - Time0;
  }

  // Return the affine isometry linearly interpolated at the requested time between H0 (t=t0) and H1 (t=t1).
  // If t0=t1 or H0=H1, this returns H0 to avoid numerical issues.
  Isometry3T operator()(double t) const
  {
    // Return first pose if interpolator is invalid to avoid numerical issues
    if (IsInvalid)
      return GetH0();
    // Otherwise, run interpolation
    const T time = T((t - Time0) / (Time1 - Time0));
    return Translation3T(Trans0 + time * (Trans1 - Trans0))  // Translation part : linear interpolation
            * Rot0.slerp(time, Rot1);                        // Rotation part : spherical interpolation
  }

  bool IsInterpolatorValid() const
  {
    return !(Time0 == Time1 || GetH0().isApprox(GetH1()));
  }

private:
  bool IsInvalid;
  double Time0, Time1;
  QuaternionT Rot0, Rot1;
  Vector3T Trans0, Trans1;
};

/**
 * \brief Interpolate spherical linearly between two isometries.
 * 
 * At t=t0, the first isometry is returned, at t=t1 the second.
 * The translation will be interpolated linearly and the rotation spherical linearly.
 */
Eigen::Isometry3d LinearInterpolation(const Eigen::Isometry3d& H0, const Eigen::Isometry3d& H1,
                                      double t, double t0 = 0., double t1 = 1.);

} // end of LidarSlam namespace