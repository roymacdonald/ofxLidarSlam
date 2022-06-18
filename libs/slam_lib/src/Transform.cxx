//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-06
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

#include "LidarSlam/Transform.h"
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{

//------------------------------------------------------------------------------
Transform::Transform(double x, double y, double z, double roll, double pitch, double yaw,
                     double t, const std::string& frame)
  : transform(Utils::XYZRPYtoIsometry(x, y, z, roll, pitch, yaw))
  , time(t)
  , frameid(frame)
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Vector6d& xyzrpy,
                     double t, const std::string& frame)
  : transform(Utils::XYZRPYtoIsometry(xyzrpy))
  , time(t)
  , frameid(frame)
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy,
                     double t, const std::string& frame)
  : transform(Utils::XYZRPYtoIsometry(trans(0), trans(1), trans(2), rpy(0), rpy(1), rpy(2)))
  , time(t)
  , frameid(frame)
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Isometry3d& transform,
                     double t, const std::string& frame)
  : transform(transform)
  , time(t)
  , frameid(frame)
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot,
                     double t, const std::string& frame)
  : transform(trans * rot.normalized())
  , time(t)
  , frameid(frame)
{}

} // end of LidarSlam namespace