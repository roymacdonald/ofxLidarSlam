//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-11-10
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
#include "LidarSlam/PointCloudStorage.h"
#include "LidarSlam/LidarPoint.h"

namespace LidarSlam
{
// Structure containing one state
// with all local information
struct LidarState
{
  using Point = LidarPoint;
  using PCStoragePtr = std::shared_ptr<PointCloudStorage<Point>>;

  // Transform to go from Sensor frame to tracking frame
  Eigen::UnalignedIsometry3d BaseToSensor = Eigen::UnalignedIsometry3d::Identity();
  // Pose transform in world coordinates
  Eigen::UnalignedIsometry3d Isometry = Eigen::UnalignedIsometry3d::Identity();
  // Covariance of current pose
  Eigen::Matrix6d Covariance;
  // [s] Timestamp of data
  double Time = 0.;
  // Index to link the pose graph (G2O)
  unsigned int Index = 0;

  LidarState() = default;
  LidarState(const Eigen::UnalignedIsometry3d& isometry, const Eigen::Matrix6d& covariance = {}, double time = 0.)
    : Isometry(isometry),
      Covariance(covariance),
      Time(time) {};
  // Keypoints extracted at current pose, undistorted and expressed in BASE coordinates
  std::map<Keypoint, PCStoragePtr> Keypoints;
  bool IsKeyFrame = true;
};

} // end of LidarSlam namespace