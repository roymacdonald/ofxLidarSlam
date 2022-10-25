//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-06-01
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

// LOCAL
#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"

// PCL
#include <pcl/point_cloud.h>

namespace LidarSlam
{
namespace Confidence
{

using Point = LidarPoint;
using PointCloud = pcl::PointCloud<Point>;

// Compute the LCP estimator (overlap estimator) for the registration of a
// pointcloud onto some prebuilt maps.
// It corresponds to the number of points from cloud which have a neighbor in
// the submaps relatively to the resolution of the maps.
// (see http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/ for more info)
// In this LCP extension, we also check the distance between nearest neighbors
// to make a smooth estimator.
// To accelerate the process, the ratio of points (between 0 and 1) from the
// input cloud to compute overlap on can be specified.
// It returns a valid overlap value between 0 and 1, or -1 if the overlap could
// not be computed (not enough points).
float LCPEstimator(PointCloud::ConstPtr cloud,
                   const std::map<Keypoint, std::shared_ptr<RollingGrid>>& maps,
                   float subsamplingRatio = 1.,
                   int nbThreads = 1);

} // enf of Confidence namespace
} // end of LidarSlam namespace