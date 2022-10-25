//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2019-12-13
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

#include "LidarSlam/LidarPoint.h"
#include <unordered_map>
#include <map>
#include <random>
#include <algorithm>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{
class VoxelGrid
{
public:
  // Useful types
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  //============================================================================
  //! Initialize the grid to contain pointMin and pointMax
  void Init(const Eigen::Vector3f& ptMin, const Eigen::Vector3f& ptMax, float res, int maxPoints = 0);
  //! Remove all points from all voxels
  void Clear();

  void SetVoxelResolution(float res);
  GetMacro(VoxelResolution, float)

  SetMacro(Dimensions, const std::vector<Eigen::Array3i>&)
  GetMacro(Dimensions, std::vector<Eigen::Array3i>)

  //============================================================================

  //! Get up to maxNbPoints points in the grid.
  //! The keypoints are sorted in each voxel relatively to there confidence value.
  //! A loop is performed over the voxels and the next more confident point is
  //! extracted at each iteration until maxNbPoints are got.
  PointCloud::Ptr GetCloud(int maxNbPoints) const;
  //! Add a point to the grid.
  //! The value of the point define its weight to limit the number of extracting points
  //! The points with the lowest weights will be extracted first
  //! Points with the same weight will be extracted in arrival order
  void AddPoint(const LidarPoint& point, double value = 0.);

private:
  Eigen::Array3i Point2Voxel(const Eigen::Vector3f& pt);

private:
  // Dimensions of the grid
  std::vector<Eigen::Array3i> Dimensions {{-100, -100, -5}, {100, 100, 5}};
  //! [m/voxel] Resolution of a voxel
  float VoxelResolution = 0.1;
  //! Map containing one voxel index and the points contained in this voxel
  std::unordered_map<int, std::map<double, LidarPoint>> Voxels;
  //! Total number of points stored in voxel grid
  int Npoints = 0;
};

} // end of LidarSlam namespace