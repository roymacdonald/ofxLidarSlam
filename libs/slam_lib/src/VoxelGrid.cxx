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

#include "LidarSlam/VoxelGrid.h"
#include "LidarSlam/Utilities.h"


namespace LidarSlam
{
//==============================================================================
//   Reset and parameters setters
//==============================================================================

//------------------------------------------------------------------------------
void VoxelGrid::Init(const Eigen::Vector3f& ptMin, const Eigen::Vector3f& ptMax, float res, int maxPoints)
{
  this->Dimensions[0] = this->Point2Voxel(ptMin);
  this->Dimensions[1] = this->Point2Voxel(ptMax);
  this->VoxelResolution = res;
  if (maxPoints)
    this->Voxels.reserve(maxPoints);
}

//------------------------------------------------------------------------------
void VoxelGrid::Clear()
{
  this->Voxels.clear();
  this->Npoints = 0;
}

//------------------------------------------------------------------------------
void VoxelGrid::SetVoxelResolution(float res)
{
  if (res == this->VoxelResolution)
    return;

  this->VoxelResolution = res;

  // Clear the voxels and store the points in a temporal structure
  std::unordered_map<int, std::map<double, LidarPoint>> tempVoxels = std::move(this->Voxels);

  this->Clear();
  for (auto& v : tempVoxels)
  {
    for (const auto& vPt : v.second)
      this->AddPoint(vPt.second, vPt.first);
  }
}

//==============================================================================
//   Main use
//==============================================================================

//------------------------------------------------------------------------------
//int RNG (int n) {return 0;}
VoxelGrid::PointCloud::Ptr VoxelGrid::GetCloud(int maxNbPoints) const
{
  PointCloud::Ptr pc(new PointCloud);

  if (this->Npoints < maxNbPoints)
  {
    pc->resize(this->Npoints);
    int ptIdx = 0;
    // Add all voxels to the output cloud
    for (const auto& v : this->Voxels)
    {
      // Add all voxel points to the output cloud
      for (const auto& vPt : v.second)
      {
        pc->at(ptIdx) = vPt.second;
        ++ptIdx;
      }
    }
    return pc;
  }

  pc->resize(maxNbPoints);
  int ptIdx = 0;
  unsigned int loop = 1;

  // Shuffle voxel indices to be sure not to get a specific spatial area
  // when browsing through the voxel grid
  std::vector<int> voxelIndices(this->Voxels.size());
  int idx = 0;
  for (const auto& vox2Pts : this->Voxels)
  {
    voxelIndices[idx] = vox2Pts.first;
    ++idx;
  }

  // Seed generator for deterministic processes
//  struct RNG {
//  int RNG (int n) {return 0; static_cast<void>(n);}
//  };
  std::shuffle(voxelIndices.begin(), voxelIndices.end(), std::default_random_engine(0));


  // Loop over all voxels while there are not enough points;
  while (ptIdx < maxNbPoints)
  {
    // For each voxel, take the point with the next biggest value
    for (int idx : voxelIndices)
    {
      // Shortcut
      const std::map<double, LidarPoint>& points = this->Voxels.at(idx);
      if (points.size() < loop)
        continue;
      // Get next most reliable keypoint in this voxel
      auto ptValuedPtr = points.end();
      for (unsigned int i = 0; i < loop; ++i)
        --ptValuedPtr;
      // Add this point to the output cloud
      pc->at(ptIdx) = ptValuedPtr->second;
      ++ptIdx;
      if (ptIdx >= maxNbPoints)
        break;
    }
    ++loop;
  }
  return pc;
}

//------------------------------------------------------------------------------
void VoxelGrid::AddPoint(const LidarPoint& point, double value)
{
  // Find the voxel containing this point
  Eigen::Array3i voxelCoords = this->Point2Voxel(point.getVector3fMap());
  // Check if point is in bounds
  bool inside = (voxelCoords - this->Dimensions[0] >= 0).all() && (voxelCoords - this->Dimensions[1] <= 0).all();
  if (!inside)
  {
    PRINT_WARNING("Point cannot be added to grid : it does not fit the dimensions")
    return;
  }

  // Get voxel index
  Eigen::Array3i gridSize = this->Dimensions[1] - this->Dimensions[0];
  int idx = voxelCoords.z() * gridSize.x() * gridSize.y() + voxelCoords.y() * gridSize.x() + voxelCoords.x();

  // Adapt value to be insertable
  while (this->Voxels[idx].count(value))
    value += 1e-6;
  // Add point to grid
  this->Voxels[idx][value] = point;

  ++this->Npoints;
}

//==============================================================================
//   Helper
//==============================================================================

Eigen::Array3i VoxelGrid::Point2Voxel(const Eigen::Vector3f& pt)
{
  Eigen::Vector3f offset = (this->VoxelResolution / 2.) * Eigen::Vector3f::Ones();
  return ((pt + offset) / this->VoxelResolution).array().cast<int>();
}

} // end of LidarSlam namespace
