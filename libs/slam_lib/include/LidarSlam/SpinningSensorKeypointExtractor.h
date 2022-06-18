//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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
#include "LidarSlam/Enums.h"

#include <pcl/point_cloud.h>

#include <unordered_map>
#include <bitset>
#include <map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

class SpinningSensorKeypointExtractor
{
public:
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  GetMacro(NbThreads, int)
  SetMacro(NbThreads, int)

  GetMacro(NeighborWidth, int)
  SetMacro(NeighborWidth, int)

  GetMacro(MinDistanceToSensor, float)
  SetMacro(MinDistanceToSensor, float)

  GetMacro(MinBeamSurfaceAngle, float)
  SetMacro(MinBeamSurfaceAngle, float)

  GetMacro(PlaneSinAngleThreshold, float)
  SetMacro(PlaneSinAngleThreshold, float)

  GetMacro(EdgeSinAngleThreshold, float)
  SetMacro(EdgeSinAngleThreshold, float)

  GetMacro(EdgeDepthGapThreshold, float)
  SetMacro(EdgeDepthGapThreshold, float)

  GetMacro(EdgeSaliencyThreshold, float)
  SetMacro(EdgeSaliencyThreshold, float)

  GetMacro(EdgeIntensityGapThreshold, float)
  SetMacro(EdgeIntensityGapThreshold, float)

  GetMacro(AzimuthalResolution, float)
  SetMacro(AzimuthalResolution, float)

  GetMacro(NbLaserRings, int)

  std::map<Keypoint, PointCloud::Ptr> GetKeypoints() const {return this->Keypoints;}

  PointCloud::Ptr GetKeypoints(Keypoint k) const { return this->Keypoints.at(k); }

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc);

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<float>> GetDebugArray() const;

private:

  // Split the whole pointcloud into separate laser ring clouds,
  // sorted by their vertical angles.
  // This expects that the lowest/bottom laser ring is 0, and is increasing upward.
  void ConvertAndSortScanLines();

  // Reset all the features vectors and keypoints clouds
  void PrepareDataForNextFrame();

  // Invalid the points with bad criteria from the list of possible future keypoints.
  // These points correspond to planar surfaces roughly parallel to laser beam
  // and points close to a gap created by occlusion.
  void InvalidateNotUsablePoints();

  // Compute the curvature and other features within each the scan line.
  // The curvature is not the one of the surface that intersects the lines but
  // the 1D curvature within each isolated scan line.
  void ComputeCurvature();

  // Labelize point to be a keypoints or not
  void SetKeyPointsLabels();

  // Auto estimate azimuth angle resolution based on current ScanLines
  // WARNING: to be correct, the points need to be in the LIDAR sensor
  // coordinates system, where the sensor is spinning around Z axis.
  void EstimateAzimuthalResolution();

  // Check if scanLine is almost empty
  inline bool IsScanLineAlmostEmpty(int nScanLinePts) const { return nScanLinePts < 2 * this->NeighborWidth + 1; }

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Max number of threads to use to process points in parallel
  int NbThreads = 1;

  // Width of the neighborhood used to compute discrete differential operators
  int NeighborWidth = 4;

  // Minimal point/sensor sensor to consider a point as valid
  float MinDistanceToSensor = 1.5;  // [m]

  // Minimum angle between laser beam and surface to consider a point as valid
  float MinBeamSurfaceAngle = 10; // [°]

  // Sharpness threshold to select a planar keypoint
  float PlaneSinAngleThreshold = 0.5;  // sin(30°) (selected if sin angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  float EdgeSinAngleThreshold = 0.86;  // ~sin(60°) (selected, if sin angle is more than threshold)
  float DistToLineThreshold = 0.20;  // [m]

  // Threshold upon depth gap in neighborhood to select an edge keypoint
  float EdgeDepthGapThreshold = 0.15;  // [m]

  // Threshold upon saliency of a neighborhood to select an edge keypoint
  float EdgeSaliencyThreshold = 1.5;  // [m]

  // Threshold upon intensity gap to select an edge keypoint
  float EdgeIntensityGapThreshold = 50.;

  // Threshold upon sphericity of a neighborhood to select a blob point
  float SphericityThreshold = 0.35;  // CHECK : unused

  // Coef to apply to the incertitude radius of the blob neighborhood
  float IncertitudeCoef = 3.0;  // CHECK : unused

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  // Azimuthal (= horizontal angle) resolution of the spinning lidar sensor
  // If it is less or equal to 0, it will be auto-estimated from next frame.
  // This angular resolution is used to compute an expected distance between two
  // consecutives firings.
  float AzimuthalResolution = 0.;  // [rad]

  // Number of lasers scan lines composing the pointcloud
  unsigned int NbLaserRings = 0;

  //! Label of a point as a keypoint
  //! We use binary flags as each point can have different keypoint labels.
  using KeypointFlags = std::bitset<Keypoint::nKeypointTypes>;

  // Curvature and other differential operations (scan by scan, point by point)
  std::vector<std::vector<float>> Angles;
  std::vector<std::vector<float>> DepthGap;
  std::vector<std::vector<float>> Saliency;
  std::vector<std::vector<float>> IntensityGap;
  std::vector<std::vector<KeypointFlags>> IsPointValid;
  std::vector<std::vector<KeypointFlags>> Label;

  // Extracted keypoints of current frame
  std::map<Keypoint, PointCloud::Ptr> Keypoints;

  // Current point cloud stored in two differents formats
  PointCloud::Ptr Scan;
  std::vector<PointCloud::Ptr> ScanLines;
};

} // end of LidarSlam namespace