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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/SpinningSensorKeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace LidarSlam
{

namespace
{
//-----------------------------------------------------------------------------
struct LineFitting
{
  //! Fitting using PCA
  bool FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
              const std::vector<int>& indices);

  //! Fitting using very local line and check if this local line is consistent
  //! in a more global neighborhood
  bool FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                 const std::vector<int>& indices);

  //! Compute the squared distance of a point to the fitted line
  inline float SquaredDistanceToPoint(Eigen::Vector3f const& point) const;

  // Direction and position
  Eigen::Vector3f Direction;
  Eigen::Vector3f Position;

  //! Max distance allowed from the farest point to estimated line to be considered as real line
  float MaxDistance = 0.02;  // [m]

  //! Max angle allowed between consecutive segments in the neighborhood to be considered as line
  float MaxAngle = DEG2RAD(40.);  // [rad]
};

//-----------------------------------------------------------------------------
bool LineFitting::FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                         const std::vector<int>& indices)
{
  // Compute PCA to determine best line approximation of the points distribution
  // and save points centroid in Position
  Eigen::Vector3f eigVals;
  Eigen::Matrix3f eigVecs;
  Utils::ComputeMeanAndPCA(cloud, indices, this->Position, eigVecs, eigVals);

  // Get Direction as main eigen vector
  this->Direction = eigVecs.col(2);

  // If a point of the neighborhood is too far from the fitted line,
  // we consider the neighborhood as non flat
  bool isLineFittingAccurate = true;
  const float sqMaxDistance = this->MaxDistance * this->MaxDistance;
  for (const auto& pointId: indices)
  {
    if (this->SquaredDistanceToPoint(cloud[pointId].getVector3fMap()) > sqMaxDistance)
    {
      isLineFittingAccurate = false;
      break;
    }
  }
  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  const float maxSinAngle = std::sin(this->MaxAngle);
  bool isLineFittingAccurate = true;

  // First check if the neighborhood is approximately straight
  const Eigen::Vector3f U = (cloud[indices.back()].getVector3fMap() - cloud[indices.front()].getVector3fMap()).normalized();
  for (unsigned int i = 0; i < indices.size() - 1; i++)
  {
    const Eigen::Vector3f V = (cloud[indices[i + 1]].getVector3fMap() - cloud[indices[i]].getVector3fMap()).normalized();
    const float sinAngle = (U.cross(V)).norm();
    if (sinAngle > maxSinAngle)
    {
      isLineFittingAccurate = false;
      break;
    }
  }

  // Then fit with PCA (only if isLineFittingAccurate is true)
  return isLineFittingAccurate && this->FitPCA(cloud, indices);
}

//-----------------------------------------------------------------------------
inline float LineFitting::SquaredDistanceToPoint(Eigen::Vector3f const& point) const
{
  return ((point - this->Position).cross(this->Direction)).squaredNorm();
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Invalidate points with bad criteria
  this->InvalidateNotUsablePoints();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->ScanLines)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as ScanLines is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->Scan)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->ScanLines.size())
      this->ScanLines.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->ScanLines[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NbLaserRings = this->ScanLines.size();

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Do not use clear(), otherwise weird things could happen
  // if outer program uses these pointers
  for (auto k : KeypointTypes)
  {
    this->Keypoints[k].reset(new PointCloud);
    Utils::CopyPointCloudMetadata(*this->Scan, *this->Keypoints[k]);
  }

  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NbLaserRings);
  this->Saliency.resize(this->NbLaserRings);
  this->DepthGap.resize(this->NbLaserRings);
  this->IntensityGap.resize(this->NbLaserRings);
  this->IsPointValid.resize(this->NbLaserRings);
  this->Label.resize(this->NbLaserRings);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    size_t nbPoint = this->ScanLines[scanLine]->size();
    this->IsPointValid[scanLine].assign(nbPoint, KeypointFlags().set());  // set all flags to 1
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLine].assign(nbPoint, 0.);
    this->Saliency[scanLine].assign(nbPoint, 0.);
    this->DepthGap[scanLine].assign(nbPoint, 0.);
    this->IntensityGap[scanLine].assign(nbPoint, 0.);
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidateNotUsablePoints()
{
  // Max angle between Lidar Ray and hyptothetic plane normal
  const float angleBeamNormal = Utils::Deg2Rad(90 - this->MinBeamSurfaceAngle);
  // If the azimuthal angle was estimated and is plausible,
  // it is used to invalidate points representing occluded areas border.
  // Otherwise, we use a default angle (Velodyne 10Hz resolution)
  float azimuthalResolution = this->AzimuthalResolution;
  if (azimuthalResolution < 1e-6 || M_PI / 4 < azimuthalResolution)
  {
    PRINT_WARNING("Unable to estimate the azimuthal resolution angle: using 0.2°");
    azimuthalResolution = Utils::Deg2Rad(0.2);
  }
  // Coeff to multiply to point depth, in order to obtain the maximal distance
  // between two neighbors of the same Lidar ray on a plane
  const float maxPosDiffCoeff = std::sin(azimuthalResolution) / std::cos(azimuthalResolution + angleBeamNormal);

  // Loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) firstprivate(maxPosDiffCoeff)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      for (int index = 0; index < Npts; ++index)
        this->IsPointValid[scanLine][index].reset();
      continue;
    }

    // Invalidate first and last points: because of undistortion, the depth of
    // first and last points can not be compared
    for (int index = 0; index < this->NeighborWidth; ++index)
    {
      this->IsPointValid[scanLine][index].reset();
      this->IsPointValid[scanLine][Npts - 1 - index].reset();
    }

    // Loop over remaining points of the scan line
    for (int index = this->NeighborWidth; index < Npts - this->NeighborWidth; ++index)
    {
      const auto& currentPoint = scanLineCloud[index].getVector3fMap();
      const float L = currentPoint.norm();
      // Invalidate points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index].reset();
      }

      // Compute maximal acceptable distance of two consecutive neighbors
      // aquired by the same laser, considering they lay on the same plane which
      // is not too oblique relatively to Lidar ray.
      // Check that this expected distance is not below range measurements noise.
      const float maxPosDiff = std::max(L * maxPosDiffCoeff, 0.02f);
      const float sqMaxPosDiff = maxPosDiff * maxPosDiff;

      // Invalidate occluded points due to depth gap or parallel beam.
      // If the distance between two successive points is bigger than the
      // expected length, it means that there is a depth gap. In this case, we
      // must invalidate the farthests points which belong to the occluded area.
      const auto& nextPoint = scanLineCloud[index + 1].getVector3fMap();
      if ((nextPoint - currentPoint).squaredNorm() > sqMaxPosDiff)
      {
        // If current point is the closest, next part is invalidated, starting from next point
        if (L < nextPoint.norm())
        {
          this->IsPointValid[scanLine][index + 1].reset();
          for (int i = index + 1; i < index + this->NeighborWidth; ++i)
          {
            const auto& Y  = scanLineCloud[i].getVector3fMap();
            const auto& Yn = scanLineCloud[i + 1].getVector3fMap();
            // If there is a new gap in the neighborhood, 
            // the remaining points of the neighborhood are kept.
            if ((Yn - Y).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the current neighbor point is disabled
            this->IsPointValid[scanLine][i + 1].reset();
          }
        }
        // If current point is the farthest, invalidate previous part, starting from current point
        else
        {
          this->IsPointValid[scanLine][index].reset();
          for (int i = index - 1; i > index - this->NeighborWidth; --i)
          {
            const auto& Yp = scanLineCloud[i].getVector3fMap();
            const auto&  Y = scanLineCloud[i + 1].getVector3fMap();
            // If there is a new gap in the neighborhood, 
            // the remaining points of the neighborhood are kept.
            if ((Y - Yp).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the previous neighbor point is disabled
            this->IsPointValid[scanLine][i].reset();
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  const float sqDistToLineThreshold = this->DistToLineThreshold * this->DistToLineThreshold;  // [m²]
  const float sqDepthDistCoeff = 0.25;
  const float minDepthGapDist = 1.5;  // [m]

  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(sqDistToLineThreshold, sqDepthDistCoeff, minDepthGapDist)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (this->IsPointValid[scanLine][index].none())
      {
        continue;
      }

      // central point
      const Point& currentPoint = scanLineCloud[index];
      const Eigen::Vector3f centralPoint = currentPoint.getVector3fMap();

      // compute intensity gap
      // CHECK : do not use currentPoint.intensity?
      const Point& previousPoint = scanLineCloud[index - 1];
      const Point& nextPoint = scanLineCloud[index + 1];
      this->IntensityGap[scanLine][index] = std::abs(nextPoint.intensity - previousPoint.intensity);

      // We will compute the line that fits the neighbors located before the current point.
      // We will do the same for the neighbors located after the current point.
      // We will then compute the angle between these two lines as an approximation
      // of the "sharpness" of the current point.
      std::vector<int> leftNeighbors(this->NeighborWidth);
      std::vector<int> rightNeighbors(this->NeighborWidth);
      LineFitting leftLine, rightLine;

      // Fill left and right neighborhoods, from central point to sides.
      // /!\ The way the neighbors are added to the vectors matters,
      // especially when computing the saliency
      for (int j = index - 1; j >= index - this->NeighborWidth; --j)
        leftNeighbors[index - 1 - j] = j;
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
        rightNeighbors[j - index - 1] = j;

      // Fit line on the left and right neighborhoods and
      // Indicate if they are flat or not
      const bool leftFlat = leftLine.FitPCAAndCheckConsistency(scanLineCloud, leftNeighbors);
      const bool rightFlat = rightLine.FitPCAAndCheckConsistency(scanLineCloud, rightNeighbors);

      // Measurement of the depth gap
      float distLeft = 0., distRight = 0.;

      // If both neighborhoods are flat, we can compute the angle between them
      // as an approximation of the sharpness of the current point
      if (leftFlat && rightFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as an angle point if it is due to gap
        distLeft = leftLine.SquaredDistanceToPoint(centralPoint);
        distRight = rightLine.SquaredDistanceToPoint(centralPoint);

        // If current point is not too far from estimated lines,
        // save the sin of angle between these two lines
        if ((distLeft < sqDistToLineThreshold) && (distRight < sqDistToLineThreshold))
          this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
      }

      // Here one side of the neighborhood is non flat.
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      // CHECK : looks strange to estimate depth gap without considering current point
      else if (!leftFlat && rightFlat)
      {
        distLeft = std::numeric_limits<float>::max();
        for (const auto& leftNeighborId: leftNeighbors)
        {
          const auto& leftNeighbor = scanLineCloud[leftNeighborId].getVector3fMap();
          distLeft = std::min(distLeft, rightLine.SquaredDistanceToPoint(leftNeighbor));
        }
        distLeft *= sqDepthDistCoeff;
      }
      else if (leftFlat && !rightFlat)
      {
        distRight = std::numeric_limits<float>::max();
        for (const auto& rightNeighborId: rightNeighbors)
        {
          const auto& rightNeighbor = scanLineCloud[rightNeighborId].getVector3fMap();
          distRight = std::min(distRight, leftLine.SquaredDistanceToPoint(rightNeighbor));
        }
        distRight *= sqDepthDistCoeff;
      }

      // No neighborhood is flat.
      // We will compute saliency of the current keypoint from its far neighbors.
      else
      {
        // Compute salient point score
        const float sqCurrDepth = centralPoint.squaredNorm();
        bool hasLeftEncounteredDepthGap = false;
        bool hasRightEncounteredDepthGap = false;

        std::vector<int> farNeighbors;
        farNeighbors.reserve(2 * this->NeighborWidth);

        // The salient point score is the distance between the current point
        // and the points that have a depth gap with the current point
        // CHECK : consider only consecutive far neighbors, starting from the central point.
        for (const auto& leftNeighborId: leftNeighbors)
        {
          // Left neighborhood depth gap computation
          if (std::abs(scanLineCloud[leftNeighborId].getVector3fMap().squaredNorm() - sqCurrDepth) > minDepthGapDist)
          {
            hasLeftEncounteredDepthGap = true;
            farNeighbors.emplace_back(leftNeighborId);
          }
          else if (hasLeftEncounteredDepthGap)
            break;
        }
        for (const auto& rightNeighborId: rightNeighbors)
        {
          // Right neigborhood depth gap computation
          if (std::abs(scanLineCloud[rightNeighborId].getVector3fMap().squaredNorm() - sqCurrDepth) > minDepthGapDist)
          {
            hasRightEncounteredDepthGap = true;
            farNeighbors.emplace_back(rightNeighborId);
          }
          else if (hasRightEncounteredDepthGap)
            break;
        }

        // If there are enough neighbors with a big depth gap,
        // we propose to compute the saliency of the current point
        // as the distance between the line that roughly fits the far neighbors
        // with a depth gap and the current point
        if (farNeighbors.size() > static_cast<unsigned int>(this->NeighborWidth))
        {
          LineFitting farNeighborsLine;
          farNeighborsLine.FitPCA(scanLineCloud, farNeighbors);
          this->Saliency[scanLine][index] = farNeighborsLine.SquaredDistanceToPoint(centralPoint);
        }
      }

      // Store max depth gap
      this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  const float sqEdgeSaliencythreshold = this->EdgeSaliencyThreshold * this->EdgeSaliencyThreshold;
  const float sqEdgeDepthGapThreshold = this->EdgeDepthGapThreshold * this->EdgeDepthGapThreshold;

  // loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(sqEdgeSaliencythreshold, sqEdgeDepthGapThreshold)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    const int Npts = this->ScanLines[scanLine]->size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // Sort the curvature score in a decreasing order
    std::vector<size_t> sortedDepthGapIdx  = Utils::SortIdx(this->DepthGap    [scanLine], false);
    std::vector<size_t> sortedAnglesIdx    = Utils::SortIdx(this->Angles      [scanLine], false);
    std::vector<size_t> sortedSaliencyIdx  = Utils::SortIdx(this->Saliency    [scanLine], false);
    std::vector<size_t> sortedIntensityGap = Utils::SortIdx(this->IntensityGap[scanLine], false);

    // Add edge according to criterion
    auto addEdgesUsingCriterion = [this, scanLine, Npts](const std::vector<size_t>& sortedValuesIdx,
                                                         const std::vector<std::vector<float>>& values,
                                                         float threshold,
                                                         int invalidNeighborhoodSize)
    {
      for (const auto& index: sortedValuesIdx)
      {
        // Check criterion threshold
        // If criterion is not respected, break loop as indices are sorted in decreasing order.
        if (values[scanLine][index] < threshold)
          break;

        // If the point is invalid as edge, continue
        if (!this->IsPointValid[scanLine][index][Keypoint::EDGE])
          continue;

        // Else indicate that the point is an edge
        this->Label[scanLine][index].set(Keypoint::EDGE);

        // Invalid its neighbors
        const int indexBegin = std::max(0,        static_cast<int>(index - invalidNeighborhoodSize));
        const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + invalidNeighborhoodSize));
        for (int j = indexBegin; j <= indexEnd; ++j)
          this->IsPointValid[scanLine][j].reset(Keypoint::EDGE);
      }
    };

    // Edges using depth gap
    addEdgesUsingCriterion(sortedDepthGapIdx, this->DepthGap, sqEdgeDepthGapThreshold, this->NeighborWidth - 1);
    // Edges using angles
    addEdgesUsingCriterion(sortedAnglesIdx, this->Angles, this->EdgeSinAngleThreshold, this->NeighborWidth);
    // Edges using saliency
    addEdgesUsingCriterion(sortedSaliencyIdx, this->Saliency, sqEdgeSaliencythreshold, this->NeighborWidth - 1);
    // Edges using intensity
    addEdgesUsingCriterion(sortedIntensityGap, this->IntensityGap, this->EdgeIntensityGapThreshold, 1);

    // Planes (using angles)
    for (int k = Npts - 1; k >= 0; --k)
    {
      size_t index = sortedAnglesIdx[k];
      const float sinAngle = this->Angles[scanLine][index];

      // thresh
      if (sinAngle > this->PlaneSinAngleThreshold)
        break;

      // if the point is invalid as plane or sinAngle value is unset, continue
      if (!this->IsPointValid[scanLine][index][Keypoint::PLANE] || sinAngle < 1e-6)
        continue;

      // else indicate that the point is a planar one
      this->Label[scanLine][index].set(Keypoint::PLANE);

      // Invalid its neighbors so that we don't have too
      // many planar keypoints in the same region. This is
      // required because of the k-nearest search + plane
      // approximation realized in the odometry part. Indeed,
      // if all the planar points are on the same scan line the
      // problem is degenerated since all the points are distributed
      // on a line.
      const int indexBegin = std::max(0,        static_cast<int>(index - 4));
      const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + 4));
      for (int j = indexBegin; j <= indexEnd; ++j)
        this->IsPointValid[scanLine][j].reset(Keypoint::PLANE);
    }

    // Blobs Points
    // CHECK : why using only 1 point over 3?
    // TODO : disable blobs if not required
    for (int index = 0; index < Npts; index += 3)
    {
      if (this->IsPointValid[scanLine][index][Keypoint::BLOB])
        this->Label[scanLine][index].set(Keypoint::BLOB);
    }
  }

  for (unsigned int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    for (unsigned int index = 0; index < scanLineCloud.size(); ++index)
    {
      for (const auto& k : KeypointTypes)
      {
        if (this->Label[scanLine][index][k])
        {
          this->IsPointValid[scanLine][index].set(k);
          this->Keypoints[k]->push_back(scanLineCloud[index]);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  for (const PointCloud::Ptr& scanLine : this->ScanLines)
  {
    for (unsigned int index = 1; index < scanLine->size(); ++index)
    {
      // Compute horizontal angle between two measurements
      // WARNING: to be correct, the points need to be in the LIDAR sensor
      // coordinates system, where the sensor is spinning around Z axis.
      Eigen::Map<const Eigen::Vector2f> p1(scanLine->at(index - 1).data);
      Eigen::Map<const Eigen::Vector2f> p2(scanLine->at(index).data);
      float angle = std::abs(std::acos(p1.dot(p2) / (p1.norm() * p2.norm())));

      // Keep only angles greater than 0 to avoid dual return issues
      if (angle > 1e-4)
        angles.push_back(angle);
    }
  }

  // A minimum number of angles is needed to get a trustable estimator
  if (angles.size() < 100)
  {
    PRINT_WARNING("Not enough points to estimate azimuthal resolution");
    return;
  }

  // Estimate azimuthal resolution from these angles
  std::sort(angles.begin(), angles.end());
  unsigned int maxInliersIdx = angles.size();
  float maxAngle = Utils::Deg2Rad(5.);
  float medianAngle = 0.;
  // Iterate until only angles between direct LiDAR beam neighbors remain.
  // The max resolution angle is decreased at each iteration.
  while (maxAngle > 1.8 * medianAngle)
  {
    maxInliersIdx = std::upper_bound(angles.begin(), angles.begin() + maxInliersIdx, maxAngle) - angles.begin();
    medianAngle = angles[maxInliersIdx / 2];
    maxAngle = std::min(medianAngle * 2., maxAngle / 1.8);
  }
  this->AzimuthalResolution = medianAngle;
  std::cout << "LiDAR's azimuthal resolution estimated to " << Utils::Rad2Deg(this->AzimuthalResolution) << "°" << std::endl;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["saliency"]       = get1DVector(this->Saliency);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["edge_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::EDGE);
  map["plane_validity"] = get1DVectorFromFlag(this->IsPointValid, Keypoint::PLANE);
  map["blob_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::BLOB);
  return map;
}

} // end of LidarSlam namespace