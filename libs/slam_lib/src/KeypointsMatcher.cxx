//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-10-16
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

#include "LidarSlam/KeypointsMatcher.h"
#include "LidarSlam/CeresCostFunctions.h"

namespace LidarSlam
{

//-----------------------------------------------------------------------------
KeypointsMatcher::KeypointsMatcher(const KeypointsMatcher::Parameters& params,
                                   const Eigen::Isometry3d& posePrior)
  : Params(params)
  , PosePrior(posePrior)
{}

//-----------------------------------------------------------------------------
KeypointsMatcher::MatchingResults KeypointsMatcher::BuildMatchResiduals(const PointCloud::Ptr& currPoints,
                                                                        const KDTree& prevPoints,
                                                                        Keypoint keypointType)
{
  // Call the correct point-to-neighborhood method
  auto BuildMatchResidual = [&](const Point& currentPoint)
  {
    switch(keypointType)
    {
      case Keypoint::EDGE:
        return this->BuildLineMatch(prevPoints, currentPoint);
      case Keypoint::INTENSITY_EDGE:
        return this->BuildLineMatch(prevPoints, currentPoint);
      case Keypoint::PLANE:
        return this->BuildPlaneMatch(prevPoints, currentPoint);
      case Keypoint::BLOB:
        return this->BuildBlobMatch(prevPoints, currentPoint);
      default:
        return MatchingResults::MatchInfo{ MatchingResults::MatchStatus::UNKOWN, 0., CeresTools::Residual() };
    }
  };

  // Reset matching results
  MatchingResults matchingResults;
  matchingResults.Reset(currPoints->size());

  // Loop over keypoints and try to build residuals
  if (!currPoints->empty() && prevPoints.GetInputCloud() && !prevPoints.GetInputCloud()->empty())
  {
    #pragma omp parallel for num_threads(this->Params.NbThreads) schedule(guided, 8)
    for (int ptIndex = 0; ptIndex < static_cast<int>(currPoints->size()); ++ptIndex)
    {
      const Point& currentPoint = currPoints->points[ptIndex];
      const auto& match = BuildMatchResidual(currentPoint);
      matchingResults.Rejections[ptIndex] = match.Status;
      matchingResults.Weights[ptIndex] = match.Weight;
      matchingResults.Residuals[ptIndex] = match.Cost;
      #pragma omp atomic
      matchingResults.RejectionsHistogram[match.Status]++;
    }
  }

  return matchingResults;
}


//----------------------------------------------------------------------------
CeresTools::Residual KeypointsMatcher::BuildResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double weight)
{
  CeresTools::Residual res;
  // Create the point-to-line/plane/blob cost function
  res.Cost = CeresCostFunctions::MahalanobisDistanceAffineIsometryResidual::Create(A, P, X);

  // Use a robustifier to limit the contribution of an outlier match
  // Tukey loss applied on residual square:
  //   rho(residual^2) = a^2 / 3 * ( 1 - (1 - residual^2 / a^2)^3 )   for residual^2 <= a^2,
  //   rho(residual^2) = a^2 / 3                                      for residual^2 >  a^2.
  // a is the scaling parameter of the function
  // See http://ceres-solver.org/nnls_modeling.html#theory for details
  auto* robustifier = new ceres::TukeyLoss(this->Params.SaturationDistance);

  // Weight the contribution of the given match by its reliability
  // WARNING : in CERES version < 2.0.0, the Tukey loss is badly implemented, so we have to correct the weight by a factor 2
  // See https://github.com/ceres-solver/ceres-solver/commit/6da364713f5b78ddf15b0e0ad92c76362c7c7683 for details
  // This is important for covariance scaling
  #if (CERES_VERSION_MAJOR < 2)
    res.Robustifier.reset(new ceres::ScaledLoss(robustifier, 2.0 * weight, ceres::TAKE_OWNERSHIP));
  // If Ceres version >= 2.0.0, the Tukey loss is corrected.
  #else
    res.Robustifier.reset(new ceres::ScaledLoss(robustifier, weight, ceres::TAKE_OWNERSHIP));
  #endif
  return res;
}

//-----------------------------------------------------------------------------
KeypointsMatcher::MatchingResults::MatchInfo KeypointsMatcher::BuildLineMatch(const KDTree& previousEdges, const Point& p)
{
  // At least 2 points are needed to fit a line model
  if (this->Params.EdgeNbNeighbors < 2 || this->Params.EdgeMinNbNeighbors < 2)
    return { MatchingResults::MatchStatus::BAD_MODEL_PARAMETRIZATION, 0., CeresTools::Residual() };

  // =====================================================
  // Transform the point using the current pose estimation

  // basePoint is the raw local position in BASE coordinates, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in WORLD coodinates, transformed using initial prior.
  Eigen::Vector3d basePoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * basePoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  if (this->Params.SingleEdgePerRing)
    this->GetPerRingLineNeighbors(previousEdges, worldPoint.data(), this->Params.EdgeNbNeighbors, knnIndices, knnSqDist);
  else
    this->GetRansacLineNeighbors(previousEdges, worldPoint.data(), this->Params.EdgeNbNeighbors, this->Params.EdgeMaxModelError, knnIndices, knnSqDist);

  // If not enough neighbors, abort
  unsigned int neighborhoodSize = knnIndices.size();
  if (neighborhoodSize < this->Params.EdgeMinNbNeighbors)
    return { MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS, 0., CeresTools::Residual() };

  // If the nearest edges are too far from the current edge keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxNeighborsDistance * this->Params.MaxNeighborsDistance)
    return { MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR, 0., CeresTools::Residual() };

  // =======================================================
  // Check if neighborhood is a good line candidate with PCA

  // Compute PCA to determine best line approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and keep it
  // if it is well distributed along a line.
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(*previousEdges.GetInputCloud(), knnIndices, mean, eigVecs, eigVals);

  // =============================================
  // Compute point-to-line optimization parameters

  // n is the director vector of the line
  const Eigen::Vector3d& n = eigVecs.col(2);

  // Compute the inverse squared out covariance matrix
  // of the target line model -> A = Covariance^(-1/2)
  // It is used to compute the Mahalanobis distance
  // The residual vector is A*(pt - mean)
  // NOTE : A^2 = (Id - n*n.t)^2 = Id - n*n.t
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - n * n.transpose();

  // =========================
  // Check parameters validity

  // It would be the case if P1 = P2, for instance if the sensor has some dual
  // returns that hit the same point.
  if (!std::isfinite(A(0, 0)))
    return { MatchingResults::MatchStatus::INVALID_NUMERICAL, 0., CeresTools::Residual() };

  // If the MSE is too high, the target model is not accurate enough, discard the match in optimization
  double mse = eigVals(0) + eigVals(1);
  if (mse >= std::pow(this->Params.EdgeMaxModelError, 2))
    return { MatchingResults::MatchStatus::MSE_TOO_LARGE, 0., CeresTools::Residual() };

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-line match
  // If the points to model error is too low, assign maximum weight.
  // Otherwise, assign a weight relative to the points to model error and a user parameter maximum value
  double fitQualityCoeff = (mse <= 1e-6) ? 1. : 1. - std::sqrt(mse) / this->Params.EdgeMaxModelError;

  CeresTools::Residual res = this->BuildResidual(A, mean, basePoint, fitQualityCoeff);
  return { MatchingResults::MatchStatus::SUCCESS, fitQualityCoeff, res };
}

//-----------------------------------------------------------------------------
KeypointsMatcher::MatchingResults::MatchInfo KeypointsMatcher::BuildPlaneMatch(const KDTree& previousPlanes, const Point& p)
{
  // At least 3 points are needed to fit a plane model
  if (this->Params.PlaneNbNeighbors < 3)
    return { MatchingResults::MatchStatus::BAD_MODEL_PARAMETRIZATION, 0., CeresTools::Residual() };

  // =====================================================
  // Transform the point using the current pose estimation

  // basePoint is the raw local position in BASE coordinates, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in WORLD coodinates, transformed using initial prior.
  Eigen::Vector3d basePoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * basePoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = previousPlanes.KnnSearch(worldPoint.data(), this->Params.PlaneNbNeighbors, knnIndices, knnSqDist);

  // It means that there is not enough keypoints in the neighborhood
  if (neighborhoodSize < this->Params.PlaneNbNeighbors)
    return { MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS, 0., CeresTools::Residual() };

  // If the nearest planar points are too far from the current keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxNeighborsDistance * this->Params.MaxNeighborsDistance)
    return { MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR, 0., CeresTools::Residual() };

  // ========================================================
  // Check if neighborhood is a good plane candidate with PCA

  // Compute PCA to determine best plane approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and keep it
  // if it is well distributed along a plane.
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(*previousPlanes.GetInputCloud(), knnIndices, mean, eigVecs, eigVals);

  // If the second eigen value is close to the highest one and bigger than the
  // smallest one, it means that the points are distributed along a plane.
  // Otherwise, discard this bad unstructured neighborhood.
  if (eigVals(1) / eigVals(2) < this->Params.PlanarityThreshold)
    return { MatchingResults::MatchStatus::BAD_PCA_STRUCTURE, 0., CeresTools::Residual() };

  // ==============================================
  // Compute point-to-plane optimization parameters

  // n is the normal vector of the plane
  const Eigen::Vector3d& n = eigVecs.col(0);

  // Compute the inverse squared out covariance matrix
  // of the target plane model -> A = Covariance^(-1/2)
  // It is used to compute the Mahalanobis distance
  // The residual vector is A*(pt - mean)
  // NOTE : A^2 = (n*n.t)^2 = n*n.t
  Eigen::Matrix3d A = n * n.transpose();

  // =========================
  // Check parameters validity

  // It would be the case if P1 = P2, P1 = P3 or P3 = P2, for instance if the
  // sensor has some dual returns that hit the same point.
  if (!std::isfinite(A(0, 0)))
    return { MatchingResults::MatchStatus::INVALID_NUMERICAL, 0., CeresTools::Residual() };

  // If the MSE is too high, the target model is not accurate enough, discard the match in optimization
  double mse = eigVals(0);
  if (mse >= std::pow(this->Params.PlaneMaxModelError, 2))
    return { MatchingResults::MatchStatus::MSE_TOO_LARGE, 0., CeresTools::Residual() };

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-plane match
  // If the points to model error is too low, assign maximum weight.
  // Otherwise, assign a weight relative to the points to model error and a user parameter maximum value
  double fitQualityCoeff = (mse <= 1e-6) ? 1. : 1. - std::sqrt(mse) / this->Params.PlaneMaxModelError;

  CeresTools::Residual res = this->BuildResidual(A, mean, basePoint, fitQualityCoeff);
  return { MatchingResults::MatchStatus::SUCCESS, fitQualityCoeff, res };
}

//-----------------------------------------------------------------------------
KeypointsMatcher::MatchingResults::MatchInfo KeypointsMatcher::BuildBlobMatch(const KDTree& previousBlobs, const Point& p)
{
  // At least 4 points are needed to fit an ellipsoid model
  if (this->Params.BlobNbNeighbors < 4)
    return { MatchingResults::MatchStatus::BAD_MODEL_PARAMETRIZATION, 0., CeresTools::Residual() };

  // =====================================================
  // Transform the point using the current pose estimation

  // basePoint is the raw local position in BASE coordinates, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in WORLD coodinates, transformed using initial prior.
  Eigen::Vector3d basePoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * basePoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = previousBlobs.KnnSearch(worldPoint.data(), this->Params.BlobNbNeighbors, knnIndices, knnSqDist);

  // It means that there is not enough keypoints in the neighborhood
  if (neighborhoodSize < this->Params.BlobNbNeighbors)
    return { MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS, 0., CeresTools::Residual() };

  // If the nearest blob points are too far from the current keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxNeighborsDistance * this->Params.MaxNeighborsDistance)
    return { MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR, 0., CeresTools::Residual() };

  // ======================================================
  // Compute point-to-blob optimization parameters with PCA

  // Compute PCA to determine best ellipsoid approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and tune a
  // distance function adapted to the distribution (Mahalanobis distance).
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(*previousBlobs.GetInputCloud(), knnIndices, mean, eigVecs, eigVals);

  // Check PCA structure
  if (eigVals(0) <= 0. || eigVals(1) <= 0.)
    return { MatchingResults::MatchStatus::BAD_PCA_STRUCTURE, 0., CeresTools::Residual()};

  // Compute the inverse squared out covariance matrix
  // of the target neighborhood -> A = Covariance^(-1/2)
  // It is used to compute the Mahalanobis distance
  // The residual vector is A*(pt - mean)
  Eigen::Vector3d eigValsSqrtInv = eigVals.array().rsqrt();
  Eigen::Matrix3d A = eigVecs * eigValsSqrtInv.asDiagonal() * eigVecs.transpose();

  // =========================
  // Check parameters validity

  // Check the determinant of the matrix
  // and check parameters validity:
  // It would be the case if P1 = P2, for instance if the sensor has some dual
  // returns that hit the same point.
  if (!std::isfinite(A(0, 0)) || !std::isfinite(eigValsSqrtInv.prod()))
    return { MatchingResults::MatchStatus::INVALID_NUMERICAL, 0., CeresTools::Residual() };

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-blob match
  // The aim is to prevent wrong matching pulling the pointcloud in a bad direction.
  double fitQualityCoeff = 1.0;
  CeresTools::Residual res = this->BuildResidual(A, mean, basePoint, fitQualityCoeff);
  return { MatchingResults::MatchStatus::SUCCESS, fitQualityCoeff, res };
}

//-----------------------------------------------------------------------------
void KeypointsMatcher::GetPerRingLineNeighbors(const KDTree& previousEdges, const double pos[3], unsigned int knearest,
                                               std::vector<int>& validKnnIndices, std::vector<float>& validKnnSqDist) const
{
  // Get nearest neighbors of the query point
  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = previousEdges.KnnSearch(pos, knearest, knnIndices, knnSqDist);

  // If empty neighborhood, return
  if (neighborhoodSize == 0)
    return;

  // Shortcut to keypoints cloud
  const PointCloud& previousEdgesPoints = *previousEdges.GetInputCloud();

  // Take the closest point
  const Point& closest = previousEdgesPoints[knnIndices[0]];
  int closestLaserId = static_cast<int>(closest.laser_id);

  // Get number of scan lines of this neighborhood
  int laserIdMin = std::numeric_limits<int>::max();
  int laserIdMax = std::numeric_limits<int>::min();
  for (unsigned int k = 0; k < neighborhoodSize; ++k)
  {
    int scanLine = previousEdgesPoints[knnIndices[k]].laser_id;
    laserIdMin = std::min(laserIdMin, scanLine);
    laserIdMax = std::max(laserIdMax, scanLine);
  }
  int nLasers = laserIdMax - laserIdMin + 1;

  // Invalid all points that are on the same scan line than the closest one
  std::vector<uint8_t> idAlreadyTook(nLasers, 0);
  idAlreadyTook[closestLaserId - laserIdMin] = 1;

  // Invalid all points from scan lines that are too far from the closest one
  const int maxScanLineDiff = 4;  // TODO : add parameter to discard too far laser rings
  for (int laserId = laserIdMin; laserId <= laserIdMax; ++laserId)
  {
    if (std::abs(closestLaserId - laserId) > maxScanLineDiff)
      idAlreadyTook[laserId - laserIdMin] = 1;
  }

  // Make a selection among the neighborhood of the query point.
  // We can only take one edge per scan line.
  validKnnIndices.clear();
  validKnnSqDist.clear();
  for (unsigned int k = 0; k < neighborhoodSize; ++k)
  {
    int scanLine = previousEdgesPoints[knnIndices[k]].laser_id - laserIdMin;
    if (!idAlreadyTook[scanLine])
    {
      idAlreadyTook[scanLine] = 1;
      validKnnIndices.push_back(knnIndices[k]);
      validKnnSqDist.push_back(knnSqDist[k]);
    }
  }
}

//-----------------------------------------------------------------------------
void KeypointsMatcher::GetRansacLineNeighbors(const KDTree& previousEdges, const double pos[3], unsigned int knearest, double maxDistInlier,
                                              std::vector<int>& validKnnIndices, std::vector<float>& validKnnSqDist) const
{
  // Get nearest neighbors of the query point
  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = previousEdges.KnnSearch(pos, knearest, knnIndices, knnSqDist);

  // If neighborhood contains less than 2 neighbors
  // no line can be fitted
  if (neighborhoodSize < 2)
    return;

  // Shortcut to keypoints cloud
  const PointCloud& previousEdgesPoints = *previousEdges.GetInputCloud();

  // To avoid square root when performing comparison
  const float squaredMaxDistInlier = maxDistInlier * maxDistInlier;

  // Take the closest point
  const Point& closest = previousEdgesPoints[knnIndices[0]];
  const auto P1 = closest.getVector3fMap();

  // Loop over neighbors of the neighborhood. For each of them, compute the line
  // between closest point and current point and compute the number of inliers
  // that fit this line.
  std::vector<std::vector<unsigned int>> inliersList;
  inliersList.reserve(neighborhoodSize - 1);
  for (unsigned int ptIndex = 1; ptIndex < neighborhoodSize; ++ptIndex)
  {
    // Fit line that links P1 and P2
    const auto P2 = previousEdgesPoints[knnIndices[ptIndex]].getVector3fMap();
    Eigen::Vector3f dir = (P2 - P1).normalized();

    // Compute number of inliers of this model
    std::vector<unsigned int> inlierIndex;
    for (unsigned int candidateIndex = 1; candidateIndex < neighborhoodSize; ++candidateIndex)
    {
      if (candidateIndex == ptIndex)
        inlierIndex.push_back(candidateIndex);
      else
      {
        const auto Pcdt = previousEdgesPoints[knnIndices[candidateIndex]].getVector3fMap();
        if (((Pcdt - P1).cross(dir)).squaredNorm() < squaredMaxDistInlier)
          inlierIndex.push_back(candidateIndex);
      }
    }
    inliersList.push_back(inlierIndex);
  }

  // Keep the line and its inliers with the most inliers.
  std::size_t maxInliers = 0;
  int indexMaxInliers = -1;
  for (unsigned int k = 0; k < inliersList.size(); ++k)
  {
    if (inliersList[k].size() > maxInliers)
    {
      maxInliers = inliersList[k].size();
      indexMaxInliers = k;
    }
  }

  // fill vectors
  validKnnIndices.clear(); validKnnIndices.reserve(inliersList[indexMaxInliers].size());
  validKnnSqDist.clear(); validKnnSqDist.reserve(inliersList[indexMaxInliers].size());
  validKnnIndices.push_back(knnIndices[0]);
  validKnnSqDist.push_back(knnSqDist[0]);
  for (unsigned int inlier: inliersList[indexMaxInliers])
  {
    validKnnIndices.push_back(knnIndices[inlier]);
    validKnnSqDist.push_back(knnSqDist[inlier]);
  }
}

} // end of LidarSlam namespace