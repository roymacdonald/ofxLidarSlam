//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-03-01
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

#include "LidarSlam/KDTreePCLAdaptor.h"
#include "LidarSlam/CeresCostFunctions.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/Utilities.h"
#include "LidarSlam/Enums.h"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>

namespace LidarSlam
{

// Helper class to match edge/planar/blob keypoints and build ceres residuals
class KeypointsMatcher
{
public:
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;
  using KDTree = KDTreePCLAdaptor<Point>;

  //! Structure to easily set all matching parameters
  struct Parameters
  {
    // Max number of threads to use to parallelize computations
    unsigned int NbThreads = 1;

    // When searching edge keypoint nearest neighbors, we can follow different
    // strategies to keep only relevant matches, instead of taking all k-nearest points.
    // If false, the method GetRansacLineNeighbors() will be used.
    // If true, the method GetPerRingLineNeighbors() will be used.
    bool SingleEdgePerRing = false;

    // [m] The max distance allowed between a current keypoint and its neighbors.
    // If one of the neighbors is farther, the neighborhood will be rejected.
    double MaxNeighborsDistance = 5.;

    // Edge keypoints matching: point-to-line distance
    unsigned int EdgeNbNeighbors = 10;   ///< [>=2] Initial number of edge neighbors to extract, that will be filtered out to keep best candidates
    unsigned int EdgeMinNbNeighbors = 4; ///< [>=2] Min number of resulting filtered edge neighbors to approximate the corresponding line model
    double EdgeMaxModelError = 0.2;      ///< [m] Max RMSE allowed between neighborhood and its fitted line model

    // Plane keypoints matching: point-to-plane distance
    unsigned int PlaneNbNeighbors = 5;   ///< [>=3] Number of plane neighbors to extract to approximate the corresponding plane model
    double PlanarityThreshold = 0.04;    ///< Threshold to filter reliable planes : l1/l2 > PlanarityThreshold (warning l = std^2)
    double PlaneMaxModelError = 0.2;     ///< [m] Max RMSE allowed between neighborhood and its fitted plane model

    // Blob keypoints matching: point-to-ellipsoid distance
    unsigned int BlobNbNeighbors = 10;   ///< [>=4] Number of blob neighbors to extract to approximate the corresponding ellipsoid model

    // [m] Maximum distance beyond which the residual errors are
    // saturated to robustify the optimization against outlier constraints.
    // The residuals will be robustified by Tukey loss at scale SatDist,
    // leading to 50% of saturation at SatDist/2, fully saturated at SatDist.
    double SaturationDistance = 1.;
  };

  //! Result of matching for one set of keypoints
  struct MatchingResults
  {
    //! Result of the keypoint matching, explaining rejection cause of matching failure.
    enum MatchStatus : uint8_t
    {
      SUCCESS = 0,                ///< Keypoint has been successfully matched
      BAD_MODEL_PARAMETRIZATION,  ///< Not enough neighbors requested to build the target model
      NOT_ENOUGH_NEIGHBORS,       ///< Not enough neighbors found to match keypoint
      NEIGHBORS_TOO_FAR,          ///< Neighbors are too far to match keypoint
      BAD_PCA_STRUCTURE,          ///< PCA eigenvalues analysis discards neighborhood fit to model
      INVALID_NUMERICAL,          ///< Optimization parameter computation has numerical invalidity
      MSE_TOO_LARGE,              ///< Mean squared error to model is too important to accept fitted model
      UNKOWN,                     ///< Unkown status (matching probably not performed yet)
      nStatus
    };

    //! Match status and quality weight of each keypoint
    struct MatchInfo
    {
      MatchStatus Status;
      double Weight;
      CeresTools::Residual Cost;
    };

    // Vector of residual functions to add to ceres problem
    std::vector<CeresTools::Residual> Residuals;

    // Matching result of each keypoint
    std::vector<MatchStatus> Rejections;
    std::vector<double> Weights;
    // Histogram of the matching rejection causes
    std::array<int, MatchStatus::nStatus> RejectionsHistogram = {};

    // Number of successful matches (shortcut to RejectionsHistogram[SUCCESS])
    unsigned int NbMatches() const { return this->RejectionsHistogram[SUCCESS]; }

    void Reset(const unsigned int N)
    {
      this->Weights.assign(N, 0.);
      this->Rejections.assign(N, MatchingResults::MatchStatus::UNKOWN);
      this->RejectionsHistogram.fill(0);
      this->Residuals.assign(N, CeresTools::Residual());
    }
  };

  //----------------------------------------------------------------------------

  // Init matcher
  // It needs matching parameters and the prior transform to apply to keypoints
  KeypointsMatcher(const Parameters& params, const Eigen::Isometry3d& posePrior);

  // Point-to-neighborhood matching parameters.
  // The goal will be to loop over all keypoints, and to build the corresponding
  // point-to-neighborhood residuals that will be optimized later.
  // For each source keypoint, the steps will be:
  // - To extract the N nearest neighbors from the target cloud.
  //   These neighbors should not be too far from the source keypoint.
  // - Assess the neighborhood shape by checking its PCA eigenvalues.
  // - Fit a line/plane/blob model on the neighborhood using PCA.
  // - Assess the model quality by checking its error relatively to the neighborhood.
  // - Build the corresponding point-to-model distance operator
  // If any of these steps fail, the matching procedure of the current keypoint aborts.
  MatchingResults BuildMatchResiduals(const PointCloud::Ptr& currPoints,
                                      const KDTree& prevPoints,
                                      Keypoint keypointType);

  //----------------------------------------------------------------------------

private:

  // Build ICP match residual functions.
  // To recover the motion, we have to minimize the function
  //   f(R, T) = sum(d(edge_kpt, line)^2) + sum(d(plane_kpt, plane)^2) + sum(d(blob_kpt, blob)^2)
  // In all cases, the squared Mahalanobis distance between the keypoint and the line/plane/blob can be written:
  //   (R * X + T - P).t * A.t * A * (R * X + T - P)
  // Where:
  // - (R, T) is the rigid transform to optimize (transform from WORLD to BASE)
  // - X is the keypoint in BASE coordinates
  // - P is the centroid of the line/plane/blob neighborhood, in WORLD coordinates
  // - A is the distance operator:
  //    * A = (I - u*u.t) for a line with u being the unit tangent vector of the line.
  //    * A = (n*n.t) for a plane with n being its normal.
  //    * A = C^{-1/2} is the squared information matrix, aka stiffness matrix, where
  //      C is the covariance matrix encoding the shape of the neighborhood for a blob.
  // - weight attenuates the distance function for outliers
  CeresTools::Residual BuildResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double weight = 1.);

  // Match the current keypoint with its neighborhood in the map / previous
  MatchingResults::MatchInfo BuildLineMatch(const KDTree& previousEdges, const Point& p);
  MatchingResults::MatchInfo BuildPlaneMatch(const KDTree& previousPlanes, const Point& p);
  MatchingResults::MatchInfo BuildBlobMatch(const KDTree& previousBlobs, const Point& p);

  // Instead of taking the k-nearest neigbors we will take specific neighbor
  // using the particularities of the lidar sensor
  void GetPerRingLineNeighbors(const KDTree& previousEdges, const double pos[3],
                               unsigned int knearest, std::vector<int>& validKnnIndices,
                               std::vector<float>& validKnnSqDist) const;

  // Instead of taking the k-nearest neighbors we will take specific neighbor
  // using a sample consensus model
  void GetRansacLineNeighbors(const KDTree& previousEdges, const double pos[3],
                              unsigned int knearest, double maxDistInlier,
                              std::vector<int>& validKnnIndices,
                              std::vector<float>& validKnnSqDist) const;

  //----------------------------------------------------------------------------

private:

  // Matching parameters
  const Parameters Params;

  // Initial guess of the pose to optimize
  const Eigen::Isometry3d PosePrior;
};

} // end of LidarSlam namespace