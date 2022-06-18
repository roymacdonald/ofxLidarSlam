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

#include "LidarSlam/CeresCostFunctions.h"
#include "LidarSlam/Utilities.h"
#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace LidarSlam
{
// Helper class to optimize the LidarSlam problem
class LocalOptimizer
{
public:

  //! Estimation of registration error
  struct RegistrationError
  {
    // Estimation of the maximum position error
    double PositionError = 0.;
    // Direction of the maximum position error
    Eigen::Vector3d PositionErrorDirection = Eigen::Vector3d::Zero();

    // Estimation of the maximum orientation error (in radians)
    double OrientationError = 0.;
    // Direction of the maximum orientation error
    Eigen::Vector3d OrientationErrorDirection = Eigen::Vector3d::Zero();

    // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
    // (DoF order : X, Y, Z, rX, rY, rZ)
    Eigen::Matrix6d Covariance = Eigen::Matrix6d::Zero();
  };

  //----------------------------------------------------------------------------
  // Optimize 2D state (X, Y, rZ) only
  void SetTwoDMode(bool twoDMode);

  // Set Levenberg Marquardt maximum number of iterations
  void SetLMMaxIter(unsigned int maxIt);

  // Set number of threads
  void SetNbThreads(unsigned int nbThreads);

  // Set prior pose
  void SetPosePrior(const Eigen::Isometry3d& posePrior);

  //----------------------------------------------------------------------------

  // Add sensor residuals (from Lidar or other
  // external sensor) to residuals vector
  void AddResidual(const CeresTools::Residual& res);
  void AddResiduals(const std::vector<CeresTools::Residual>& residuals);

  // Clear all residuals
  void Clear();

  // Build and optimize the Ceres problem
  ceres::Solver::Summary Solve();

  // Get optimization results
  Eigen::Isometry3d GetOptimizedPose() const;

  // Estimate registration error
  RegistrationError EstimateRegistrationError();

  //----------------------------------------------------------------------------
private:

  // Optimize 2D pose only.
  // This will only optimize X, Y (ground coordinates) and yaw (rZ).
  // This will hold Z (elevation), rX (roll) and rY (pitch) constant.
  bool TwoDMode = false;

  // Max number of threads to use to parallelize computations
  unsigned int NbThreads = 1;

  // Maximum number of iteration
  unsigned int LMMaxIter = 15;

  // DoF to optimize (= output)
  Eigen::Vector6d PoseArray;  ///< Pose parameters to optimize (XYZRPY)

  // Residuals vector
  // These residuals must involve the full 6D pose array (X, Y, Z, rX, rY, rZ)
  std::vector<CeresTools::Residual> Residuals;

  // The Ceres problem to optimize
  std::unique_ptr<ceres::Problem> Problem;
};

} // end of LidarSlam namespace