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

#include "LidarSlam/LocalOptimizer.h"
#include "LidarSlam/CeresCostFunctions.h"

namespace LidarSlam
{

//----------------------------------------------------------------------------
// Set params
//----------------------------------------------------------------------------

void LocalOptimizer::SetTwoDMode(bool twoDMode)
{
  this->TwoDMode = twoDMode;
}

void LocalOptimizer::SetLMMaxIter(unsigned int maxIt)
{
  this->LMMaxIter = maxIt;
}

void LocalOptimizer::SetNbThreads(unsigned int nbThreads)
{
  this->NbThreads = nbThreads;
}

void LocalOptimizer::SetPosePrior(const Eigen::Isometry3d& posePrior)
{
  // Convert isometry to 6D state vector : X, Y, Z, rX, rY, rZ
  this->PoseArray = Utils::IsometryToXYZRPY(posePrior);
}

//----------------------------------------------------------------------------
// Set residuals
//----------------------------------------------------------------------------

void LocalOptimizer::AddResidual(const CeresTools::Residual& res)
{
  this->Residuals.push_back(res);
}

void LocalOptimizer::AddResiduals(const std::vector<CeresTools::Residual>& residuals)
{
  this->Residuals.insert(this->Residuals.end(), residuals.begin(), residuals.end());
}

void LocalOptimizer::Clear()
{
  this->Residuals.clear();
}

//----------------------------------------------------------------------------
// Run optimization
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
ceres::Solver::Summary LocalOptimizer::Solve()
{
  ceres::Problem::Options  option;
  option.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  option.cost_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  // Clear problem and add residuals to optimize
  this->Problem = std::make_unique<ceres::Problem>(option);
  for (const CeresTools::Residual& res : this->Residuals)
  {
    if (res.Cost)
      this->Problem->AddResidualBlock(res.Cost.get(), res.Robustifier.get(), this->PoseArray.data());
  }

  // If 2D mode is enabled, hold Z, rX and rY constant
  if (this->TwoDMode)
    this->Problem->SetParameterization(this->PoseArray.data(), new ceres::SubsetParameterization(6, {2, 3, 4}));

  // LM solver options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;  // TODO : try also DENSE_NORMAL_CHOLESKY or SPARSE_NORMAL_CHOLESKY
  options.max_num_iterations = this->LMMaxIter;
  options.num_threads = this->NbThreads;

  // Run optimization
  ceres::Solver::Summary summary;
  ceres::Solve(options, this->Problem.get(), &summary);
  return summary;
}

//----------------------------------------------------------------------------
Eigen::Isometry3d LocalOptimizer::GetOptimizedPose() const
{
  // Convert 6D state vector (X, Y, Z, rX, rY, rZ) to isometry
  return Utils::XYZRPYtoIsometry(this->PoseArray);
}

//----------------------------------------------------------------------------
LocalOptimizer::RegistrationError LocalOptimizer::EstimateRegistrationError()
{
  RegistrationError err;

  // Covariance computation options
  ceres::Covariance::Options covOptions;
  covOptions.apply_loss_function = true;
  covOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
  covOptions.null_space_rank = -1;
  covOptions.num_threads = this->NbThreads;

  // Computation of the variance-covariance matrix
  ceres::Covariance covarianceSolver(covOptions);
  std::vector<std::pair<const double*, const double*>> covarianceBlocks;
  const double* paramBlock = this->PoseArray.data();
  covarianceBlocks.emplace_back(paramBlock, paramBlock);
  covarianceSolver.Compute(covarianceBlocks, &(*this->Problem));
  covarianceSolver.GetCovarianceBlock(paramBlock, paramBlock, err.Covariance.data());

  // Estimate max position/orientation errors and directions from covariance
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3, 3>());
  err.PositionError = std::sqrt(eigPosition.eigenvalues()(2));
  err.PositionErrorDirection = eigPosition.eigenvectors().col(2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3, 3>());
  err.OrientationError = Utils::Rad2Deg(std::sqrt(eigOrientation.eigenvalues()(2)));
  err.OrientationErrorDirection = eigOrientation.eigenvectors().col(2);

  return err;
}

} // end of LidarSlam namespace