//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-03-15
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

#include "LidarSlam/ExternalSensorManagers.h"

namespace LidarSlam
{
namespace ExternalSensors
{
// ---------------------------------------------------------------------------
 bool WheelOdometryManager::ComputeSynchronizedMeasure(double lidarTime, WheelOdomMeasurement& synchMeas, bool verbose)
{
  if (!this->CanBeUsed())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);

  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, verbose);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate odometry measurement at LiDAR timestamp
  synchMeas.Time = lidarTime;
  double rt = (lidarTime - bounds.first->Time) / (bounds.second->Time - bounds.first->Time);
  synchMeas.Distance = (1 - rt) * bounds.first->Distance + rt * bounds.second->Distance;

  return true;
}

// ---------------------------------------------------------------------------
bool WheelOdometryManager::ComputeConstraint(double lidarTime, bool verbose)
{
  this->ResetResidual();

  WheelOdomMeasurement synchMeas;
  if (!ComputeSynchronizedMeasure(lidarTime, synchMeas, verbose))
    return false;

  // Build odometry residual
  // If there is no memory of previous poses
  // The current measure is taken as reference
  if (this->RefDistance > 1e9)
  {
    if (verbose)
      PRINT_INFO("No previous wheel odometry measure : no constraint added to optimization")
    // Update reference distance for next frames
    this->RefDistance = synchMeas.Distance;
    return false;
  }

  // If there is memory of a previous pose
  double distDiff = std::abs(synchMeas.Distance - this->RefDistance);
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->PreviousPose.translation(), distDiff);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  if(verbose && !this->Relative)
    PRINT_INFO("Adding absolute wheel odometry residual : " << distDiff << " m travelled since first frame.")
  if (verbose && this->Relative)
    PRINT_INFO("Adding relative wheel odometry residual : " << distDiff << " m travelled since last frame.")

  // Update reference distance if relative mode enabled
  if (this->Relative)
    this->RefDistance = synchMeas.Distance;

  return true;
}

// ---------------------------------------------------------------------------
 bool ImuManager::ComputeSynchronizedMeasure(double lidarTime, GravityMeasurement& synchMeas, bool verbose)
{
  if (!this->CanBeUsed())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, verbose);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate gravity measurement at LiDAR timestamp
  synchMeas.Time = lidarTime;
  double rt = (lidarTime - bounds.first->Time) / (bounds.second->Time - bounds.first->Time);
  synchMeas.Acceleration = (1 - rt) * bounds.first->Acceleration.normalized() + rt * bounds.second->Acceleration.normalized();
  // Normalize interpolated gravity vector
  if (synchMeas.Acceleration.norm() > 1e-6) // Check to ensure consistent IMU measure
    synchMeas.Acceleration.normalize();
  else
    return false;

  return true;
}

// ---------------------------------------------------------------------------
bool ImuManager::ComputeConstraint(double lidarTime, bool verbose)
{
  this->ResetResidual();

  // Compute reference gravity vector
  if (this->GravityRef.norm() < 1e-6)
    this->ComputeGravityRef(Utils::Deg2Rad(5.f));

  GravityMeasurement synchMeas;
  if (!ComputeSynchronizedMeasure(lidarTime, synchMeas, verbose))
    return false;

  // Build gravity constraint
  this->Residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(this->GravityRef, synchMeas.Acceleration);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  PRINT_INFO("\t Adding gravity residual with gravity reference : " << this->GravityRef.transpose())
  return true;
}

// ---------------------------------------------------------------------------
void ImuManager::ComputeGravityRef(double deltaAngle)
{
  std::lock_guard<std::mutex> lock(this->Mtx);
  // Init histogram 2D (phi and theta)
  int NPhi = std::ceil(2 * M_PI / deltaAngle);
  int NTheta = std::ceil(M_PI / deltaAngle);
  std::vector<std::vector<std::vector<GravityMeasurement*>>> histogram(NPhi, std::vector<std::vector<GravityMeasurement*>>(NTheta));

  // Store acceleration vector indices in histogram
  for (auto& meas : this->Measures)
  {
    Eigen::Vector3d AccelDirection = meas.Acceleration.normalized();
    int idxPhi = ( std::atan2(AccelDirection.y(), AccelDirection.x()) + M_PI ) / deltaAngle;
    int idxTheta = ( std::acos(AccelDirection.z()) ) / deltaAngle;
    histogram[idxPhi][idxTheta].push_back(&meas);
  }
  // Get bin containing most points
  int bestPhi = 0;
  int bestTheta = 0;
  for (int idxPhi = 0; idxPhi < NPhi; ++idxPhi)
  {
    for (int idxTheta = 0; idxTheta < NTheta; ++idxTheta)
    {
      if (histogram[idxPhi][idxTheta].size() > histogram[bestPhi][bestTheta].size())
      {
        bestPhi = idxPhi;
        bestTheta = idxTheta;
      }
    }
  }

  // Compute mean of acceleration vectors in this bin
  this->GravityRef = Eigen::Vector3d::Zero();
  for (auto& itAcc : histogram[bestPhi][bestTheta])
    this->GravityRef += itAcc->Acceleration.normalized();
  this->GravityRef.normalize();
}

// ---------------------------------------------------------------------------
LandmarkManager::LandmarkManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                                 double sat, bool positionOnly, const std::string& name)
                : SensorManager(w, timeOffset, timeThresh, maxMeas, name),
                  SaturationDistance(sat),
                  PositionOnly(positionOnly),
                  CovarianceRotation(false)
{}

// ---------------------------------------------------------------------------
LandmarkManager::LandmarkManager(const LandmarkManager& lmManager)
                : LandmarkManager(lmManager.GetWeight(),
                                  lmManager.GetTimeOffset(),
                                  lmManager.GetTimeThreshold(),
                                  lmManager.GetMaxMeasures(),
                                  lmManager.GetSaturationDistance(),
                                  lmManager.GetPositionOnly(),
                                  lmManager.GetSensorName())
{
  this->CovarianceRotation = lmManager.GetCovarianceRotation();
  this->Measures = lmManager.GetMeasures();
  this->PreviousIt = this->Measures.begin();
}

// ---------------------------------------------------------------------------
void LandmarkManager::operator=(const LandmarkManager& lmManager)
{
  this->SensorName = lmManager.GetSensorName();
  this->Weight = lmManager.GetWeight();
  this->TimeOffset = lmManager.GetTimeOffset();
  this->TimeThreshold = lmManager.GetTimeThreshold();
  this->MaxMeasures = lmManager.GetMaxMeasures();
  this->SaturationDistance = lmManager.GetSaturationDistance();
  this->PositionOnly = lmManager.GetPositionOnly();
  this->CovarianceRotation = lmManager.GetCovarianceRotation();
  this->Measures = lmManager.GetMeasures();
  this->PreviousIt = this->Measures.begin();
}

// ---------------------------------------------------------------------------
void LandmarkManager::SetAbsolutePose(const Eigen::Vector6d& pose, const Eigen::Matrix6d& cov = Eigen::Matrix6d::Identity())
{
  this->AbsolutePose = pose;
  this->AbsolutePoseCovariance = cov;
  this->HasAbsolutePose = true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::HasBeenUsed(double lidarTime)
{
  // Absolute is used to discard the initial case LastUpdateTimes.second = infinity
  return std::abs(lidarTime - this->LastUpdateTimes.second) < 1e-6;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::NeedsReferencePoseRefresh(double lidarTime)
{
  return this->HasBeenUsed(lidarTime) &&
         (!this->HasAbsolutePose || this->LastUpdateTimes.second - this->LastUpdateTimes.first > this->TimeThreshold);
}

// ---------------------------------------------------------------------------
bool LandmarkManager::UpdateAbsolutePose(const Eigen::Isometry3d& baseTransform, double lidarTime)
{
  if (!this->HasBeenUsed(lidarTime))
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // If it is the first time the tag is detected
  // or if the last time the tag has been seen was long ago
  // (re)set the absolute pose using the current base transform and
  // the relative transform measured
  if (NeedsReferencePoseRefresh(lidarTime))
  {
    this->AbsolutePose = Utils::IsometryToXYZRPY(baseTransform * this->RelativeTransform);
    this->HasAbsolutePose = true;
    this->Count = 1;
  }
  // If it has already been seen, the absolute pose is updated averaging the computed poses
  else
  {
    Eigen::Vector6d newAbsolutePose = Utils::IsometryToXYZRPY(baseTransform * this->RelativeTransform);
    this->AbsolutePose = ( (this->AbsolutePose * this->Count) + newAbsolutePose ) / (this->Count + 1);
    ++this->Count;
  }
  return true;
}

// ---------------------------------------------------------------------------
 bool LandmarkManager::ComputeSynchronizedMeasure(double lidarTime, LandmarkMeasurement& synchMeas, bool verbose)
{
  if (!this->CanBeUsed())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, verbose);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate landmark relative pose at LiDAR timestamp
  this->RelativeTransform = LinearInterpolation(bounds.first->TransfoRelative, bounds.second->TransfoRelative, lidarTime, bounds.first->Time, bounds.second->Time);
  synchMeas.Time = lidarTime;
  synchMeas.TransfoRelative = this->RelativeTransform;
  // Rotate covariance if required
  if (this->CovarianceRotation)
  {
    Eigen::Isometry3d update = this->RelativeTransform * bounds.first->TransfoRelative.inverse();
    Eigen::Vector6d xyzrpy = Utils::IsometryToXYZRPY(bounds.first->TransfoRelative);
    synchMeas.Covariance = CeresTools::RotateCovariance(xyzrpy, bounds.first->Covariance, update.linear());
  }

  return true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::ComputeConstraint(double lidarTime, bool verbose)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return false;

  LandmarkMeasurement synchMeas;
  if (!ComputeSynchronizedMeasure(lidarTime, synchMeas, verbose))
    return false;

  this->RelativeTransform = synchMeas.TransfoRelative;

  // Last times the tag was used
  // this is used to update the absolute reference tag pose when the
  // sensor absolute pose will be estimated (if required).
  this->LastUpdateTimes.first = this->LastUpdateTimes.second;
  this->LastUpdateTimes.second = lidarTime;

  // Check if the absolute pose has been computed
  // If not, the next tag detection is waited
  if (!this->HasAbsolutePose)
  {
    PRINT_WARNING("\t No absolute pose, waiting for next detection")
    return false;
  }

  // Build constraint
  // NOTE : the covariances are not used because the uncertainty is not comparable with common keypoint constraints
  // The user must play with the weight parameter to get the best result depending on the tag detection accuracy.
  if (this->PositionOnly)
    this->Residual.Cost = CeresCostFunctions::LandmarkPositionResidual::Create(this->RelativeTransform, this->AbsolutePose);
  else
    this->Residual.Cost = CeresCostFunctions::LandmarkResidual::Create(this->RelativeTransform, this->AbsolutePose);
  // Use a robustifier to limit the contribution of an outlier tag detection (the tag may have been moved)
  // Tukey loss applied on residual square:
  //   rho(residual^2) = a^2 / 3 * ( 1 - (1 - residual^2 / a^2)^3 )   for residual^2 <= a^2,
  //   rho(residual^2) = a^2 / 3                                      for residual^2 >  a^2.
  // a is the scaling parameter of the function
  // See http://ceres-solver.org/nnls_modeling.html#theory for details
  auto* robustifier = new ceres::TukeyLoss(this->SaturationDistance);

  // Weight the contribution of the given match by its reliability
  // WARNING : in CERES version < 2.0.0, the Tukey loss is badly implemented, so we have to correct the weight by a factor 2
  // See https://github.com/ceres-solver/ceres-solver/commit/6da364713f5b78ddf15b0e0ad92c76362c7c7683 for details
  // This is important for covariance scaling
  #if (CERES_VERSION_MAJOR < 2)
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, 2.0 * this->Weight, ceres::TAKE_OWNERSHIP));
  // If Ceres version >= 2.0.0, the Tukey loss is corrected.
  #else
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, this->Weight, ceres::TAKE_OWNERSHIP));
  #endif

  return true;
}

} // end of ExternalSensors namespace
} // end of LidarSlam namespace