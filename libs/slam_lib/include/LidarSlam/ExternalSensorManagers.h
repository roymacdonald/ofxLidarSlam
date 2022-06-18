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

#pragma once

#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres
#include "LidarSlam/Utilities.h"
#include <list>
#include <cfloat>
#include <mutex>

namespace LidarSlam
{

#define SetSensorMacro(name,type) void Set##name (type _arg) { this->name = _arg; }
#define GetSensorMacro(name,type) type Get##name () const { return this->name; }

namespace ExternalSensors
{

// ---------------------------------------------------------------------------
struct LandmarkMeasurement
{
  double Time = 0.;
  // Relative transform between the tracking frame and the tag
  Eigen::Isometry3d TransfoRelative = Eigen::Isometry3d::Identity();
  Eigen::Matrix6d Covariance = Eigen::Matrix6d::Identity();
};

// ---------------------------------------------------------------------------
struct WheelOdomMeasurement
{
  double Time = 0.;
  double Distance = 0.;
};

// ---------------------------------------------------------------------------
struct GravityMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
};

// ---------------------------------------------------------------------------
template <typename T>
class SensorManager
{
public:
  SensorManager(const std::string& name = "BaseSensor")
  : SensorName(name), PreviousIt(Measures.begin()) {}

  SensorManager(double w, double timeOffset, double timeThreshold,
                unsigned int maxMeas, const std::string& name = "BaseSensor")
  : TimeOffset(timeOffset),
    Weight(w),
    TimeThreshold(timeThreshold),
    MaxMeasures(maxMeas),
    SensorName(name),
    PreviousIt(Measures.begin())
  {}

  // -----------------Setters/Getters-----------------
  GetSensorMacro(SensorName, std::string)
  SetSensorMacro(SensorName, std::string)

  GetSensorMacro(Weight, double)
  SetSensorMacro(Weight, double)

  GetSensorMacro(TimeOffset, double)
  SetSensorMacro(TimeOffset, double)

  GetSensorMacro(TimeThreshold, double)
  SetSensorMacro(TimeThreshold, double)

  GetSensorMacro(MaxMeasures, unsigned int)
  void SetMaxMeasures(unsigned int maxMeas)
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    this->MaxMeasures = maxMeas;
    while (this->Measures.size() > this->MaxMeasures)
    {
      if (this->PreviousIt == this->Measures.begin())
        ++this->PreviousIt;
      this->Measures.pop_front();
    }
  }

  std::list<T> GetMeasures() const
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Measures;
  }

  GetSensorMacro(Residual, CeresTools::Residual)

  // -----------------Basic functions-----------------

  // ------------------
  // Add one measure at a time in measures list
  void AddMeasurement(const T& m)
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    this->Measures.emplace_back(m);
    if (this->Measures.size() > this->MaxMeasures)
    {
      if (this->PreviousIt == this->Measures.begin())
        ++this->PreviousIt;
      this->Measures.pop_front();
    }
  }

  // ------------------
  void Reset()
  {
    this->ResetResidual();
    std::lock_guard<std::mutex> lock(this->Mtx);
    this->Measures.clear();
    this->PreviousIt = this->Measures.begin();
    this->TimeOffset = 0.;
  }

  // ------------------
  // Check if sensor can be used in optimization
  // The weight must be not null and the measures list must contain
  // at leat 2 elements to be able to interpolate
  bool CanBeUsed()
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Weight > 1e-6 && this->Measures.size() > 1;
  }

  // Compute the interpolated measure to be synchronised with SLAM output (at lidarTime)
  virtual bool ComputeSynchronizedMeasure(double lidarTime, T& synchMeas, bool verbose = false){return true;}
  // Compute the constraint associated to the measurement
  virtual bool ComputeConstraint(double lidarTime, bool verbose = false){return false;}

protected:
  // ------------------
  // Reset the current residual
  void ResetResidual()
  {
    this->Residual.Cost.reset();
    this->Residual.Robustifier.reset();
  }

  // ------------------
  std::pair<typename std::list<T>::iterator, typename std::list<T>::iterator> GetMeasureBounds(double lidarTime, bool verbose = false)
  {
    // Check if the measurements can be interpolated (or slightly extrapolated)
    if (lidarTime < this->Measures.front().Time || lidarTime > this->Measures.back().Time + this->TimeThreshold)
    {
      if (verbose)
        PRINT_INFO(std::fixed << std::setprecision(9)
                   << "\t Measures contained in : [" << this->Measures.front().Time << ","
                   << this->Measures.back().Time <<"]\n"
                   << "\t -> " << this->SensorName << " not used"
                   << std::scientific)
      return std::make_pair(this->Measures.begin(), this->Measures.begin());
    }

    // Reset if the timeline has been modified (and if there is memory of a previous pose)
    if (this->PreviousIt == this->Measures.end() || this->PreviousIt->Time > lidarTime)
      this->PreviousIt = this->Measures.begin();

    // Get iterator pointing to the first measurement after LiDAR time
    auto postIt = std::upper_bound(this->PreviousIt,
                                   this->Measures.end(),
                                   lidarTime,
                                   [&](double time, const T& measure) {return time < measure.Time;});

    // If the last measure was taken before Lidar points
    // extract the two last measures (for extrapolation)
    if (postIt == this->Measures.end())
      --postIt;

    // Get iterator pointing to the last measurement before LiDAR time
    auto preIt = postIt;
    --preIt;

    // Update the previous iterator for next call
    this->PreviousIt = preIt;

    // If the time between the 2 measurements is too long
    // Do not use the current measures
    if (postIt->Time - preIt->Time > this->TimeThreshold)
    {
      if (verbose)
        PRINT_INFO("\t The two last " << this->SensorName << " measures can not be interpolated (too much time difference)"
                   << "-> " << this->SensorName << " not used")
      return std::make_pair(this->Measures.begin(), this->Measures.begin());
    }
    return std::make_pair(preIt, postIt);
  }

protected:
  // Sensor name for output
  std::string SensorName;
  // Measures stored
  std::list<T> Measures;
  // Iterator pointing to the last measure used
  // This allows to keep a time track
  typename std::list<T>::iterator PreviousIt;
  // Measures length limit
  // The oldest measures are forgotten
  unsigned int MaxMeasures = 1e6;
  // Weight to apply to sensor constraint
  double Weight = 0.;
  // Time offset to make external sensors/Lidar correspondance
  double TimeOffset = 0.;
  // Time threshold between 2 measures to consider they can be interpolated
  double TimeThreshold = 0.5;
  // Resulting residual
  CeresTools::Residual Residual;
  // Mutex to handle the data from outside the library
  mutable std::mutex Mtx;
};

// ---------------------------------------------------------------------------
class WheelOdometryManager : public SensorManager<WheelOdomMeasurement>
{
public:
  WheelOdometryManager(const std::string& name = "Wheel odometer"): SensorManager(name){}
  //Setters/Getters
  GetSensorMacro(PreviousPose, Eigen::Isometry3d)
  SetSensorMacro(PreviousPose, const Eigen::Isometry3d&)

  GetSensorMacro(Relative, bool)
  SetSensorMacro(Relative, bool)

  GetSensorMacro(RefDistance, double)
  SetSensorMacro(RefDistance, double)

  // Compute the interpolated measure to be synchronised with SLAM output (at lidarTime)
  bool ComputeSynchronizedMeasure(double lidarTime, WheelOdomMeasurement& synchMeas, bool verbose = false);
  // Wheel odometry constraint (unoriented)
  // Can be relative since last frame or absolute since first pose
  bool ComputeConstraint(double lidarTime, bool verbose = false) override;

private:
  // Members used when using the relative distance with last estimated pose
  Eigen::Isometry3d PreviousPose = Eigen::Isometry3d::Identity();
  double RefDistance = FLT_MAX;
  // Boolean to indicate whether to compute an absolute constraint (since first frame)
  // or relative constraint (since last acquired frame)
  bool Relative = false;
};

// ---------------------------------------------------------------------------
class ImuManager : public SensorManager<GravityMeasurement>
{
public:
  ImuManager(const std::string& name = "IMU"): SensorManager(name){}
  //Setters/Getters
  GetSensorMacro(GravityRef, Eigen::Vector3d)
  SetSensorMacro(GravityRef, const Eigen::Vector3d&)

  // Compute the interpolated measure to be synchronised with SLAM output (at lidarTime)
  bool ComputeSynchronizedMeasure(double lidarTime, GravityMeasurement& synchMeas, bool verbose = false);

  // IMU constraint (gravity)
  bool ComputeConstraint(double lidarTime, bool verbose = false) override;
  // Compute Reference gravity vector from IMU measurements
  void ComputeGravityRef(double deltaAngle);

private:
  Eigen::Vector3d GravityRef = Eigen::Vector3d::Zero();
};

// ---------------------------------------------------------------------------
class LandmarkManager: public SensorManager<LandmarkMeasurement>
{
public:
  LandmarkManager(const std::string& name = "Tag detector") : SensorManager(name){}
  LandmarkManager(const LandmarkManager& lmManager);
  LandmarkManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                  double sat, bool positionOnly = true, const std::string& name = "Tag detector");

  void operator=(const LandmarkManager& lmManager);

  // Setters/Getters
  // The absolute pose can be set from outside the lib
  // or will be detected online, averaging the previous detections
  GetSensorMacro(AbsolutePose, Eigen::Vector6d)
  GetSensorMacro(AbsolutePoseCovariance, Eigen::Matrix6d)

  GetSensorMacro(SaturationDistance, float)
  SetSensorMacro(SaturationDistance, float)

  GetSensorMacro(PositionOnly, bool)
  SetSensorMacro(PositionOnly, bool)

  GetSensorMacro(CovarianceRotation, bool)
  SetSensorMacro(CovarianceRotation, bool)
  // Set the initial absolute pose
  // NOTE : the absolute pose can be updated if UpdateAbsolutePose is called
  void SetAbsolutePose(const Eigen::Vector6d& pose, const Eigen::Matrix6d& cov);

  // Compute the interpolated measure to be synchronised with SLAM output (at lidarTime)
  bool ComputeSynchronizedMeasure(double lidarTime, LandmarkMeasurement& synchMeas, bool verbose = false);

  // Landmark constraint
  bool ComputeConstraint(double lidarTime, bool verbose = false) override;

  // Update the absolute pose in case the tags are used as relative constraints
  // (i.e, no absolute poses of the tags are supplied)
  bool UpdateAbsolutePose(const Eigen::Isometry3d& baseTransform, double lidarTime);
  bool NeedsReferencePoseRefresh(double lidarTime);

private:
  bool HasBeenUsed(double lidarTime);

private:
  // Absolute pose of the landmark in the global frame
  Eigen::Vector6d AbsolutePose = Eigen::Vector6d::Zero();
  Eigen::Matrix6d AbsolutePoseCovariance = Eigen::Matrix6d::Zero();
  Eigen::Isometry3d RelativeTransform = Eigen::Isometry3d::Identity();
  // Boolean to check the absolute pose has been loaded
  // or if the tag has already been seen
  bool HasAbsolutePose = false;
  std::pair<double, double> LastUpdateTimes = {FLT_MAX, FLT_MAX};
  // Counter to check how many frames the tag was seen on
  // This is used to average the pose in case the absolute poses
  // were not supplied initially and are updated (cf. UpdateAbsolutePose)
  int Count = 0;
  // Threshold distance to not take into account the landmark constraint
  // (The tag may have been moved or the SLAM has drifted too much)
  // This distance is used in a robustifier to weight the landmark residuals
  float SaturationDistance = 5.f;
  // The constraint created can use the whole position (orientation + position) -> false
  // or only the position -> true (if the orientation is not reliable enough)
  bool PositionOnly = true;
  // Allow to rotate the covariance
  // Can be disabled if the covariance is fixed
  bool CovarianceRotation = false;
};

} // end of ExternalSensors namespace
} // end of LidarSlam namespace