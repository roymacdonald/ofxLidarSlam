//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-10-08
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

#include "LidarSlam/ExternalSensorManagers.h"
#include "LidarSlam/Enums.h"
#include "LidarSlam/State.h"

#include <list>
#include <g2o/core/sparse_optimizer.h>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{
/**
 * @brief The PoseGraphOptimizer class enable to optimize a trajectory given
 * some GPS anchor points.
 */
class PoseGraphOptimizer
{
public:
  PoseGraphOptimizer();
  void ResetGraph();

  //! Add new states from LiDAR SLAM to the graph
  void AddLidarStates(const std::list<LidarState>& newStates);

  //! Position (pose) of the sensor that produces the trajectory to optimize in GPS antenna cordinates.
  //! As only GPS position is used, only translation offset will be used.
  void AddExternalSensor(const Eigen::Isometry3d& BaseToSensorOffset, unsigned int index = 0);

  //! Compute optimized SLAM trajectory using ground-control points.
  bool Process(std::list<LidarState>& optimizedStates);

  //! Add a landmark as a new node in the graph
  void AddLandmark(const Eigen::Isometry3d& lmPose, unsigned int index, bool onlyPosition = false);

  //! Add a constraint from a landmark detection
  void AddLandmarkConstraint(int lidarIdx, int lmIdx, const ExternalSensors::LandmarkMeasurement& lm, bool onlyPosition = false);

  //! Add a constraint from a gps measure
  //! The offset allows to represent the GPS positions in the same world frame as Lidar poses
  //! This odometry reference frame minimize some angle errors
  void AddGpsConstraint(int lidarIdx, const ExternalSensors::GpsMeasurement& gpsMeas);

  //! Modify all Lidar poses to be in GPS reference frame
  //! Offset is the translation to perform to align trajectories (GPS/Lidar)
  //! GPS data are provided in an external global reference frame
  //! We can set the Lidar positions in this frame using the first corresponding GPS/Lidar poses
  //! As GPS does not provide orientation, we cannot set Lidar orientation in this frame
  //! so, a new vertex with the reference frame is created and the first Lidar pose orientation is freed
  //! to be modified so it fits the GPS data the best it can
  //! WARNING : if the trajectory is rectilinear, roll angle might be wrong after optimization
  void InitForGps(const Eigen::Vector3d& offset);

  SetMacro(SaveG2OFile, bool)

  GetMacro(G2OFileName, std::string)
  SetMacro(G2OFileName, const std::string&)

  SetMacro(NbIteration, int)

  GetMacro(Verbose, bool)
  SetMacro(Verbose, bool)

  GetMacro(FixFirst, bool)
  SetMacro(FixFirst, bool)

  GetMacro(FixLast, bool)
  SetMacro(FixLast, bool)

private:

  g2o::SparseOptimizer Optimizer;
  bool SaveG2OFile = false;
  std::string G2OFileName = "";
  int NbIteration = 200;
  bool Verbose = false;
  int ExtIdx = INT_MAX;
  // Boolean to decide whether to fix the first pose or not
  bool FixFirst = false;
  // Boolean to decide whether to fix the last pose or not
  bool FixLast = false;
  // Saturation distance to remove outliers (used in robustifier)
  float SaturationDistance = 5.f;

  // Linking between external sensor supplied indices and graph indices
  std::unordered_map<int, int> LMIndicesLinking;
};

} // end of LidarSlam namespace