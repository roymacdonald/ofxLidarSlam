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
  //! landmarkStates contains the relative poses between landmark (defined by the SensorState index) and
  //! the relative pose defined by the corresponding index in poseIndices
  void AddLandmarkConstraint(int lidarIdx, int lmIdx, const ExternalSensors::LandmarkMeasurement& lm, bool onlyPosition = false);

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
  int LandmarkIdx = INT_MAX;
  // Boolean to decide whether to fix the first pose or not
  bool FixFirst = false;
  // Boolean to decide whether to fix the last pose or not
  bool FixLast = false;

  // Linking between external sensor supplied indices and graph indices
  std::unordered_map<int, int> LMIndicesLinking;
};

} // end of LidarSlam namespace