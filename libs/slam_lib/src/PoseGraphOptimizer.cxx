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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/PoseGraphOptimizer.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/edge_se3_euler.h>

namespace LidarSlam
{
//------------------------------------------------------------------------------
PoseGraphOptimizer::PoseGraphOptimizer()
{
  // Create optimizer
  auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
  this->Optimizer.setAlgorithm(solver);
  this->Optimizer.setVerbose(this->Verbose);
}

//------------------------------------------------------------------------------
void PoseGraphOptimizer::ResetGraph()
{
  this->Optimizer.clear();
  this->LMIndicesLinking.clear();
  this->LandmarkIdx = INT_MAX;
}

//------------------------------------------------------------------------------
void PoseGraphOptimizer::AddExternalSensor(const Eigen::Isometry3d& BaseToSensorOffset, unsigned int index)
{
  // Check if the offset has never been set
  if (this->Optimizer.parameter(index))
  {
    PRINT_WARNING("Sensor pose could not be set : index already exists");
    return;
  }

  // Add base/sensor offset parameter
  auto* BaseToSensorSE3Offset = new g2o::ParameterSE3Offset;
  BaseToSensorSE3Offset->setId(index);
  this->Optimizer.addParameter(BaseToSensorSE3Offset);
  BaseToSensorSE3Offset->setOffset(BaseToSensorOffset.inverse());
}

//------------------------------------------------------------------------------
void PoseGraphOptimizer::AddLandmark(const Eigen::Isometry3d& lm, unsigned int index, bool onlyPosition)
{
  // If this landmark was already added to the process, return
  if (this->LMIndicesLinking.count(index))
    return;
  // Add landmark as a new vertex
  // Choose index of the new landmark -> output index in decreasing order since INT_MAX
  // so the input index does not overwrite a pose vertex index
  // and pose vertices are in correct order
  int idx = --this->LandmarkIdx;
  this->LMIndicesLinking[index] = idx;
  if (onlyPosition)
  {
    auto* landmarkVertex = new g2o::VertexPointXYZ;
    // Use reverse Id (from INT_MAX) to let pose indices in arrival order
    landmarkVertex->setId(idx);
    landmarkVertex->setEstimate(lm.translation());
    landmarkVertex->setFixed(true);
    this->Optimizer.addVertex(landmarkVertex);
  }
  else
  {
    auto* landmarkVertex = new g2o::VertexSE3;
    // Use reverse Id (from INT_MAX) to let pose indices in arrival order
    landmarkVertex->setId(idx);
    landmarkVertex->setEstimate(lm);
    landmarkVertex->setFixed(true);
    this->Optimizer.addVertex(landmarkVertex);
  }
}

//------------------------------------------------------------------------------
void PoseGraphOptimizer::AddLidarStates(const std::list<LidarState>& states)
{
  if (states.empty())
    return;

  // Extract last keystate
  unsigned int lastIndex = states.back().Index;
  auto it = states.end();
  --it;
  while (it != states.begin() && !it->IsKeyFrame)
    --it;
  lastIndex = it->Index;

  // Initialize local values
  int prevIdx = 0;
  Eigen::Isometry3d prevState;
  int nStates = 0;
  for (const auto& state : states)
  {
    if (!state.IsKeyFrame)
      continue;

    // Add new state as a new vertex
    auto* newVertex = new g2o::VertexSE3;
    newVertex->setId(state.Index);
    newVertex->setEstimate(state.Isometry);
    bool fixedVertex = (this->FixFirst && state.Index == states.front().Index) ||
                       (this->FixLast  && state.Index == lastIndex);

    newVertex->setFixed(fixedVertex);
    this->Optimizer.addVertex(newVertex);
    ++nStates;

    // If first pose, no relative constraint is added
    // Just update the local values
    if (state.Index == states.front().Index)
    {
      prevIdx   = state.Index;
      prevState = state.Isometry;
      continue;
    }

    // Add an edge with the relative transform
    // Information is given with Euler angle (order R = Rz * Ry * Rx)
    // G2o only proposes this Euler format
    auto* newEdge = new g2o::EdgeSE3Euler;
    // Set vertices
    newEdge->setVertex(0, this->Optimizer.vertex(prevIdx));
    newEdge->setVertex(1, this->Optimizer.vertex(state.Index));
    // Get inverse of last frame
    Eigen::Isometry3d lastFrameInv = prevState.inverse();
    // Compute relative transform with new frame
    Eigen::Isometry3d Trelative = lastFrameInv * state.Isometry;
    // Rotate covariance
    // Lidar Slam gives the covariance expressed in the map frame
    // We want the covariance expressed in the last frame to be consistent with supplied relative transform
    Eigen::Vector6d xyzrpy = Utils::IsometryToXYZRPY(state.Isometry);
    Eigen::Matrix6d covariance = CeresTools::RotateCovariance(xyzrpy, state.Covariance, lastFrameInv.linear());
    // Use g2o read function to transform Euler covariance into quaternion covariance
    // This function takes an istream as input
    // It needs the measurement vector as 6D euler pose in addition to the covariance
    // Luckily, g2o uses the same convention as SLAM lib (RPY)
    std::stringstream measureInfo;
    Eigen::Vector6d poseRelative = Utils::IsometryToXYZRPY(Trelative);
    measureInfo << poseRelative(0) << " " << poseRelative(1) << " " << poseRelative(2) << " "
                << poseRelative(3) << " " << poseRelative(4) << " " << poseRelative(5) << " ";
    Eigen::Matrix6d information = covariance.inverse();
    for (int i = 0; i < 6; ++i)
    {
      for (int j = i; j < 6; ++j)
        measureInfo << information(i, j) << " ";
    }
    newEdge->read(measureInfo);
    // Add edge
    this->Optimizer.addEdge(newEdge);
    // Update local values
    prevIdx = state.Index;
    prevState = state.Isometry;
  }
  if (this->Verbose)
    PRINT_INFO(nStates << " lidar states added to the graph");
}

//------------------------------------------------------------------------------
void PoseGraphOptimizer::AddLandmarkConstraint(int lidarIdx, int lmIdx, const ExternalSensors::LandmarkMeasurement& lm, bool onlyPosition)
{
  // Add an edge between a SLAM pose vertex and a landmark vertex
  if (onlyPosition)
  {
    auto* externalEdge = new g2o::EdgeSE3PointXYZ;
    externalEdge->setVertex(0, this->Optimizer.vertex(lidarIdx));
    externalEdge->setVertex(1, this->Optimizer.vertex(this->LMIndicesLinking[lmIdx]));

    externalEdge->setMeasurement(lm.TransfoRelative.translation());
    externalEdge->setInformation(lm.Covariance.block(0, 0, 3, 3).inverse());
    // Add offset transformation reference Id
    // Useless here but compulsory in G2o
    externalEdge->setParameterId(0, 0);
    // Add edge
    if (!this->Optimizer.addEdge(externalEdge))
      PRINT_ERROR("Tag constraint could not be added to the graph")
  }
  else
  {
    // It must be of Euler type to handle RPY/Euler conversions
    auto* externalEdge = new g2o::EdgeSE3Euler;
    externalEdge->setVertex(0, this->Optimizer.vertex(lidarIdx));
    externalEdge->setVertex(1, this->Optimizer.vertex(this->LMIndicesLinking[lmIdx]));
    // Use g2o read function to transform Euler covariance into quaternion covariance
    // This function takes an istream as input
    // It needs the measurement vector as 6D euler pose in addition to the covariance
    // Luckily, g2o uses the same convention as SLAM lib (RPY)
    std::stringstream measureInfo;
    Eigen::Vector6d poseRelative = Utils::IsometryToXYZRPY(lm.TransfoRelative);
    measureInfo << poseRelative(0) << poseRelative(1) << poseRelative(2)
                << poseRelative(3) << poseRelative(4) << poseRelative(5);
    Eigen::Matrix6d information = lm.Covariance.inverse();
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        measureInfo << information(i, j);
    }
    externalEdge->read(measureInfo);
    // Add offset transformation reference Id
    externalEdge->setParameterId(0, 0);
    // Add edge
    if (!this->Optimizer.addEdge(externalEdge))
      PRINT_ERROR("Tag constraint could not be added to the graph")
  }
  if (this->Verbose)
    PRINT_INFO("Add landmark constraint between state #" << lidarIdx <<" and tag #"<< lmIdx << " (i.e. vertex #" << this->LMIndicesLinking[lmIdx] << ")");
}

//------------------------------------------------------------------------------
bool PoseGraphOptimizer::Process(std::list<LidarState>& statesToOptimize)
{
  // Save Graph before optimization
  if (this->SaveG2OFile)
  {
    if (!this->G2OFileName.empty())
      this->Optimizer.save(this->G2OFileName.c_str());
    else
      PRINT_WARNING("Could not save the g2o graph. Please specify a filename.");
  }

  // Print debug info
  if (this->Verbose)
  {
    PRINT_INFO("\nThe Graph is composed of:\n"
               << "\t" << this->Optimizer.vertices().size() << " vertices\n"
               << "\t" << this->Optimizer.edges().size()    << " edges\n");
  }

  // Optimize the graph
  if (!this->Optimizer.initializeOptimization())
  {
    PRINT_ERROR("Pose graph initialization failed !");
    return false;
  }

  int iterations = this->Optimizer.optimize(this->NbIteration);

  // Print debug info if needed
  if (this->Verbose)
    PRINT_INFO("Pose graph optimization succeeded in " << iterations << " iterations.\n");

  // Set the output optimized data
  for (auto& state : statesToOptimize)
  {
    if (!state.IsKeyFrame)
      continue;
    // Get optimized SLAM vertex pose
    auto* v = this->Optimizer.vertex(state.Index);
    g2o::VertexSE3* vSE3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (!vSE3)
    {
      PRINT_ERROR("Error: could not cast the vertex")
      continue;
    }
    // Fill new optimized trajectory
    // The new covariances can not be reached -> uncertainty() getter was removed from g2o
    // see https://answers.ros.org/question/39175/retrieving-the-uncertainty-of-a-vertex-in-g2o/ for more details
    // one will not be able to optimize again the poses outside this library.
    state.Isometry = vSE3->estimate();
  }

  return true;
}

} // end of LidarSlam namespace