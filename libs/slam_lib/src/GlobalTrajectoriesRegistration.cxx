//==============================================================================
// Copyright 2019-2020 Kitware, Inc.
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-14
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

#include "LidarSlam/GlobalTrajectoriesRegistration.h"
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{

//------------------------------------------------------------------------------
bool GlobalTrajectoriesRegistration::ComputeTransformOffset(const std::vector<Transform>& initPoses,
                                                            const std::vector<Transform>& finalPoses,
                                                            Eigen::Isometry3d& initToFinal) const
{
  // TODO Use timestamps to filter out outliers points

  unsigned int nbInitPoses = initPoses.size();
  unsigned int nbFinalPoses = finalPoses.size();

  // Check input vector sizes (at least 2 elements)
  if ((nbInitPoses < 2) || (nbFinalPoses < 2))
  {
    PRINT_ERROR("Init and Final trajectories must have at least 2 points "
                "(Got " << nbInitPoses << " Init points and " << nbFinalPoses << " Final points).");
    return false;
  }

  // Compute rough transformation to get better initialization if needed
  Eigen::Isometry3d initToRough = Eigen::Isometry3d::Identity();
  if (this->InitWithRoughEstimate)
    this->ComputeRoughTransformOffset(initPoses, finalPoses, initToRough);

  // Convert to PCL pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const Transform& pose: initPoses)
    fromCloud->push_back(pcl::PointXYZ(pose.x(), pose.y(), pose.z()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const Transform& pose: finalPoses)
    toCloud->push_back(pcl::PointXYZ(pose.x(), pose.y(), pose.z()));

  // It is better (faster, more accurate and correct score estimation) to fit a
  // sparser trajectory to a denser one. As a result, if 'finalPoses' is sparser
  // than 'initPoses', swap source and target for ICP.
  bool swapInitAndFinal = nbInitPoses > nbFinalPoses;
  if (swapInitAndFinal)
  {
    if (this->Verbose)
      std::cout << "Swaping Init and Final trajectories." << std::endl;
    std::swap(fromCloud, toCloud);
    initToRough = initToRough.inverse();
  }

  // Run ICP for transform refinement
  pcl::PointCloud<pcl::PointXYZ> optimCloud;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
  icp.setMaximumIterations(this->NbrIcpIterations);
  icp.setInputSource(fromCloud);
  icp.setInputTarget(toCloud);
  icp.align(optimCloud, initToRough.matrix());
  initToFinal = icp.getFinalTransformation();

  // Swap back init and final trajectories.
  if (swapInitAndFinal)
  {
    initToRough = initToRough.inverse();
    initToFinal = initToFinal.inverse();
  }

  // DEBUG If requested, impose no roll angle
  if (this->NoRoll)
  {
    // Eigen::Vector3d rpy = initToFinal.rotation().eulerAngles(0, 1, 2);
    // rpy(0) = 0.;
    // initToFinal.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
    //                                 * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
    //                                 * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d rpy = initToFinal.inverse().linear().eulerAngles(0, 1, 2);
    initToFinal = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * initToFinal;
  }

  // Print estimated transforms
  if (this->Verbose)
  {
    std::cout << "ICP has converged : "  << icp.hasConverged()
              << "\nICP loss : "         << icp.getFitnessScore()
              << "\nRough transform :\n" << initToRough.matrix()
              << "\nICP transform :\n"   << (initToRough.inverse() * initToFinal).matrix()
              << "\nFinal transform :\n" << initToFinal.matrix() << std::endl;
  }

  return icp.hasConverged();
}

//------------------------------------------------------------------------------
bool GlobalTrajectoriesRegistration::ComputeRoughTransformOffset(const std::vector<Transform>& initPoses,
                                                                 const std::vector<Transform>& finalPoses,
                                                                 Eigen::Isometry3d& initToFinal)
{
  Eigen::Translation3d translation = ComputeRoughTranslationOffset(initPoses[0], finalPoses[0]);
  Eigen::Quaterniond rotation = ComputeRoughRotationOffset(initPoses[0], initPoses.back(),
                                                           finalPoses[0], finalPoses.back());
  initToFinal = translation * rotation;
  return true;
}

//------------------------------------------------------------------------------
Eigen::Translation3d GlobalTrajectoriesRegistration::ComputeRoughTranslationOffset(const Transform& initPose,
                                                                                   const Transform& finalPose)
{
  return Eigen::Translation3d(finalPose.GetPosition() - initPose.GetPosition());
}

//------------------------------------------------------------------------------
Eigen::Quaterniond GlobalTrajectoriesRegistration::ComputeRoughRotationOffset(const Transform& initPoseFrom,
                                                                              const Transform& initPoseTo,
                                                                              const Transform& finalPoseFrom,
                                                                              const Transform& finalPoseTo)
{
  // Get approximate initPose direction
  Eigen::Vector3d initDirection = initPoseTo.GetPosition() - initPoseFrom.GetPosition();
  // Get approximate finalPose direction
  Eigen::Vector3d finalDirection = finalPoseTo.GetPosition() - finalPoseFrom.GetPosition();
  // Compute orientation alignment between two sub-trajectories
  return Eigen::Quaterniond::FromTwoVectors(initDirection, finalDirection);
}

} // end of LidarSlam namespace