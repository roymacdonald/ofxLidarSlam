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

#pragma once

#include "LidarSlam/Transform.h"
#include <pcl/registration/icp.h>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

/**
 * @brief Find the global transform between two trajectories with ICP.
 * 
 * TODO Add optional constraint to get global transform parallel to output XY-plane.
 */
class GlobalTrajectoriesRegistration
{
public:

  //----------------------------------------------------------------------------

  SetMacro(NbrIcpIterations, unsigned int)

  SetMacro(InitWithRoughEstimate, bool)

  SetMacro(NoRoll, bool)

  SetMacro(Verbose, bool)

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute global translation/rotation offset between two trajectories.
   * @param[in]  initPoses   The initial trajectory (only positions will be used).
   * @param[in]  finalPoses  The final trajectory (only positions will be used).
   * @param[out] initToFinal The global transform to apply to 'initPoses' to get 'finalPoses'.
   * @return true if initToFinal has been computed correctly, false if error occured.
   * 
   * NOTE 1 : The returned transform is the one to transform each initPose to a finalPose such that :
   *          finalPose = initToFinal * initPose
   *
   * NOTE 2 : Matching is done only based on positions, orientations are ignored.
   */
  bool ComputeTransformOffset(const std::vector<Transform>& initPoses,
                              const std::vector<Transform>& finalPoses,
                              Eigen::Isometry3d& initToFinal) const;

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute approximate global translation/rotation offset between two
   *        trajectories using only first and last points.
   * @param[in]  initPoses   The initial trajectory (only positions will be used).
   * @param[in]  finalPoses  The final trajectory (only positions will be used).
   * @param[out] initToFinal The global transform to apply to 'initPoses' to get 'finalPoses'.
   * @return true if initToFinal has been computed correctly, false if error occured.
   * 
   * NOTE : The returned transform is the one to transform each initPose to a finalPose such that :
   *        finalPose = initToFinal * initPose
   */
  static bool ComputeRoughTransformOffset(const std::vector<Transform>& initPoses,
                                          const std::vector<Transform>& finalPoses,
                                          Eigen::Isometry3d& initToFinal);

private:

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute translation offset between two poses.
   * @param[in] initPose  The initial point.
   * @param[in] finalPose The final point.
   * @return The translation from 'initPose' to 'finalPose'.
   */
  static Eigen::Translation3d ComputeRoughTranslationOffset(const Transform& initPose, const Transform& finalPose);

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute orientation offset between two trajectories.
   * @param[in] initPoseFrom  The initial point of the initial trajectory.
   * @param[in] initPoseTo    The final point of the initial trajectory.
   * @param[in] finalPoseFrom The initial point of the final trajectory.
   * @param[in] finalPoseTo   The final point of the final trajectory.
   * @return The rotation from (initPoseFrom, initPoseTo) to (finalPoseFrom, finalPoseTo).
   */
  static Eigen::Quaterniond ComputeRoughRotationOffset(const Transform& initPoseFrom, const Transform& initPoseTo,
                                                       const Transform& finalPoseFrom, const Transform& finalPoseTo);

private:

  unsigned int NbrIcpIterations = 50;  ///< Max number of iterations to do in ICP matching.
  bool InitWithRoughEstimate = true;   ///< Init ICP with a rough estimate of the transform to help convergence.
  bool NoRoll = false;   ///< If true, roll angle (X axis) will be set to 0 in output transform.
  bool Verbose = false;  ///< If true, print some debug info.
};

} // end of LidarSlam namespace