//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Nicolas Cadart (Kitware SAS)
// Creation date: 2020-11-10
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

#include <vector>
#include <string>
#include <map>

namespace LidarSlam
{

//------------------------------------------------------------------------------
//! Type of a keypoint
enum Keypoint
{
  EDGE  = 0,   ///< edge keypoint (sharp local structure)
  INTENSITY_EDGE  = 1,   ///< intensity edge keypoint (sharp local intensity)
  PLANE = 2,   ///< plane keypoint (flat local structure)
  BLOB  = 3,   ///< blob keypoint (spherical local structure)
  nKeypointTypes
};

static const std::vector<Keypoint> KeypointTypes = { EDGE, INTENSITY_EDGE, PLANE, BLOB };
static const std::map<Keypoint, std::string> KeypointTypeNames = { {EDGE, "edge"}, {INTENSITY_EDGE, "intensity_edge"}, {PLANE, "plane"}, {BLOB, "blob"} };

//------------------------------------------------------------------------------
//! How to deal with undistortion
enum UndistortionMode
{
  //! No undistortion is performed:
  //!  - End scan pose is optimized using rigid registration of raw scan and map.
  //!  - Raw input scan is added to map.
  NONE = 0,

  //! Undistortion is performed only once using estimated ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using estimated ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is iteratively optimized using rigid registration of undistorted scan and map.
  //!  - Undistorted scan is added to map.
  ONCE = 1,

  //! Undistortion is iteratively refined using optimized ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is optimized using rigid registration of undistorted scan and map.
  //!  - Iterate the three previous steps with updated ego-motion and poses.
  //!  - Undistorted scan is added to map.
  REFINED = 2,

  //! Undistort once with external pose information
  EXTERNAL = 3
};

//------------------------------------------------------------------------------
//! How to estimate Ego-Motion (approximate relative motion since last frame)
enum class EgoMotionMode
{
  //! No ego-motion step is performed : relative motion is Identity, new
  //! estimated Tworld is equal to previous Tworld.
  //! Fast, but may lead to unstable and imprecise Localization step if motion
  //! is important.
  NONE = 0,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses.
  //! Fast and precise if motion is roughly constant and continuous.
  MOTION_EXTRAPOLATION = 1,

  //! Estimate Trelative (and therefore Tworld) by globally registering new
  //! frame on previous frame.
  //! Slower and need textured enough environment, but do not rely on
  //! constant motion hypothesis.
  REGISTRATION = 2,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses. Then this estimation is refined by globally
  //! registering new frame on previous frame.
  //! Slower and need textured enough environment, but should be more precise
  //! and rely less on constant motion hypothesis.
  MOTION_EXTRAPOLATION_AND_REGISTRATION = 3,

  //! Use external pose as prior and none if external not available
  EXTERNAL = 4,

  //! Use external pose as prior and motion extrapolation if external not available
  EXTERNAL_OR_MOTION_EXTRAPOLATION = 5
};

//------------------------------------------------------------------------------
//! How to update the map
enum class MappingMode
{
  //! Do not update map, use initial map
  //! Performant in static environment and
  //! more robust to moving objects
  // Forbiding maps update can be useful for example in case
  // of post-SLAM optimization with GPS and then run localization only in fixed
  // optimized map or when performing two SLAM steps (mapping + localization)
  NONE = 0,

  //! Expand the map with new keypoints
  //! The points of the initial maps (if some were loaded) will not be modified
  ADD_KPTS_TO_FIXED_MAP = 1,

  //! Update map with new keypoints
  //! The points of the initial maps can disappear
  UPDATE = 2,
};

//------------------------------------------------------------------------------
//! How to downsample the map
// A voxel grid is used and various downsampling modes
// are possible to select the remaining point in each voxel
enum class SamplingMode
{
  //! Use the first point acquired
  //! Useful for performances issues
  FIRST = 0,

  //! Use the last point acquired
  //! Useful in dynamic environments
  LAST = 1,

  //! Use the point with maximum intensity
  //! The max intensity points can be the most acurate
  MAX_INTENSITY = 2,

  //! Use the closest point to the voxel center
  //! This allows the most uniform sampling but can be biased
  CENTER_POINT = 3,

  //! Use the centroid of the voxel
  //! This smoothes the points (can be useful for planes)
  //! /!\ The sampling process is longer
  CENTROID = 4
};

//------------------------------------------------------------------------------
//! External sensors' references
enum ExternalSensor
{
  //! Wheel odometer
  WHEEL_ODOM = 0,

  //! IMU
  IMU = 1,

  //! Landmark detector
  LANDMARK_DETECTOR = 2,

  //! GPS
  GPS = 3,

  //! Pose sensor
  POSE = 4
};

static const std::map<ExternalSensor, std::string> ExternalSensorNames = { {WHEEL_ODOM,"Wheel odometer"},
                                                                           {IMU, "IMU"},
                                                                           {LANDMARK_DETECTOR, "Landmark detector"},
                                                                           {GPS, "GPS"},
                                                                           {POSE, "POSE"} };

} // end of LidarSlam namespace