//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Authors: Laurenson Nick (Kitware SAS),
//			Sanchez Julia (Kitware SAS)
// Creation date: 2019-05-13
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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace LidarSlam
{

/** \brief A point structure representing Euclidean xyz coordinates, time, intensity, laser_id, device_id and label.
  * \ingroup common
  */
struct EIGEN_ALIGN16 LidarPoint
{
  inline LidarPoint (const LidarPoint &p) : x(p.x), y(p.y), z(p.z), time(p.time), intensity(p.intensity), laser_id(p.laser_id), device_id(p.device_id), label(p.label)
  {
    data[3] = 1.0f;
  }
  inline LidarPoint& operator=(const LidarSlam::LidarPoint& p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    time = p.time;
    intensity = p.intensity;
    laser_id = p.laser_id;
    device_id = p.device_id;
    label = p.label;
    return *this;
  }
  inline LidarPoint () : x(0.0f), y(0.0f), z(0.0f), time(0.0), intensity(0.0f), laser_id(0), device_id(0), label(0)
  {
    data[3] = 1.0f;
  }
  
  PCL_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  double time;
  float intensity;
  std::uint16_t laser_id;
  std::uint8_t device_id;
  std::uint8_t label;
  
  friend std::ostream& operator << (std::ostream& os, const LidarPoint& p);
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} ;

} // end of LidarSlam namespace

POINT_CLOUD_REGISTER_POINT_STRUCT (LidarSlam::LidarPoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (double, time, time)
                                   (float, intensity, intensity)
                                   (std::uint16_t, laser_id, laser_id)
                                   (std::uint8_t, device_id, device_id)
                                   (std::uint8_t, label, label)
)
