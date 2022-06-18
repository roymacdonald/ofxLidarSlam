//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-06
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

#include <Eigen/Geometry>

namespace LidarSlam
{

struct Transform
{
private:
  //! We use an unaligned Isometry3d in order to avoid having to use
  //! Eigen::aligned_allocator<Transform> in each declaration of std::container
  //! storing Transform instances, as documented here :
  //! http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
  using UnalignedIsometry3d = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;

public:

  UnalignedIsometry3d transform = UnalignedIsometry3d::Identity();  ///< Isometry representing pose/transform
  double time = 0.;          ///< [s] Timestamp of this transform.
  std::string frameid = "";  ///< Name of the frame coordinates the transform represents or is represented into.

  //----------------------------------------------------------------------------

  //! Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(double x, double y, double z, double rx, double ry, double rz, double t = 0., const std::string& frame = "");

  //! Uses (X, Y, Z, rX, rY, rZ) as input. Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(const Eigen::Matrix<double, 6, 1>& xyzrpy, double t = 0., const std::string& frame = "");

  //! Uses roll/pitch/yaw Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy, double t = 0., const std::string& frame = "");

  Transform(const Eigen::Isometry3d& transform, double t = 0., const std::string& frame = "");

  Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot, double t = 0., const std::string& frame = "");

  static Transform Identity() {return Transform(Eigen::Isometry3d::Identity());}

  //----------------------------------------------------------------------------

  //! Direct access to translation (position) part.
  double& x() {return this->transform(0, 3);}
  double& y() {return this->transform(1, 3);}
  double& z() {return this->transform(2, 3);}
  double x() const {return this->transform(0, 3);}
  double y() const {return this->transform(1, 3);}
  double z() const {return this->transform(2, 3);}

  void SetIsometry(const Eigen::Isometry3d& isometry) {this->transform = isometry;}
  Eigen::Isometry3d GetIsometry() const {return this->transform;}

  Eigen::Vector3d GetPosition() const {return this->transform.translation();}

  Eigen::Translation3d GetTranslation() const {return Eigen::Translation3d(this->transform.translation());}

  Eigen::Quaterniond GetRotation() const {return Eigen::Quaterniond(this->transform.linear());}

  Eigen::Matrix4d GetMatrix() const {return this->transform.matrix();}
};

} // end of LidarSlam namespace