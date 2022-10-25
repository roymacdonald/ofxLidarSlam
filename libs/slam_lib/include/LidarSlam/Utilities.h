//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-06-16
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

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <iostream>
#include <iomanip>
#include <math.h>
#include <numeric>
#include <cctype>

//==============================================================================
//   Usefull macros or typedefs
//==============================================================================

// If M_PI is not defined, define it
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// Set cout to print floating values with fixed precision of a given number of decimals
#define SET_COUT_FIXED_PRECISION(decimals)                   \
  std::streamsize ss = std::cout.precision();                \
  std::cout << std::fixed << std::setprecision(decimals);

// Reset cout float printing state back to before
// NOTE : RESET_COUT_FIXED_PRECISION in the same scope as SET_COUT_FIXED_PRECISION
#define RESET_COUT_FIXED_PRECISION                           \
  std::cout << std::setprecision(ss);                        \
  std::cout.unsetf(std::ios::fixed | std::ios_base::floatfield);

// Print with colors on terminals that support ANSI color codes
// (Supported by UNIX systems and from Windows 10)
#define DEFAULT "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define GRAY    "\033[37m"
#define PRINT_COLOR(color, s) std::cout << color << s << DEFAULT << std::endl;
#define PRINT_INFO(s) std::cout << GRAY << s << DEFAULT << std::endl;
#define PRINT_WARNING(s) std::cerr << YELLOW << "[WARNING] " << s << DEFAULT << std::endl;
#define PRINT_ERROR(s)   std::cerr << RED    << "[ERROR] "   << s << DEFAULT << std::endl;

namespace Eigen
{
  ///! @brief 6x6 matrix of double
  using Matrix6d = Matrix<double, 6, 6>;

  ///! @brief 6D Vector of double
  using Vector6d = Matrix<double, 6, 1>;

  //! We could use an unaligned Isometry3d in order to avoid having to use
  //! Eigen::aligned_allocator<Eigen::Isometry3d> in each declaration of
  //! std::container storing isometries instances, as documented here:
  //! http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
  using UnalignedIsometry3d = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;
}

namespace LidarSlam
{
namespace Utils
{
//==============================================================================
//   Common helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Sort a vector and return sorted indices
 * @param v The vector to sort
 * @param ascending If true, sort in ascending (increasing) order
 * @return The sorted indices such that the first index is the biggest input
 *         value and the last the smallest.
 */
template<typename T>
std::vector<size_t> SortIdx(const std::vector<T>& v, bool ascending=true)
{
  // Initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // Sort indices based on comparing values in v
  if (ascending)
    std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
  else
    std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

  return idx;
}

//------------------------------------------------------------------------------
/*!
 * @brief Clamp a value between min and max saturation thresholds.
 * @param val The value to clamp
 * @param min The lower saturation threshold
 * @param max The higher saturation threshold
 * @return The value clamped between min and max: min <= retval <= max
 */
//----------------------------------------------------------------------------
template<typename T>
T Clamp(const T& val, const T& min, const T& max)
{
  return (val < min) ? min : (max < val) ? max : val;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert first char to upper case
 */
std::string Capitalize(std::string st);

//------------------------------------------------------------------------------
/*!
 * @brief Make the string plural (add 's')
 */
std::string Plural(std::string st);

//==============================================================================
//   Geometry helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Convert from radians to degrees
 * @param rad The angle value, in radians
 * @return The angle value, in degrees
 */
template<typename T>
inline constexpr T Rad2Deg(const T& rad)
{
  return rad / M_PI * 180.;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert from degrees to radians
 * @param deg The angle value, in degrees
 * @return The angle value, in radians
 */
template<typename T>
inline constexpr T Deg2Rad(const T& deg)
{
  return deg / 180. * M_PI;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert roll, pitch and yaw euler angles to rotation matrix
 * @param roll Rotation on X axis
 * @param pitch Rotation on Y axis
 * @param yaw Rotation on Z axis
 * @return The 3x3 rotation matrix defined by : R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
Eigen::Matrix3d RPYtoRotationMatrix(double roll, double pitch, double yaw);

//------------------------------------------------------------------------------
/*!
 * @brief Get Roll Pitch and Yaw euler angles from rotation matrix
 * @param rot The 3x3 rotation matrix, defined by : R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * @return Euler angles around X, Y and Z axes, according to ZYX convention.
 */
Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& rot);

//------------------------------------------------------------------------------
/*!
 * @brief Convert translation and RPY to full rigid transform
 * @param x Translation along X axis
 * @param y Translation along Y axis
 * @param z Translation along Z axis
 * @param roll Rotation on X axis
 * @param pitch Rotation on Y axis
 * @param yaw Rotation on Z axis
 * @return The rigid transform (rotation + translation)
 */
Eigen::Isometry3d XYZRPYtoIsometry(double x, double y, double z, double roll, double pitch, double yaw);

//------------------------------------------------------------------------------
/*!
 * @brief Convert pose as XYZRPY to full rigid transform
 * @param pose
 * @return The rigid transform (rotation + translation)
 */
Eigen::Isometry3d PoseToIsometry(const Eigen::Vector6d& pose);

//------------------------------------------------------------------------------
/*!
 * @brief Convert translation and RPY to full rigid transform
 * @param xyzrpy Translation (X, Y, Z) and Rotation (rX, rY, rZ)
 * @return The rigid transform (rotation + translation)
 */
template<typename T>
inline Eigen::Isometry3d XYZRPYtoIsometry(const T& xyzrpy)
{
  return XYZRPYtoIsometry(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert RPY and translation to full rigid transform
 * @param rpyxyz Rotation (rX, rY, rZ) and Translation (X, Y, Z)
 * @return The rigid transform (rotation + translation)
 */
template<typename T>
inline Eigen::Isometry3d RPYXYZtoIsometry(const T& rpyxyz)
{
  return XYZRPYtoIsometry(rpyxyz[3], rpyxyz[4], rpyxyz[5], rpyxyz[0], rpyxyz[1], rpyxyz[2]);
}

//-----------------------------------------------------------------------------
/*!
 * @brief Get X, Y, Z, Roll, Pitch, Yaw from rigid transform
 * @param transform The rigid transform
 * @return 6D array, with translation (X, Y, Z) and rotation (rX, rY, rZ)
 */
Eigen::Vector6d IsometryToXYZRPY(const Eigen::Isometry3d& transform);

//------------------------------------------------------------------------------
/*!
 * @brief Get Roll, Pitch, Yaw, X, Y, Z from rigid transform
 * @param transform The rigid transform
 * @return 6D array, with rotation (rX, rY, rZ) and translation (X, Y, Z)
 */
Eigen::Vector6d IsometryToRPYXYZ(const Eigen::Isometry3d& transform);

//------------------------------------------------------------------------------
/*!
 * @brief Compute the centroid and PCA of a pointcloud subset.
 * @param[in] cloud The input pointcloud
 * @param[in] indices The points to consider from cloud
 * @param[out] centroid The mean point of the subset of points
 * @param[out] eigenVectors The PCA eigen vectors corresponding to eigenValues
 * @param[out] eigenValues The PCA eigen values, sorted by ascending order
 */
template<typename PointT, typename Scalar>
void ComputeMeanAndPCA(const pcl::PointCloud<PointT>& cloud,
                       const std::vector<int>& indices,
                       Eigen::Matrix<Scalar, 3, 1>& centroid,
                       Eigen::Matrix<Scalar, 3, 3>& eigenVectors,
                       Eigen::Matrix<Scalar, 3, 1>& eigenValues)
{
  // Compute mean and normalized covariance matrix
  EIGEN_ALIGN16 Eigen::Matrix<Scalar, 3, 3> covarianceMatrix;
  EIGEN_ALIGN16 Eigen::Matrix<Scalar, 4, 1> xyzCentroid;
  pcl::computeMeanAndCovarianceMatrix(cloud, indices, covarianceMatrix, xyzCentroid);
  centroid = xyzCentroid.template head<3>();

  // Compute eigen values and corresponding eigen vectors
  pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);
}

//==============================================================================
//   Covariance helpers
//==========================================================================

//------------------------------------------------------------------------------
// Check if covariance matrix is valid
template<typename Matrix>
inline bool isCovarianceValid(const Matrix& cov, double thresh = 1e-12)
{
  return cov.array().abs().maxCoeff() > thresh;
}

//------------------------------------------------------------------------------
// Create spherical covariance with position and angle error estimations
inline Eigen::Matrix6d CreateDefaultCovariance(float positionErr = 1e-2, float angleErr = 1e-2)
{
  Eigen::Matrix6d covariance = Eigen::Matrix6d::Identity();
  covariance.block(0,0,3,3) = std::pow(positionErr, 2) * Eigen::Matrix3d::Identity();
  covariance.block(3,3,3,3) = std::pow(angleErr, 2) * Eigen::Matrix3d::Identity();
  return covariance;
}

//==============================================================================
//   PCL helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Apply a rigid transform to a point (in-place transformation).
 * @param p The point to transform, will be in-place transformed.
 * @param transform The rigid transform (rotation + translation) to apply.
 */
template<typename PointT>
inline void TransformPoint(PointT& p, const Eigen::Isometry3d& transform)
{
  p.getVector4fMap() = transform.template cast<float>() * p.getVector4fMap();
}

//------------------------------------------------------------------------------
/*!
 * @brief Apply a rigid transform to a point
 * @param p The point to transform
 * @param transform The rigid transform (rotation + translation) to apply.
 * @return The transformed point (all other fields are copied from p)
 */
template<typename PointT>
inline PointT TransformPoint(const PointT& p, const Eigen::Isometry3d& transform)
{
  PointT out(p);
  TransformPoint(out, transform);
  return out;
}

//------------------------------------------------------------------------------
/*!
 * @brief Copy pointcloud metadata to an other cloud
 * @param[in] from The pointcloud to copy info from
 * @param[out] to The pointcloud to copy info to
 */
template<typename PointT>
inline void CopyPointCloudMetadata(const pcl::PointCloud<PointT>& from, pcl::PointCloud<PointT>& to)
{
  to.header = from.header;
  to.is_dense = from.is_dense;
  to.sensor_orientation_ = from.sensor_orientation_;
  to.sensor_origin_ = from.sensor_origin_;
}

//------------------------------------------------------------------------------
/*!
 * @brief Build and return a PCL header
 * @param timestamp PCL timestamp, in microseconds
 * @param frameId Coordinates system frame ID
 * @param seq Sequence number
 * @return PCL header filled with these info
 */
inline pcl::PCLHeader BuildPclHeader(uint64_t timestamp, const std::string& frameId, unsigned int seq = 0)
{
  pcl::PCLHeader header;
  header.stamp = timestamp;
  header.frame_id = frameId;
  header.seq = seq;
  return header;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert PCL timestamp (in microseconds) to seconds
 * @param pclStampUs PCL timestamp, in microseconds
 * @return Timestamp in seconds
 */
inline constexpr double PclStampToSec(uint64_t pclStampUs)
{
  return pclStampUs * 1e-6;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert seconds to PCL timestamp (in microseconds)
 * @param seconds Timestamp, in seconds
 * @return PCL timestamp in microseconds, rounded up to closer integer microsecond
 */
inline uint64_t SecToPclStamp(double seconds)
{
  return std::round(seconds * 1e6);
}

//==============================================================================
//   Processing duration measurements
//==============================================================================

namespace Timer
{
  //----------------------------------------------------------------------------
  /*!
  * @brief Reset all timers values.
  *
  * NOTE: This resets timers declared in all translation units.
  */
  void Reset();

  //----------------------------------------------------------------------------
  /*!
  * @brief Init a timer.
  * @param timer The name of the timer
  */
  void Init(const std::string& timer);

  //----------------------------------------------------------------------------
  /*!
  * @brief Get the timer value, in seconds.
  * @param timer The name of the timer
  * @return The duration value, in seconds, since the initialization of the timer
  *
  * NOTE : This may return garbage if the counter has not been initialized yet.
  */
  double Stop(const std::string& timer);

  //----------------------------------------------------------------------------
  /*!
  * @brief Print a given timer value and its average value, in milliseconds.
  * @param timer The name of the timer
  * @param nbDigits The number of digits to use to round milliseconds timer value
  *
  * NOTE : This may display garbage if the counter has not been initialized yet.
  */
  void StopAndDisplay(const std::string& timer, int nbDigits = 3);

  //----------------------------------------------------------------------------
  /*!
  * @brief Print a given timer average value, in milliseconds.
  * @param timer The name of the timer
  * @param nbDigits The number of digits to use to round milliseconds timer value
  *
  * NOTE : This may display garbage if the counter has not been initialized yet.
  */
  void Display(const std::string& timer, int nbDigits = 3);
}  // end of Timer namespace
}  // end of Utils namespace
}  // end of LidarSlam namespace