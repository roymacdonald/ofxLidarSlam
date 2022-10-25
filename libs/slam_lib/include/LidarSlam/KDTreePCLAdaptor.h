//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Nicolas Cadart (Kitware SAS)
// Creation date: 2018-04-19
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

#include <nanoflann.hpp>
#include <pcl/point_cloud.h>

namespace LidarSlam
{

template<typename PointT>
class KDTreePCLAdaptor
{
  using Point = PointT;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = typename PointCloud::Ptr;

  using metric_t = typename nanoflann::metric_L2_Simple::traits<float, KDTreePCLAdaptor<Point>>::distance_t;
  using index_t = nanoflann::KDTreeSingleIndexAdaptor<metric_t, KDTreePCLAdaptor<Point>, 3, int>;

public:

  /**
    * \brief Build a Kd-tree from a given pointcloud.
    * \param cloud The pointcloud to encode in the kd-tree.
    * \param leafMaxSize The maximum size of a leaf of the tree (refer to
    * https://github.com/jlblancoc/nanoflann#21-kdtreesingleindexadaptorparamsleaf_max_size)
    */
  KDTreePCLAdaptor(PointCloudPtr cloud = PointCloudPtr(new PointCloud), int leafMaxSize = 16)
  {
    this->Reset(cloud, leafMaxSize);
  }

  /**
    * \brief Init the Kd-tree from a given pointcloud.
    * \param cloud The pointcloud to encode in the kd-tree.
    * \param leafMaxSize The maximum size of a leaf of the tree (refer to
    * https://github.com/jlblancoc/nanoflann#21-kdtreesingleindexadaptorparamsleaf_max_size)
    */
  void Reset(PointCloudPtr cloud = PointCloudPtr(new PointCloud), int leafMaxSize = 16)
  {
    // Copy the input cloud
    this->Cloud = cloud;

    // Build KD-tree
    this->Index = std::make_unique<index_t>(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leafMaxSize));
    this->Index->buildIndex();
  }

  /**
    * \brief Finds the `K` nearest neighbors points in the KD-tree to a given query point.
    * \param[in] queryPoint Input point to look closest neighbors to.
    * \param[in] knearest Number of nearest neighbors to find.
    * \param[out] knnIndices Indices of the NN.
    * \param[out] knnSqDistances Squared distances of the NN to the query point.
    * \return Number `N` of neighbors found.
    *
    * \note Only the first `N` entries in `knnIndices` and `knnSqDistances` will
    * be valid. Return may be less than `knearest` only if the number of
    * elements in the tree is less than `knearest`.
    */
  inline size_t KnnSearch(const float queryPoint[3], int knearest, int* knnIndices, float* knnSqDistances) const
  {
    return this->Index->knnSearch(queryPoint, knearest, knnIndices, knnSqDistances);
  }
  inline size_t KnnSearch(const float queryPoint[3], int knearest, std::vector<int>& knnIndices, std::vector<float>& knnSqDistances) const
  {
    // Init result to have large enough buffers that will be filled by knnSearch
    knnIndices.resize(knearest);
    knnSqDistances.resize(knearest);
    // Find nearest neighbors
    size_t kneighbors = this->KnnSearch(queryPoint, knearest, knnIndices.data(), knnSqDistances.data());
    // If less than 'knearest' NN have been found, the last neighbors values are
    // wrong, therefore we need to ignore them
    knnIndices.resize(kneighbors);
    knnSqDistances.resize(kneighbors);
    return kneighbors;
  }
  inline size_t KnnSearch(const double queryPoint[3], int knearest, std::vector<int>& knnIndices, std::vector<float>& knnSqDistances) const
  {
    float pt[3];
    std::copy(queryPoint, queryPoint + 3, pt);
    return this->KnnSearch(pt, knearest, knnIndices, knnSqDistances);
  }
  inline size_t KnnSearch(const Point& queryPoint, int knearest, std::vector<int>& knnIndices, std::vector<float>& knnSqDistances) const
  {
    return this->KnnSearch(queryPoint.data, knearest, knnIndices, knnSqDistances);
  }

  /**
    * \brief Get the input pointcloud.
    * \return The input pointcloud used to build KD-tree.
    */
  inline PointCloudPtr GetInputCloud() const
  {
    return this->Cloud;
  }

  // ---------------------------------------------------------------------------
  //   Methods required by nanoflann adaptor design
  // ---------------------------------------------------------------------------

  inline const KDTreePCLAdaptor& derived() const
  {
    return *this;
  }

  inline KDTreePCLAdaptor& derived()
  {
    return *this;
  }

  /**
    * \brief Returns the number of points in the input pointcloud.
    * \note This method is required by nanoflann design, and should not be used
    * by user.
    */
  inline size_t kdtree_get_point_count() const
  {
    return this->Cloud->size();
  }

  /**
    * \brief Returns the dim'th component of the idx'th point of the pointcloud.
    * \note `dim` should only be in range [0-3].
    * \note This method is required by nanoflann design, and should not be used
    * by user.
    */
  inline float kdtree_get_pt(const int idx, const int dim) const
  {
    return this->Cloud->points[idx].data[dim];
  }

  /**
    * Optional bounding-box computation.
    * Return false to default to a standard bbox computation loop.
    * Return true if the BBOX was already computed by the class and returned in
    * "bb" so it can be avoided to redo it again.
    * Look at bb.size() to find out the expected dimensionality.
    * \note This method is required by nanoflann design, and should not be used
    * by user.
    */
  template <class BBOX>
  inline bool kdtree_get_bbox(BBOX& /*bb*/) const
  {
    return false;
  }

protected:

  //! The kd-tree index for the user to call its methods as usual with any other FLANN index.
  std::unique_ptr<index_t> Index;

  //! The input data
  PointCloudPtr Cloud;
};

} // end of LidarSlam namespace