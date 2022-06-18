//==============================================================================
// Copyright 2020 Kitware, Inc.
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-01-22
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

// A new PCL Point is added so we need to recompile PCL to be able to use
// filters (pcl::io::OctreePointCloudCompression) with this new type
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "LidarSlam/Utilities.h"

// PCL Octree compression does not compile properly on Windows with MSVC
// This issue may not be present on PCL version higher than 1.10.1
// OctreeCompressedPointCloud is aliased into a basic PCLPointCloud.
#if defined(_MSC_VER)
#define PCL_OCTREE_COMPRESSION_NOT_AVAILABLE
#else

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/impl/octree_pointcloud_compression.hpp>  // seems to be missing in otree_pointcloud_compression.h

// This workaround is only available on Linux, as sigaction is only supported on UNIX systems.
// See OctreeCompressedPointCloud::GetCloud() for more details about why this is needed.
#ifdef __linux__
  #include <signal.h>
  namespace
  {
    // Attach SIGFPE to c++ exception.
    // This allows to properly deal with division by 0 or other computation errors.
    // NOTE : See OctreeCompressedPointCloud::GetCloud() for more details about why this is needed.
    void sigfpe_handler(int /* signum */) { throw std::logic_error("SIGFPE"); }
  }
#endif
#endif


namespace LidarSlam
{

//! PCD file data format.
enum PCDFormat
{
  ASCII = 0,
  BINARY = 1,
  BINARY_COMPRESSED = 2
};

//! PointCloudStorage data type.
enum PointCloudStorageType
{
  PCL_CLOUD = 0,
  OCTREE_COMPRESSED = 1,
  PCD_ASCII = 2,
  PCD_BINARY = 3,
  PCD_BINARY_COMPRESSED = 4
};

//------------------------------------------------------------------------------
/*!
 * @brief Save pointcloud to PCD file according to data format.
 * @param[in] path The path to PCD file to write pointcloud to.
 * @param[in] cloud The pointcloud to save.
 * @param[in] pcdDataFormat The PCD file data format to use.
 * @return 0 if ok, negative error number otherwise.
 */
template<typename PointT>
int savePointCloudToPCD(std::string const& path,
                        pcl::PointCloud<PointT> const& cloud,
                        PCDFormat pcdDataFormat,
                        bool verbose = false)
{
  if (cloud.empty())
    return -3;

  switch (pcdDataFormat)
  {
    case ASCII:
      if (verbose)
        std::cout << "Saving pointcloud to ascii PCD file at " << path << std::endl;
      return pcl::io::savePCDFileASCII<PointT>(path, cloud);

    case BINARY:
      if (verbose)
        std::cout << "Saving pointcloud to binary PCD file at " << path << std::endl;
      return pcl::io::savePCDFileBinary<PointT>(path, cloud);

    case BINARY_COMPRESSED:
      if (verbose)
        std::cout << "Saving pointcloud to binary_compressed PCD file at " << path << std::endl;
      return pcl::io::savePCDFileBinaryCompressed<PointT>(path, cloud);

    default:
      PRINT_ERROR("Unknown PCDFormat value (" << pcdDataFormat << "). Unable to save pointcloud.");
      return -4;
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief Base abstract class to store a PCL pointcloud under different formats.
 */
template<typename PointT>
struct PointCloudData
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  virtual ~PointCloudData() = default;

  virtual void SetCloud(CloudTPtr const& cloud) = 0;  ///< Fill with new cloud.
  virtual CloudTPtr GetCloud() = 0;                   ///< Get stored cloud.
  virtual size_t GetMemorySize() = 0;                 ///< Approximate memory usage by pointcloud data.
};

//------------------------------------------------------------------------------
/*!
 * @brief Store PCL pointcloud without any compression in RAM.
 */
template<typename PointT>
struct PCLPointCloud final : public PointCloudData<PointT>
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  PCLPointCloud(CloudTPtr const& cloud) : Cloud(cloud) {}
  void SetCloud(CloudTPtr const& cloud) override { this->Cloud = cloud; }
  CloudTPtr GetCloud() override { return this->Cloud; }
  size_t GetMemorySize() override { return sizeof(*this->Cloud) + (sizeof(PointT) * this->Cloud->size()); }

  private:
    CloudTPtr Cloud;  ///< Raw uncompressed pointcloud.
};


//------------------------------------------------------------------------------
// PCL Octree compression does not compile properly on Windows with MSVC 2015
// and below. Therefore, we cannot compile OctreeCompressedPointCloud.
// If we use this code with MSVC 2015 and below, OctreeCompressedPointCloud
// becomes just an alias for basic PCLPointCloud.
#ifdef PCL_OCTREE_COMPRESSION_NOT_AVAILABLE
#pragma message("PCL Octree PointCloud Compression is not supported by MSVC 2015 and below.")
//------------------------------------------------------------------------------
template<typename PointT>
using OctreeCompressedPointCloud = PCLPointCloud<PointT>;
#else
//------------------------------------------------------------------------------
/*!
 * @brief Compress (with small loss) pointcloud with octree, and store pointcloud as binary data in RAM.
 */
template<typename PointT>
struct OctreeCompressedPointCloud final : public PointCloudData<PointT>
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  OctreeCompressedPointCloud(CloudTPtr const& cloud)
  {
    #ifdef __linux__
      // DEBUG : Attach SIGFPE to exception
      // See OctreeCompressedPointCloud::GetCloud() for more details about why this is needed.
      struct sigaction action;
      sigemptyset(&action.sa_mask);
      action.sa_flags = SA_NODEFER;
      action.sa_handler = &sigfpe_handler;
      sigaction(SIGFPE, &action, NULL);
    #endif
    // Compress pointcloud data
    this->SetCloud(cloud);
  }

  void SetCloud(CloudTPtr const& cloud) override
  {
    // Octree compression
    pcl::io::OctreePointCloudCompression<PointT> compression(pcl::io::MANUAL_CONFIGURATION, false,
                                                             0.001, // pointResolution
                                                             0.05,  // octreeResolution
                                                             false, // doVoxelGridDownDownSampling
                                                             100,   // iFrameRate
                                                             1,     // colorBitResolution
                                                             false  // doColorEncoding
                                                             );
    compression.encodePointCloud(cloud, this->CompressedData);
  }

  CloudTPtr GetCloud() override
  {
    // Decode compressed pointcloud
    CloudTPtr cloud(new CloudT);
    pcl::io::OctreePointCloudCompression<PointT> compression;
    #ifdef __linux__
      // DEBUG : If pointcloud has a little less than 2^i points, octree
      // compression encoding is wrongly done, and can lead to division by 0 in
      // decompression step.
      // See https://github.com/PointCloudLibrary/pcl/pull/3579 for more details.
      // This workaround is necessary until ROS uses PCL > 1.10.0.99 (>= 8ed756fcfaf710cd5f3051704fdd8af7b0d4bf61)
      try
      {
        compression.decodePointCloud(this->CompressedData, cloud);
      }
      catch (const std::logic_error& e)
      {
        PRINT_ERROR("Decompression failed. Returning empty pointcloud.");
      }
    #else
      compression.decodePointCloud(this->CompressedData, cloud);
    #endif
    // Set back compressed data read position to beginning (missing in OctreePointCloudCompression::decodePointCloud())
    this->CompressedData.seekg(0, std::ios::beg);
    return cloud;
  }

  size_t GetMemorySize() override
  {
    std::streampos current = this->CompressedData.tellp();
    this->CompressedData.seekp(0, std::ios::end);
    size_t size = this->CompressedData.tellp();
    this->CompressedData.seekp(current, std::ios::beg);
    return size;
  }

  private:
    std::stringstream CompressedData;  ///< Binary compressed pointcloud data.
};
#endif

//------------------------------------------------------------------------------
/*!
 * @brief Store PCL pointcloud on disk as PCD file.
 */
template<typename PointT, PCDFormat pcdFormat>
struct PCDFilePointCloud final : public PointCloudData<PointT>
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  PCDFilePointCloud(CloudTPtr const& cloud, std::string const& pcdDirPath = "point_cloud_log/")
  {
    boost::filesystem::create_directory(pcdDirPath);
    this->PCDFilePath = pcdDirPath + std::to_string(this->PCDFileIndex) + ".pcd";
    this->PCDFileIndex++;
    this->SetCloud(cloud);
  }

  // Define default constructor/assignement/move semantics as ~PCDFilePointCloud is specified
  PCDFilePointCloud() = default;
  PCDFilePointCloud(const PCDFilePointCloud&) = default;
  PCDFilePointCloud(PCDFilePointCloud&&) = default;
  PCDFilePointCloud& operator=(const PCDFilePointCloud&) = default;
  PCDFilePointCloud& operator=(PCDFilePointCloud&&) = default;

  ~PCDFilePointCloud() override
  {
    if (std::remove(this->PCDFilePath.c_str()) != 0)
      PRINT_WARNING("Unable to delete PCD file at " << this->PCDFilePath);
    // No need to decrement PCDFileIndex, it will be clearer for debug like that.
  }

  void SetCloud(CloudTPtr const& cloud) override
  {
    if (savePointCloudToPCD(this->PCDFilePath, *cloud, pcdFormat) != 0)
      PRINT_ERROR("Failed to write binary PCD file to " << this->PCDFilePath);
  }

  CloudTPtr GetCloud() override
  {
    CloudTPtr cloud(new CloudT);
    if (pcl::io::loadPCDFile(this->PCDFilePath, *cloud) != 0)
      PRINT_ERROR("PCD file loading failed. Returning empty pointcloud.");
    return cloud;
  }

  size_t GetMemorySize() override
  {
    std::ifstream in(this->PCDFilePath, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
  }

  protected:
    static unsigned int PCDFileIndex;  ///< The index of the PCD file currently being written.
    std::string PCDFilePath;           ///< Path to PCD file.
};

template<typename PointT, PCDFormat pcdFormat>
unsigned int PCDFilePointCloud<PointT, pcdFormat>::PCDFileIndex = 0;

template<typename PointT>
using AsciiPCDFilePointCloud = PCDFilePointCloud<PointT, PCDFormat::ASCII>;

template<typename PointT>
using BinaryPCDFilePointCloud = PCDFilePointCloud<PointT, PCDFormat::BINARY>;

template<typename PointT>
using BinaryCompressedPCDFilePointCloud = PCDFilePointCloud<PointT, PCDFormat::BINARY_COMPRESSED>;

//------------------------------------------------------------------------------
/*!
 * @brief Structure used to log pointclouds either under uncompressed/compressed format.
 */
template<typename PointT>
struct PointCloudStorage
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  PointCloudStorage(CloudTPtr const& cloud, PointCloudStorageType storage) { this->SetCloud(cloud, storage); }

  inline PointCloudStorageType StorageType() const { return this->Storage; }
  inline size_t PointsSize() const { return this->Points; }
  inline size_t MemorySize() const { return this->Data->GetMemorySize(); }

  void SetCloud(CloudTPtr const& cloud, PointCloudStorageType storage)
  {
    this->Storage = storage;
    this->Points = cloud->size();
    switch (storage)
    {
      case PCL_CLOUD:             this->Data.reset(new PCLPointCloud<PointT>(cloud));                     break;
      case OCTREE_COMPRESSED:     this->Data.reset(new OctreeCompressedPointCloud<PointT>(cloud));        break;
      case PCD_ASCII:             this->Data.reset(new AsciiPCDFilePointCloud<PointT>(cloud));            break;
      case PCD_BINARY:            this->Data.reset(new BinaryPCDFilePointCloud<PointT>(cloud));           break;
      case PCD_BINARY_COMPRESSED: this->Data.reset(new BinaryCompressedPCDFilePointCloud<PointT>(cloud)); break;
      default:
        PRINT_ERROR("Unkown PointCloudStorageType (" << storage << ")."); break;
    }
  }

  CloudTPtr GetCloud() const { return this->Data->GetCloud(); }

  private:
    size_t Points;                                 ///< Number of points in stored pointcloud.
    PointCloudStorageType Storage;                 ///< How is stored pointcloud data.
    std::unique_ptr<PointCloudData<PointT>> Data;  ///< Pointcloud data.
};

} // end of LidarSlam namespace
