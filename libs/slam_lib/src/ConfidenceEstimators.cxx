//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-06-01
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

#include <LidarSlam/ConfidenceEstimators.h>

namespace LidarSlam
{
namespace Confidence
{

//-----------------------------------------------------------------------------
float LCPEstimator(PointCloud::ConstPtr cloud,
                   const std::map<Keypoint, std::shared_ptr<RollingGrid>>& maps,
                   float subsamplingRatio,
                   int nbThreads)
{
  // Number of points to process
  int nbPoints = cloud->size() * subsamplingRatio;
  if (nbPoints == 0 || maps.empty())
    return -1.;

  // Iterate on all points of input cloud to process
  float lcp = 0.;
  #pragma omp parallel for num_threads(nbThreads) reduction(+:lcp)
  for (int n = 0; n < nbPoints; ++n)
  {
    // Compute the LCP contribution of the current point
    const auto& point = cloud->at(n / subsamplingRatio);
    float bestProba = 0.;
    for (const auto& map : maps)
    {
      // Get nearest neighbor
      int nnIndex;
      float nnSqDist;
      if (map.second->GetSubMapKdTree().KnnSearch(point.data, 1, &nnIndex, &nnSqDist))
      {
        // We use a Gaussian like estimation for each point fitted in target leaf space
        // to check the probability that one cloud point has a neighbor in the target
        // Probability = 1 if the two points are superimposed
        // Probability < 0.011 if the distance is g.t. the leaf size
        float sqLCPThreshold = std::pow(map.second->GetLeafSize() / 3.f, 2);
        float currentProba = std::exp( -nnSqDist / (2.f * sqLCPThreshold) );
        if (currentProba > bestProba)
          bestProba = currentProba;
      }
    }
    lcp += bestProba;
  }
  return lcp / nbPoints;
}

} // end of Confidence namespace
} // end of LidarSlam namespace