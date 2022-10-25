//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Authors: Guilbert Pierre (Kitware SAS)
//          Laurenson Nick (Kitware SAS)
// Creation date: 2018-03-27
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

#include <LidarSlam/SpinningSensorKeypointExtractor.h>
#include "ofParameter.h"
#include "ofEvent.h"

class ofxSpinningSensorKeypointExtractor
{
public:
    ofxSpinningSensorKeypointExtractor();
    
    /// Max number of threads to use to process points in parallel
    ofParameter<int> NbThreads = {"Max num threads", 1,1, 8};

    
    
    /// Minimum number of points used on each side of the studied point to compute its curvature
    
    ofParameter<int> MinNeighNb = {"Min Num Neighbours", 4, 1, 10};
    
    
    ///  Minimum distance between a point and the sensor to be processed for
    ///  the keypoint extraction. In other words, all points too close from the
    ///  sensor are automatically rejected.
    ofParameter<float> MinDistanceToSensor = {"Min distance to sensor", 1.5, 0, 10};
    
    
    ///  Minimum angle between the laser beam and the surface on which
    ///  a point is laying to consider it as a potential keypoint.
    ///  In other words, all points laying on too oblique surfaces
    ///  relatively to the sensor are automatically rejected.
    ofParameter<float> MinBeamSurfaceAngle = {"Min laser beam to surface angle", 10., 0, 360};
    
    
    ///  One strategy to consider a point as a planar keypoint is to compute
    ///  the angle between the two lines that fit its previous and next neighborhoods.
    ///  If the angle is close enough to 0° or 180° (sin(Angle) low enough), we consider
    /// the point as a planar keypoint.
    ofParameter<float> PlaneSinAngleThreshold = {"Plane max sinus angle", 0.5,0,1};
    
    
    ///  One strategy to consider a point as an edge keypoint is to compute
    ///  the angle between the two lines that fit its previous and next neighborhoods.
    ///  If the angle is close enough to 90° (sin(Angle) high enough), we consider
    ///  the point as an edge keypoint.
    ofParameter<float> EdgeSinAngleThreshold = {"Edge min sinus angle", 0.86, 0, 1};
    
    
    
    ///  One strategy to consider a point as an edge keypoint is to compute
    ///  the intensity gap between the point and its left and right neighbors.
    ///  If the gap is big enough, we consider the point as an edge keypoint.
    ofParameter<float> EdgeIntensityGapThreshold = {"Edge min intensity gap", 50.,0,255};
    
    
    
    ///  One strategy to consider a point as an edge keypoint is to compute
    ///  the gap between the point and its neighborhood, fitted by a line. Then
    ///  we compute the distance between the point and the fitted lines. If the
    ///  gap is big enough on at least one side, we consider the point as an
    ///  edge keypoint.
    ofParameter<float> EdgeDepthGapThreshold = {"Edge min depth gap", 0.15};
    
    
    /// Maximum number of keypoints to extract
    ofParameter<int> MaxPoints = {"Max Points", INT_MAX, 0, INT_MAX};

    /// Sampling ratio to perform for real time issues
    ofParameter<float> InputSamplingRatio = {"Input Sampling Ratio", 1., 0, 1};


    /// Minimum radius to define the neighborhood to compute curvature of a studied point
    ofParameter<float> MinNeighRadius = {"Min Neigh Radius", 0.05f, 0.0001, 1};
    
    /// Minimum azimuth angle in degrees
    ofParameter<float> AzimuthMin = {"Azimuth Min", 0, 0,360};
    
    /// Maximum azimuth angle in degrees
    ofParameter<float> AzimuthMax = {"Azimuth Max", 360, 0, 360};
    
    /// Nb of points missed to define a space gap
    ofParameter<int> EdgeNbGapPoints = {"Edge Nb Gap Points", 5, 1,20};

    /// Size of a voxel used to downsample the keypoints
    /// It corresponds approx to the mean distance in meters between closest neighbors in the output keypoints cloud.
    ofParameter<float> VoxelResolution = {"Voxel Resolution", 0.1, 0.001, 1};

    
    ofParameterGroup parameters = {"Keypoint Extractor"};
    
    std::shared_ptr<LidarSlam::SpinningSensorKeypointExtractor> extractor = nullptr;
protected:
    
    ofEventListeners listeners;
    
};


