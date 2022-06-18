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
    
    
    ///  Number of neighbors to use on each side of current point to estimate
    ///  the keypoints scores (curvature, planarity, depth gap, ...) in each
    ///  scan line.
    
    ofParameter<int> NeighborWidth = {"Neighborhood width", 4, 1, 10};
    
    
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
    ///  the depth gap between the point and its far background neighborhood.
    ///  If the gap is big enough, we consider the point as an edge keypoint.
    ofParameter<float> EdgeSaliencyThreshold = {"Edge min saliency distance", 1.5, 0, 10};
    
    
    
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
    
    
    ofParameterGroup parameters = {"Keypoint Extractor"};
    
    std::shared_ptr<LidarSlam::SpinningSensorKeypointExtractor> extractor = nullptr;
protected:
    
    ofEventListeners listeners;
    
};


