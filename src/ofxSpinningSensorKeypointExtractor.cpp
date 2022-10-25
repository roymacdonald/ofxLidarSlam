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

#include "ofxSpinningSensorKeypointExtractor.h"


#define ADD_PARAM_LISTENER_MACRO(name, type)                                  \
listeners.push( name .newListener([&](type & _arg){extractor->Set##name(_arg);}));




//-----------------------------------------------------------------------------
ofxSpinningSensorKeypointExtractor::ofxSpinningSensorKeypointExtractor()
: extractor(std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>())
{
    
    parameters.add(NbThreads);
    parameters.add(InputSamplingRatio);
    
    parameters.add(AzimuthMin);
    parameters.add(AzimuthMax);
    
    parameters.add(MinNeighNb);
    parameters.add(MinNeighRadius);
    
    parameters.add(MinDistanceToSensor);
    parameters.add(MinBeamSurfaceAngle);
    parameters.add(PlaneSinAngleThreshold);
    parameters.add(EdgeSinAngleThreshold);
    parameters.add(EdgeDepthGapThreshold);
    parameters.add(EdgeIntensityGapThreshold);
    parameters.add(EdgeNbGapPoints);
    
    parameters.add(MaxPoints);
    parameters.add(VoxelResolution);
    
    
    
    
    ADD_PARAM_LISTENER_MACRO(NbThreads, int);
    ADD_PARAM_LISTENER_MACRO(MaxPoints, int);
    ADD_PARAM_LISTENER_MACRO(InputSamplingRatio, float);
    ADD_PARAM_LISTENER_MACRO(MinNeighRadius, float);
    ADD_PARAM_LISTENER_MACRO(AzimuthMin, float);
    ADD_PARAM_LISTENER_MACRO(AzimuthMax, float);
    ADD_PARAM_LISTENER_MACRO(EdgeNbGapPoints, int);
    ADD_PARAM_LISTENER_MACRO(VoxelResolution, float);
    ADD_PARAM_LISTENER_MACRO(MinNeighNb, int)
    ADD_PARAM_LISTENER_MACRO(MinDistanceToSensor, float)
    ADD_PARAM_LISTENER_MACRO(MinBeamSurfaceAngle, float)
    ADD_PARAM_LISTENER_MACRO(PlaneSinAngleThreshold, float)
    ADD_PARAM_LISTENER_MACRO(EdgeSinAngleThreshold, float)
    ADD_PARAM_LISTENER_MACRO(EdgeDepthGapThreshold, float)
    ADD_PARAM_LISTENER_MACRO(EdgeIntensityGapThreshold, float)
    
    
    
    
}
