//
//  ofxLidarSlamParameters.h
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/10/22.
//

#ifndef ofxLidarSlamParameters_hpp
#define ofxLidarSlamParameters_hpp

#include "ofMain.h"
#include "ofxGui.h"

#include "ofxDropdown.h"


class ofxLidarSlamParameters{
    
public:

    ofxLidarSlamParameters();
    
    
    void draw();
    
    
    // ---------------------------------------------------------------------------
    //   General parameters
    // ---------------------------------------------------------------------------
    
    ofParameter<bool> bUseImuData = {"Use IMU data", true};
    ofParameter<int> NbThreads = {"Num Thrads", 1, 1, 8};
    ofParameter<bool> bEnableSlam = {"Enable Slam", true};
    ofParameter<void> reset = {"Reset"};
    
    ofParameter<void> saveMaps = {"Save Maps"};
    ofParameter<bool> saveMarkersOnly = {"Save Markers Only", true};
//    ofParameter<bool> saveMeshes = {"Save Meshes", false};
    
    
    ofParameter<bool> bDrawMarkers = {"Draw Markers", true};
    ofParameter<bool> bUseDepthTest = {"Use Depth Test", false};

    
//    ofParameter<bool> bAccumulateRegistered = {"Accumulate reg. meshes", false};
    
    ofParameter<float> accumEveryDistance = {"Accum. Distance", 1, 0, 10};
    ofParameter<size_t> accumEveryFrames = {"Accum. Frames", 4, 1, 100};
    ofParameter<bool> bSaveAccumToDisk = {"Enable Save To disk", false};
    ofParameter<size_t> drawAccumMax = {"Draw Accum Max", 10, 0, 100};
    
    ofParameter<uint8_t> accumulateByMode = {"Accumulate by", 0, 0, 2};
    
    ofParameter<bool> bDrawAccum = {"Draw Accumulated", true};
    
    /// AdvancedReturnMode
    /// If enabled, advanced return mode will add arrays to outputs showing some
    /// additional results or info of the SLAM algorithm such as :
    ///  - Trajectory : matching summary, localization error summary
    ///  - Output transformed frame : saliency, planarity, intensity gap, keypoint validity
    ///  - Extracted keypoints : ICP matching results
    ofParameter<bool> AdvancedReturnMode = {"Advanced Return Mode", false};
    
    /// OutputCurrentKeypoints
    /// If enabled, SLAM filter will output keypoints extracted from current
    /// frame. Otherwise, these filter outputs are left empty to save time.
    ofParameter<bool> OutputCurrentKeypoints = {"Output Current Keypoints", true};
    
    
    /// MapsUpdateStep
    /// Update keypoints maps only each MapsUpdateStep frame
    /// (ex: every frame, every 2 frames, 3 frames, ...)
    ofParameter<unsigned int> MapsUpdateStep = {"Maps Update Step", 1, 1, 10};
    
    
    
    /// OutputKeypointsInWorldCoordinates
    /// If disabled, return raw keypoints extracted from current frame in BASE
    /// coordinates, without undistortion. If enabled, return keypoints in WORLD
    /// coordinates, optionally undistorted if undistortion is activated.
    /// Only used if OutputCurrentKeypoints = true.
    ofParameter<bool> OutputKeypointsInWorldCoordinates = {"Output Keypoints In World Coordinates", true};
    
    
    ofParameter<bool> UseBlobs = {"UseBlobs", false};
    
    
    // Allows to choose which map keypoints to output
    ofParameter<uint8_t> OutputKeypointsMaps = {"Output Keypoints Maps", 1, 0, 3};
    
    ofParameter<uint8_t> undistortionMode = {"UndistortionMode",  0, 0, 3};
    
    ofParameter<uint8_t> mappingMode = {"MappingMode",  0, 0, 3};
    
    
    ofParameter<bool> TimestampFirstPacket = {"Timestamp First Packet", true};
    // ---------------------------------------------------------------------------
    //   Optimization parameters
    // ---------------------------------------------------------------------------
    
    ofParameter< bool> TwoDMode = {"2D Mode", false};
    
    ofParameter<uint8_t> egoMotionMode = {"EgoMotionMode",  0, 0, 4};
    
    // Get/Set EgoMotion
    ofParameter< unsigned int> EgoMotionLMMaxIter = {"Ego Motion LM Max Iter", 1, 0, 100};
    ofParameter< unsigned int> EgoMotionICPMaxIter = {"Ego Motion ICP Max Iter", 1, 0, 100};
    ofParameter< double> EgoMotionMaxNeighborsDistance = {"Ego Motion Max Neighbors Distance", 0, 0, 1000};
    ofParameter< unsigned int> EgoMotionEdgeNbNeighbors = {"Ego Motion Edge Nb Neighbors", 0,0,100};
    ofParameter< unsigned int> EgoMotionEdgeMinNbNeighbors = {"Ego Motion Edge Min Nb Neighbors", 0, 0, 100};
    ofParameter< double> EgoMotionEdgeMaxModelError = {"Ego Motion Edge Max Model Error", 0, 1, 10};
    ofParameter< unsigned int> EgoMotionPlaneNbNeighbors = {"Ego Motion Plane Nb Neighbors", 0, 0, 1000};
    ofParameter< double> EgoMotionPlanarityThreshold = {"Ego Motion Planarity Threshold", 0, 0, 10};
    ofParameter< double> EgoMotionPlaneMaxModelError = {"Ego Motion Plane Max Model Error", 0, 0,  10} ;
    ofParameter< double> EgoMotionInitSaturationDistance = {"Ego Motion Init Saturation Distance", 0, 0, 1000};
    ofParameter< double> EgoMotionFinalSaturationDistance = {"Ego Motion Final Saturation Distance", 0, 0, 1000};
    // Get/Set Localization
    ofParameter< unsigned int> LocalizationLMMaxIter = {"Localization LM Max Iter", 1, 0, 100};;
    ofParameter< unsigned int> LocalizationICPMaxIter = {"Localization ICP Max Iter", 1, 0, 100};
    ofParameter< double> LocalizationMaxNeighborsDistance = {"Localization Max Neighbors Distance", 0,0,1000};
    ofParameter< unsigned int> LocalizationEdgeNbNeighbors = {"Localization Edge Nb Neighbors", 0,0,100};
    ofParameter< unsigned int> LocalizationEdgeMinNbNeighbors = {"Localization Edge Min Nb Neighbors", 0,0,100};
    ofParameter< double> LocalizationEdgeMaxModelError = {"Localization Edge Max Model Error", 0,0,10};
    ofParameter< unsigned int> LocalizationPlaneNbNeighbors = {"Localization Plane Nb Neighbors", 0,0,100};
    ofParameter< double> LocalizationPlanarityThreshold = {"Localization Planarity Threshold", 0,0,10};
    ofParameter< double> LocalizationPlaneMaxModelError = {"Localization Plane Max Model Error", 0,0,10};
    ofParameter< unsigned int> LocalizationBlobNbNeighbors = {"Localization Blob Nb Neighbors", 0,0,100};
    ofParameter< double> LocalizationInitSaturationDistance = {"Localization Init Saturation Distance", 0,0,1000};
    ofParameter< double> LocalizationFinalSaturationDistance = {"Localization Final Saturation Distance", 0,0,1000};
    ofParameter< double> WheelOdomWeight = {"Wheel Odom Weight", 0,0,1};
    ofParameter< bool> WheelOdomRelative = {"Wheel Odom Relative", false};
    ofParameter< double> GravityWeight = {"Gravity Weight", 0, 0, 1};
    ofParameter< double> SensorTimeOffset = {"Sensor Time Offset", 0,0, 4000};
    ofParameter< double> SensorTimeThreshold = {"Sensor Time Threshold", 0,0, 4000};
    ofParameter< unsigned int> SensorMaxMeasures = {"Sensor Max Measures", 0, 0, 10000};
    
    // ---------------------------------------------------------------------------
    //   Key frames
    // ---------------------------------------------------------------------------
    
    ofParameter< double> KfDistanceThreshold = {"Keyframe Distance Threshold",0, 0, 5000};
    ofParameter< double> KfAngleThreshold = {"Keyframe Angle Threshold",0, 0, 180};
    
    // ---------------------------------------------------------------------------
    //   Rolling Grid
    // ---------------------------------------------------------------------------
    
    ofParameter<uint8_t> samplingModeEdges = {"Sampling Mode Edges",  0, 0, 5};
    ofParameter<uint8_t> samplingModePlanes = {"Sampling Mode Planes",  0, 0, 5};
    ofParameter<uint8_t> samplingModeBlobs = {"Sampling Mode Blobs",  0, 0, 5};
    
    ofParameter< double> VoxelGridDecayingThreshold = {"Voxel Grid Decaying Threshold", 0, 0, 100};
    ofParameter<int> VoxelGridSize = {"Voxel Grid Size", 0, 0, 1000};
    ofParameter<double> VoxelGridResolution = {"Voxel Grid Resolution", 0,0, 100};
    ofParameter<unsigned int> VoxelGridMinFramesPerVoxel = {"Voxel Grid Min Frames Per Voxel",0,0, 10};

    ofParameter<float> LeafSizeEdges = {"Leaf Size Edges", 0.30, 0, 10};
    ofParameter<float> LeafSizePlanes = {"Leaf Size Planes", 0.60, 0, 10};
    ofParameter<float> LeafSizeBlobs = {"Leaf Size Blobs", 0.30, 0, 10};
    

    // ---------------------------------------------------------------------------
    //   Confidence estimator parameters
    // ---------------------------------------------------------------------------
    
    // Internal variable to store overlap sampling ratio when advanced return mode is disabled.
    ofParameter< float> OverlapSamplingRatio = {"Overlap Sampling Ratio", 0.25};
    
    
    // Motion constraints

    ofParameter<float> linearAccLimit = {"Linear Acc", FLT_MAX, -FLT_MAX, FLT_MAX};
    ofParameter<float> angularAccLimit = {"Angular Acc", FLT_MAX, -FLT_MAX, FLT_MAX};
    ofParameter<float> linearVelLimit = {"Linear Vel", FLT_MAX, -FLT_MAX, FLT_MAX};
    ofParameter<float> angularVelLimit = {"Angular Vel", FLT_MAX, -FLT_MAX, FLT_MAX};
    
    
    // Internal variable to store window time to estimate local velocity when advanced return mode is disabled.
    
    ofParameter< float> TimeWindowDuration = {"Time Window Duration", 0.5, 0, 5};
    


    // ---------------------------------------------------------------------------
    //   Drawing parameters
    // ---------------------------------------------------------------------------
    

    ofParameter<bool> bDrawLidarFeed = {"Draw Lidar Feed", true};
    ofParameter<bool> bDrawTrajectoryLine = {"Draw Trajectory Line", true};
    ofParameter<bool> bDrawRegisteredMap = {"Draw Registered Map", true};
    ofParameter<bool> bDrawEdgeMap = {"Draw Edge Map", true};
    ofParameter<bool> bDrawPlanarMap = {"Draw Planar Map", true};
    ofParameter<bool> bDrawBlobMap = {"Draw Blob Map", true};
    ofParameter<bool> bDrawEdgeKeypoints = {"Draw Edge KeyPoints", true};
    ofParameter<bool> bDrawPlanarKeypoints = {"Draw Planar KeyPoints", true};
    ofParameter<bool> bDrawBlobKeypoints = {"Draw Blob KeyPoints", true};
    

    // ---------------------------------------------------------------------------
    //   Parameter Groups
    // ---------------------------------------------------------------------------

    /// parameter group which contains all the parameters.
    
    ofParameterGroup accumulateParams = {"Accumulate Registered"};
    ofParameterGroup optimizationParams = {"Optimization"};
    ofParameterGroup egoMotionParams = {"Ego Motion"};
    ofParameterGroup localizationParams = {"Localization"};
    ofParameterGroup keyframesParams = {"Keyframes"};
    ofParameterGroup rollingGridParams = {"Rolling Grid"};
    ofParameterGroup confidenceParams = {"Confidence"};
    ofParameterGroup motionConstraintParams = {"Motion Constraint"};    

    ofParameterGroup samplingModesParams= {"Sampling modes"};
    
    ofParameterGroup drawParams = {"Draw"};
    
    ofxPanel gui;

    
    ofParameter<bool> bDrawColorsGui = {"Show Colors", false};
    
    ofParameter<ofColor> TrajectoryLineColor;
    ofParameter<ofColor> RegisteredMapColor;
    ofParameter<ofColor> EdgeMapColor;
    ofParameter<ofColor> PlanarMapColor;
    ofParameter<ofColor> BlobMapColor;
    ofParameter<ofColor> EdgeKeypointsColor;
    ofParameter<ofColor> PlanarKeypointsColor;
    ofParameter<ofColor> BlobKeypointsColor;
    
    ofxPanel guiColors;

//    ofParameter<bool> destaggerSignal = {"Destagger Signal", false};

    // 0: print errors, warnings or one time info
    // 1: 0 + frame number, total frame processing time
    // 2: 1 + extracted features, used keypoints, localization variance, ego-motion and localization summary
    // 3: 2 + sub-problems processing duration
    // 4: 3 + ceres optimization summary
    // 5: 4 + logging/maps memory usage
    ofParameter<uint8_t> verboseLevel = {"Verbose Level", 3, 0, 5};

protected:

    shared_ptr<ofxDropdown_<uint8_t>> makeDropdown(const vector<string>& names, ofParameter<uint8_t> & param , bool bMultiselect = false, bool bCollapseOnSelect = true);

    shared_ptr<ofxDropdown_<uint8_t>> OutputKeypointsMapsDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> undistortionModeDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> mappingModeDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> egoMotionModeDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> samplingModeEdgesDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> samplingModePlanesDropdown = nullptr;
    shared_ptr<ofxDropdown_<uint8_t>> samplingModeBlobsDropdown = nullptr;
    
    shared_ptr<ofxDropdown_<uint8_t>> accumulateByDropdown = nullptr;
    
    
    void makeDropdowns();
 
    void initAndAddColorParam(ofParameter<ofColor>& param, string name,const ofColor& color);
    
};


#endif /* ofxLidarSlamParameters_hpp */



