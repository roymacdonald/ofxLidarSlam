//
//  ofxLidarSlamParameters.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/10/22.
//

#include "ofxLidarSlamParameters.h"

ofxLidarSlamParameters::ofxLidarSlamParameters(){
    
    string settingsFile = "Slam_settings.json";
    gui.setup("Lidar SLAM",settingsFile);
    gui.setWidthElements(300);
    gui.add(bEnableSlam);
    gui.add(NbThreads);
    gui.add(bUseImuData);
    gui.add(bDrawColorsGui);
    gui.add(destaggerSignal);
    
    gui.add(verboseLevel);
//    gui.add(bUseMillimeters);
    gui.add(reset);
    makeDropdowns();
    
    gui.add(AdvancedReturnMode);
    gui.add(OutputCurrentKeypoints);
    gui.add(MapsUpdateStep);
    gui.add(OutputKeypointsInWorldCoordinates);
//    gui.add(OutputKeypointsMaps);
    gui.add(UseBlobs);
    
    gui.add(TimestampFirstPacket);

    gui.add(OutputKeypointsMapsDropdown.get());

    gui.add(undistortionModeDropdown.get());
    gui.add(mappingModeDropdown.get());
    gui.add(egoMotionModeDropdown.get());
    gui.add(samplingModesParams);
    auto& g = gui.getGroup(samplingModesParams.getName());
    g.add(samplingModeEdgesDropdown.get());
    g.add(samplingModePlanesDropdown.get());
    g.add(samplingModeBlobsDropdown.get());
    
    
    drawParams.add(bDrawLidarFeed);
    drawParams.add(bDrawTrajectoryLine);
    drawParams.add(bDrawRegisteredMap);
    drawParams.add(bDrawEdgeMap);
    drawParams.add(bDrawPlanarMap);
    drawParams.add(bDrawBlobMap);
    drawParams.add(bDrawEdgeKeypoints);
    drawParams.add(bDrawPlanarKeypoints);
    drawParams.add(bDrawBlobKeypoints);
    
    
    gui.add(drawParams);
    
    
    optimizationParams.add(TwoDMode);
    // Get/Set EgoMotion
    egoMotionParams.add(EgoMotionLMMaxIter);
    egoMotionParams.add(EgoMotionICPMaxIter);
    egoMotionParams.add(EgoMotionMaxNeighborsDistance);
    egoMotionParams.add(EgoMotionEdgeNbNeighbors);
    egoMotionParams.add(EgoMotionEdgeMinNbNeighbors);
    egoMotionParams.add(EgoMotionEdgeMaxModelError);
    egoMotionParams.add(EgoMotionPlaneNbNeighbors);
    egoMotionParams.add(EgoMotionPlanarityThreshold);
    egoMotionParams.add(EgoMotionPlaneMaxModelError);
    egoMotionParams.add(EgoMotionInitSaturationDistance);
    egoMotionParams.add(EgoMotionFinalSaturationDistance);
    
    optimizationParams.add(egoMotionParams);
    
    // Get/Set Localization
    localizationParams.add(LocalizationLMMaxIter);
    localizationParams.add(LocalizationICPMaxIter);
    localizationParams.add(LocalizationMaxNeighborsDistance);
    localizationParams.add(LocalizationEdgeNbNeighbors);
    localizationParams.add(LocalizationEdgeMinNbNeighbors);
    localizationParams.add(LocalizationEdgeMaxModelError);
    localizationParams.add(LocalizationPlaneNbNeighbors);
    localizationParams.add(LocalizationPlanarityThreshold);
    localizationParams.add(LocalizationPlaneMaxModelError);
    localizationParams.add(LocalizationBlobNbNeighbors);
    localizationParams.add(LocalizationInitSaturationDistance);
    localizationParams.add(LocalizationFinalSaturationDistance);
    localizationParams.add(WheelOdomWeight);
    localizationParams.add(WheelOdomRelative);
    localizationParams.add(GravityWeight);
    localizationParams.add(SensorTimeOffset);
    localizationParams.add(SensorTimeThreshold);
    localizationParams.add(SensorMaxMeasures);
    
    optimizationParams.add(localizationParams);
    
    gui.add(optimizationParams);
    
    
    // Key frames
    keyframesParams.add(KfDistanceThreshold);
    keyframesParams.add(KfAngleThreshold);
    
    gui.add(keyframesParams);
    
    
    
    
    
    rollingGridParams.add(VoxelGridDecayingThreshold);
    rollingGridParams.add(VoxelGridSize);
    rollingGridParams.add(VoxelGridResolution);
    rollingGridParams.add(VoxelGridMinFramesPerVoxel);
    rollingGridParams.add(LeafSizeEdges);
    rollingGridParams.add(LeafSizePlanes);
    rollingGridParams.add(LeafSizeBlobs);
    
    gui.add(rollingGridParams);
    
    
    
    
    confidenceParams.add(OverlapSamplingRatio);
    
    
    // Motion constraints
    
    
    motionConstraintParams.add(linearAccLimit);
    motionConstraintParams.add(angularAccLimit);
    motionConstraintParams.add(linearVelLimit);
    motionConstraintParams.add(angularVelLimit);
    
    
    confidenceParams.add(motionConstraintParams);
    confidenceParams.add(TimeWindowDuration);
    
    gui.add(confidenceParams);
    
    
    if(ofFile::doesFileExist(settingsFile)){
        gui.loadFromFile(settingsFile);
    }
    
    string colorsSettingsFile = "colorSettings.json";
    
    guiColors.setup("Colors", colorsSettingsFile);
    
    initAndAddColorParam(TrajectoryLineColor, "Trajectory Line", ofColor::red);
    initAndAddColorParam(RegisteredMapColor, "Registered Map", ofColor::yellow);
    initAndAddColorParam(EdgeMapColor, "Edge Map", ofColor::cyan);
    initAndAddColorParam(PlanarMapColor, "Planar Map", ofColor::magenta);
    initAndAddColorParam(BlobMapColor, "Blob Map", ofColor::green);
    initAndAddColorParam(EdgeKeypointsColor, "Edge Keypoints", ofColor::fireBrick);
    initAndAddColorParam(PlanarKeypointsColor, "Planar Keypoints", ofColor::blueSteel);
    initAndAddColorParam(BlobKeypointsColor, "Blob Keypoints", ofColor::aliceBlue);
        
    
    if(ofFile::doesFileExist(colorsSettingsFile)){
        gui.loadFromFile(colorsSettingsFile);
    }
}

void ofxLidarSlamParameters::draw(){
    gui.draw();
    if(bDrawColorsGui){
        guiColors.draw();
    }
}

void ofxLidarSlamParameters::initAndAddColorParam(ofParameter<ofColor>& param, string name, const ofColor& color){
    param.set(name, color, ofColor(0,0,0,0), ofColor(255,255,255,255));
    guiColors.add(param);
}

shared_ptr<ofxDropdown_<uint8_t>> ofxLidarSlamParameters::makeDropdown(const vector<string>& names, ofParameter<uint8_t> & param , bool bMultiselect , bool bCollapseOnSelect){
    map<uint8_t, string> displayNames;
    
    for(uint8_t i = 0; i < names.size(); i++){
        displayNames[i] = names[i];
    }
    
    auto dropdown = make_shared<ofxDropdown_<uint8_t>>(param, displayNames);
    
    if(!bMultiselect) dropdown->disableMultipleSelection();
    if(bCollapseOnSelect) dropdown->enableCollapseOnSelection();
    dropdown->setDropDownPosition(ofxDropdown_<uint8_t>::DD_LEFT);
    
    return dropdown;
    
}



void ofxLidarSlamParameters::makeDropdowns(){
    
    OutputKeypointsMapsDropdown = makeDropdown({
        "NONE",
        "FULL_MAPS",
        "SUB_MAPS"}, OutputKeypointsMaps);
    
    undistortionModeDropdown = makeDropdown({
        "NONE" ,
        "ONCE" ,
        "REFINED" }, undistortionMode);
    
    mappingModeDropdown = makeDropdown({
        "NONE" ,
        "ADD_KPTS_TO_FIXED_MAP" ,
        "UPDATE" }, mappingMode);
    
    egoMotionModeDropdown = makeDropdown({
        "NONE" ,
        "MOTION_EXTRAPOLATION" ,
        "REGISTRATION" ,
        "MOTION_EXTRAPOLATION_AND_REGISTRATION"}
        , egoMotionMode);
    
    vector<string> sampling_names = {
        "FIRST" ,
        "LAST" ,
        "MAX_INTENSITY" ,
        "CENTER_POINT" ,
        "CENTROID"};
    
    samplingModeEdgesDropdown = makeDropdown(sampling_names , samplingModeEdges);
    
    samplingModePlanesDropdown = makeDropdown(sampling_names, samplingModePlanes);
    
    samplingModeBlobsDropdown = makeDropdown(sampling_names, samplingModeBlobs);
    
    
    
    
}
