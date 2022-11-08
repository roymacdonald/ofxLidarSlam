//
//  ofxLidarSlamParameters.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/10/22.
//

#include "ofxLidarSlamParameters.h"
#include "LidarSlam/Enums.h"
#include "ofxLidarSlam.h"

void ofxLidarSlamParameters::setup(LidarSlam::SpinningSensorKeypointExtractor * keypointExtractor){
    
    keypointExtractorParams = make_unique<ofxSpinningSensorKeypointExtractor>(keypointExtractor);
    
    makeDropdowns();
    setupGuiGroups();

    setupGui();
    
//    loadGui(&gui, settingsFile);
    
    setupColorsGui();
    setGuisPositions();
    
    windowSizeListener = ofEvents().windowResized.newListener([&](ofResizeEventArgs& args){
        setGuisPositions();
    });
    setGuisPositions();
    setupTooltips();
}

void ofxLidarSlamParameters::setupColorsGui(){
    string colorsSettingsFile = "colorSettings.json";
    
    guiColors.setup("Colors", colorsSettingsFile);
    
    initAndAddColorParam(TrajectoryLineColor, "Trajectory Line", ofColor::red);
    initAndAddColorParam(RegisteredMapColor, "Registered Map", ofColor::yellow);
    
    for(auto k : ofxLidarSlam::UsableKeypoints){
        guiColors.add(*mapColor[k].get());
    }

    for(auto k : ofxLidarSlam::UsableKeypoints){
        guiColors.add(*keypointColor[k].get());
    }
    loadGui(&guiColors, colorsSettingsFile);
}

void ofxLidarSlamParameters::setupGuiGroups(){
    selectedTab.setName("selected tab");
    tabs = make_unique<ofxGuiIntTabs>(selectedTab);
    
    tabs->enableKeys();
    
    guiGroups.resize(GuiTypesNames.size());
    for(size_t i = 0; i < GuiTypesNames.size(); i ++){
        guiGroups[i] = make_unique<ofxGuiGroup>();
        guiGroups[i]->setup(GuiTypesNames[i]);
        tabs->add(i, GuiTypesNames[i]);
    }
    
    selectedTabListener = selectedTab.newListener([&](int& i){
        setCurrentGuiGroup((GuiTypes)i);
    });
    advancedReturnListener = AdvancedReturnMode.newListener([&](bool& i){
        if(current_gui == GUI_KEYFRAMES){
            setCurrentGuiGroup(GUI_KEYFRAMES, true);
        }
    });
    
    
    
    
    guiGroups[GUI_INPUT]->add(undistortionModeDropdown.get());
    
    guiGroups[GUI_DRAW]->add(bDrawColorsGui);
    
    accumulateParams.add(saveMaps);
    accumulateParams.add(saveMarkersOnly);
//    accumulateParams.add(saveMeshes);
    accumulateParams.add(accumEveryDistance);
    accumulateParams.add(accumEveryFrames);
    accumulateParams.add(bSaveAccumToDisk);
    accumulateParams.add(drawAccumMax);
    
    
    guiGroups[GUI_OUTPUT]->add(accumulateParams);
    guiGroups[GUI_OUTPUT]->add(accumulateByDropdown.get());
    
    
    size_t n = ofxLidarSlam::UsableKeypoints.size();
    bDrawMaps.resize(n);
    bDrawKeypoints.resize(n);
    keypointColor.resize(n);
    mapColor.resize(n);
    
    int i = 0;
    for(auto k : ofxLidarSlam::UsableKeypoints){
        bDrawMaps[k] = make_unique<ofParameter<bool>>("Draw " + LidarSlam::KeypointTypeNames.at(k) + " Map", true);
        bDrawKeypoints[k] = make_unique<ofParameter<bool>>("Draw " + LidarSlam::KeypointTypeNames.at(k) + " Keypoints", true);
        
        ofColor c;
        c.setHsb( (i*2) * 255.0f/ ofxLidarSlam::UsableKeypoints.size()*2 , 255, 255);
        mapColor[k] = make_unique<ofParameter<ofColor>>(LidarSlam::KeypointTypeNames.at(k) + " Map Color", c,ofColor(0,0,0,0), ofColor(255,255,255,255));
        c.setHsb( (i*2 + 1) * 255.0f/ ofxLidarSlam::UsableKeypoints.size()*2 , 255, 255);
        keypointColor[k] = make_unique<ofParameter<ofColor>>(LidarSlam::KeypointTypeNames.at(k) + " Keypoint Color", c,ofColor(0,0,0,0), ofColor(255,255,255,255));
        i++;
    }
    
    guiGroups[GUI_KEYPOINTS]->add(OutputCurrentKeypoints);
    guiGroups[GUI_KEYPOINTS]->add(MapsUpdateStep);
    guiGroups[GUI_KEYPOINTS]->add(OutputKeypointsInWorldCoordinates);
//    guiGroups[GUI_KEYPOINTS]->add(UseBlobs);
    guiGroups[GUI_KEYPOINTS]->add(OutputKeypointsMapsDropdown.get());
    //    gui.add(TimestampFirstPacket);
//    guiGroups[GUI_KEYPOINTS]->add(samplingModesParams);
//    auto& g = gui.getGroup(samplingModesParams.getName());
    for(auto& k : ofxLidarSlam::UsableKeypoints){
        guiGroups[GUI_KEYPOINTS]->add(samplingModeDropdown[k].get());
    }
//    guiGroups[GUI_KEYPOINTS]->add(samplingModePlanesDropdown.get());
//    guiGroups[GUI_KEYPOINTS]->add(samplingModeBlobsDropdown.get());
    
    
    
    rollingGridParams.add(VoxelGridDecayingThreshold);
    rollingGridParams.add(VoxelGridSize);
    rollingGridParams.add(VoxelGridResolution);
    rollingGridParams.add(VoxelGridMinFramesPerVoxel);
    for(auto& k : ofxLidarSlam::UsableKeypoints){
        rollingGridParams.add(LeafSize[k]);
    }
//    rollingGridParams.add(LeafSizePlanes);
//    rollingGridParams.add(LeafSizeBlobs);
    
    guiGroups[GUI_KEYPOINTS]->add(rollingGridParams);
    
    
    guiGroups[GUI_KEYPOINTS]->add(keypointExtractorParams->parameters);
    
    
    
    drawParams.add(bUseDepthTest);
    drawParams.add(bDrawLidarFeed);
    drawParams.add(bDrawTrajectoryLine);
    drawParams.add(bDrawRegisteredMap);

    for(auto& p : bDrawMaps){
        if(p){
            drawParams.add(*p.get());
        }
    }
    for(auto& p : bDrawKeypoints){
        if(p){
            drawParams.add(*p.get());
        }
    }
        
    drawParams.add(bDrawMarkers);
    drawParams.add(bDrawAccum);
    
    guiGroups[GUI_DRAW]->add(drawParams);
    
    
//    optimizationParams.add(TwoDMode);
    // Get/Set EgoMotion
    egoMotionParams.add(TwoDMode);
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
    
    
    guiGroups[GUI_MOTION]->add(egoMotionModeDropdown.get());
    guiGroups[GUI_MOTION]->add(egoMotionParams);
    
    
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
//    localizationParams.add(WheelOdomWeight);
//    localizationParams.add(WheelOdomRelative);
    localizationParams.add(GravityWeight);
    localizationParams.add(SensorTimeOffset);
    localizationParams.add(SensorTimeThreshold);
    localizationParams.add(SensorMaxMeasures);
    
    guiGroups[GUI_LOCALIZATION]->add(localizationParams);
    
//    gui.add(optimizationParams);
    
    
    // Key frames
    
//    guiGroups[GUI_KEYFRAMES]->add(OutputKeypointsMaps);
    guiGroups[GUI_KEYFRAMES]->add(mappingModeDropdown.get());

    guiGroups[GUI_KEYFRAMES]->add(KfDistanceThreshold);
    guiGroups[GUI_KEYFRAMES]->add(KfAngleThreshold);
    guiGroups[GUI_KEYFRAMES]->add(AdvancedReturnMode);
    
    advancedReturnParams.add(TimeWindowDuration);
    advancedReturnParams.add(OverlapSamplingRatio);
    

    
    
    
    
    // Motion constraints
    
    
    motionConstraintParams.add(linearAccLimit);
    motionConstraintParams.add(angularAccLimit);
    motionConstraintParams.add(linearVelLimit);
    motionConstraintParams.add(angularVelLimit);

    guiGroups[GUI_MOTION]->add(motionConstraintParams);

        
}
void ofxLidarSlamParameters::setupGui(){
    
    gui.setup("Lidar SLAM",settingsFile);
    gui.setWidthElements(300);
    gui.add(bEnableSlam);
    gui.add(NbThreads);
    gui.add(bUseImuData);
    gui.add(verboseLevelDropdown.get());
    gui.add(reset);
    gui.add(bTooltipsEnabled);
//    gui.add(AdvancedReturnMode);

    gui.add(tabs.get());
//    gui.add(TimestampFirstPacket);
    if((int)current_gui < guiGroups.size()){
        gui.add(guiGroups[(int)current_gui].get());
        if(current_gui == GUI_KEYFRAMES){
            if(AdvancedReturnMode.get()){
                gui.add(advancedReturnParams);
            }
        }
    }
    setGuisPositions();
    
    
}

void ofxLidarSlamParameters::setupTooltips(){
    ofJson json;
    string tooltipFilepath = "tooltips_slam.json";
    
    
    for(size_t i = 0; i <GuiTypesNames.size(); i++){
        groups_tooltips.emplace_back(make_unique<ofxGuiTooltip>());
        groups_tooltips[i]->registerGui(guiGroups[i].get(), tooltipFilepath, "/group/"+GuiTypesNames[i]);
        groups_tooltips[i]->disable();
    }
    main_tooltip.registerGui(&gui, tooltipFilepath);
    
    tooltipsListener = bTooltipsEnabled.newListener([&](bool&){
   
        for(auto & g : groups_tooltips){
            if(bTooltipsEnabled.get()){
                g->enable();
            }else{
                g->disable();
            }
        }
        if(bTooltipsEnabled.get()){
            main_tooltip.enable();
        }else{
            main_tooltip.disable();
        }
        
    });
    
}

void ofxLidarSlamParameters::draw(){
    gui.draw();
    main_tooltip.draw();
    groups_tooltips[current_gui]->draw();
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
        "REFINED",
        "EXTERNAL" }, undistortionMode);
    
    mappingModeDropdown = makeDropdown({
        "NONE" ,
        "ADD_KPTS_TO_FIXED_MAP" ,
        "UPDATE" }, mappingMode);
    
    egoMotionModeDropdown = makeDropdown({
        "NONE",
        "MOTION_EXTRAPOLATION",
        "REGISTRATION",
        "MOTION_EXTRAPOLATION_AND_REGISTRATION",
        "EXTERNAL",
        "EXTERNAL_OR_MOTION_EXTRAPOLATION"
        }
        , egoMotionMode);
    
    accumulateByDropdown = makeDropdown({"NONE","DISTANCE", "FRAMES"}, accumulateByMode);
    
    vector<string> sampling_names = {
        "FIRST" ,
        "LAST" ,
        "MAX_INTENSITY" ,
        "CENTER_POINT" ,
        "CENTROID"};
    
    
    verboseLevelDropdown = makeDropdown({"0","1","2","3","4","5"}, verboseLevel);
    
    
    auto numKeypoints = ofxLidarSlam::UsableKeypoints.size();
    
    samplingMode.resize(numKeypoints);
    LeafSize.resize(numKeypoints);
    samplingModeDropdown.resize(numKeypoints);
    
    for(auto& d: ofxLidarSlam::UsableKeypoints){
        auto name = LidarSlam::KeypointTypeNames.at((LidarSlam::Keypoint)d);
        samplingMode[d].set("Sampling Mode " + name,  0, 0, 5);
        samplingModeDropdown[d] = makeDropdown(sampling_names , samplingMode[d]);
        LeafSize[d].set("Leaf Size "+ name, 0.30, 0, 10);
    }
    LeafSize[LidarSlam::PLANE] = 0.60;
    
    
    
    
    
    
}

string ofxLidarSlamParameters::getCurrentGuiFilename(){
    return "settings_"+GuiTypesNames[current_gui]+".json";
}

void ofxLidarSlamParameters::setTab(int tabIndex){
    if(tabIndex >= 0 && tabIndex < GuiTypesNames.size()){
        selectedTab = tabIndex;
    }
}

void ofxLidarSlamParameters::setCurrentGuiGroup(GuiTypes group, bool bForceUpdate){
    if(group != current_gui || bForceUpdate){
//        cout <<"ofxLidarSlamParameters::setCurrentGuiGroup " << GuiTypesNames[group] << " -> " << GuiTypesNames[current_gui] <<endl;
        if(current_gui < groups_tooltips.size() && groups_tooltips[current_gui]){
            groups_tooltips[current_gui]->disable();
        }
        guiGroups[current_gui]->saveToFile(getCurrentGuiFilename());
        current_gui = group ;
        setupGui();
        loadGui(guiGroups[current_gui].get(), getCurrentGuiFilename());
        if(current_gui < groups_tooltips.size() && groups_tooltips[current_gui]){
            groups_tooltips[current_gui]->enable();
        }
    }
}
bool ofxLidarSlamParameters::loadGui(ofxGuiGroup* g, const string& filepath){
    if(!g)return false;
    if(ofFile::doesFileExist(filepath)){
        g->loadFromFile(filepath);
        return true;
    }
    return false;
}

void ofxLidarSlamParameters::setGuisPositions(){
    gui.setPosition(ofGetWidth() - gui.getShape().width, 0);
    guiColors.setPosition(gui.getShape().getMinX() - guiColors.getWidth(), 0);
}
