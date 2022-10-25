//
//  ofxLidarSlam.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/5/22.
//

#include "ofxLidarSlam.h"
// PCL
#include <pcl/common/transforms.h>

//#include "ofxTimeMeasurements.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <LidarSlam/Utilities.h>

#define addListenerMacro(param,type) \
listeners.push(params.param.newListener([&](type & _arg){\
this->SlamAlgo->Set##param(_arg);\
}));

//#define GET_MAP(map, type) if(params.bDraw##map) {PclToOf(this->SlamAlgo->GetMap(type), results.map);}

//#define GET_SUBMAP(map, type) if(params.bDraw##map) {PclToOf(this->SlamAlgo->GetTargetSubMap(type), results.map);}

//#define GET_KEYPOINTS(keypoints, type) if(params.bDraw##keypoints) {PclToOf(this->SlamAlgo->GetKeypoints(type, params.OutputKeypointsInWorldCoordinates.get()), results.keypoints);}




#define SetDefaultValueMacro(param) params.param  = this->SlamAlgo->Get##param();

#define PrintMacro(param) ss << #param  << " : " << this->SlamAlgo->Get##param() << "\n";

void PclToOf( LidarSlam::Slam::PointCloud::Ptr pc, ofVboMesh& mesh){
    size_t np = pc->size();
    mesh.setMode(OF_PRIMITIVE_POINTS);
    auto & v = mesh.getVertices();
    v.resize(np);
    for(size_t i = 0; i < np; i++){
        auto& pp = pc->at(i);
        v[i].x = pp.x;
        v[i].y = pp.y;
        v[i].z = pp.z;
    }
}
void AddPclToOf( LidarSlam::Slam::PointCloud::Ptr pc, ofVboMesh& mesh){
    size_t np = pc->size();
    mesh.setMode(OF_PRIMITIVE_POINTS);
    auto & v = mesh.getVertices();
    size_t startSize = v.size();
    v.resize(startSize + np);
    size_t n = startSize + np;
    for(size_t i = startSize; i < n; i++){
        auto& pp = pc->at(i-startSize);
        v[i].x = pp.x;
        v[i].y = pp.y;
        v[i].z = pp.z;
    }
}



//-----------------------------------------------------------------------------
ofxLidarSlam::ofxLidarSlam():pointShader("Points Shader")
{
 
    bMarkCurrent = false;
    SlamAlgo = std::make_unique<LidarSlam::Slam>();
    
    SlamAlgo->SetKeyPointsExtractor(getKeyPointExtractor().extractor);
    InitPose.setZero();
    
    setDefaults();
    SlamAlgo->SetVerbosity(5);
    
    this->reset();
    
    params.guiGroups[ofxLidarSlamParameters::GUI_KEYPOINTS]->add(getKeyPointExtractor().parameters);
    

    cam.setFarClip(1000000);
    cam.setNearClip(0);
#ifdef HAS_OFXGRABCAM
    cam.toggleFixUpDirectionEnabled();
#else
    cam.setRelativeYAxis(!cam.getRelativeYAxis());
#endif
    /// the SLAM library uses meters as its units
    pointShader.setRangeUnit(1);
    
    
 
}
//-----------------------------------------------------------------------------
ofxLidarSlam::~ofxLidarSlam()
{

    fromSlam.close();

    waitForThread(true);
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::setup(std::shared_ptr<ofxOuster>& lidar){
    this->lidar = lidar;
    this->lidar->disableRenderer();
    setParamsListeners();
    


    listeners.push(ofEvents().update.newListener(this, &ofxLidarSlam::_update));
    
    
    
    pointShader.loadShaderFromFiles(ofToDataPath("shaders/shader.vert"),ofToDataPath("shaders/shader.frag"));
//    pointShader.colorMap.makeColorMap("GRAY SCALE", {ofFloatColor::black, ofFloatColor::white});
    
//    pointShader.setGuiPosition( glm::vec2(ofGetWidth() - 600,10) );
    
    params.guiGroups[ofxLidarSlamParameters::GUI_DRAW]->add(&pointShader.gui);
    
    pointShader.colorMap.colorMapGui->colorMapDropdown->setDropDownPosition(ofxIntDropdown::DD_LEFT);
    
    startThread();
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::reset()
{
    this->SlamAlgo->Reset(true);
    this->SlamAlgo->ResetSensors(true);
    this->SlamAlgo->SetSensorMaxMeasures(10000);
    
    
    trajectory.clear();
    
    TrajectoryLine.clear();
    frameCount = 0;
    
    InitPose.setZero();
    
    
    this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::PoseToIsometry(this->InitPose));
    
    // Enable overlap computation only if advanced return mode is activated
    this->SlamAlgo->SetOverlapSamplingRatio((params.AdvancedReturnMode ? params.OverlapSamplingRatio.get() : 0.));
    // Enable motion limitation checks if advanced return mode is activated
    this->SlamAlgo->SetTimeWindowDuration(params.AdvancedReturnMode ? params.TimeWindowDuration.get() : 0.);
    // Log the necessary poses if advanced return mode is activated
    this->SlamAlgo->SetLoggingTimeout(params.AdvancedReturnMode ? 1.1 * params.TimeWindowDuration.get() : 0.);
    
    
    markedFrames.clear();
    
    lastSavedPose = {0,0,0};
    
    bMarkCurrent = false;
    
    slamResults.clear();
    
    
    
    
    
    
}

////-----------------------------------------------------------------------------
//void ofxLidarSlam::onLidarData(ouster::LidarScan &scan){
//
//    if(bMarkCurrent){
//        markedFrames.push_back(scan.frame_id);
//        bMarkCurrent = false;
//    }
//}
//-----------------------------------------------------------------------------
string ofxLidarSlam::createSaveDir(string timestamp){
    currentSavePath  = "RegisteredMeshes/"+ timestamp;
    
    {
        ofDirectory dir (currentSavePath);
        dir.create(true);
    }
    {
        ofDirectory dir (currentSavePath + "/metadata");
        dir.create(true);
    }
    return currentSavePath;
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::saveRegisteredMeshes(string timestamp){
    if(currentSavePath.empty()){
        createSaveDir(timestamp);
    }
    for(size_t i =0; i < trajectory.size(); i++){
        if((params.saveMarkersOnly.get() && trajectory[i].isMarker) || !params.saveMarkersOnly.get()){
            trajectory[i].save(currentSavePath, i, true);
        }
    }
    cout <<"saveRegisteredMeshes:: done!\n";

}


//-----------------------------------------------------------------------------
void ofxLidarSlam::processLidarData(ouster::LidarScan &scan, ofxLidarSlamResults & results){
            
//    if(bMarkCurrent){
//        markedFrames.push_back(scan.frame_id);
//        bMarkCurrent = false;
//    }
    if(params.bEnableSlam){
        //TS_START("processLidarData");
//        cout << "ofxLidarSlam::processLidarData\n";
        // Get frame first point time in vendor format
        double frameFirstPointTime = scan.timestamp()[0] * this->TimeToSecondsFactor;
        // Get first frame packet reception time
        
        double frameReceptionPOSIXTime = ofGetElapsedTimef();
        double absCurrentOffset = std::abs(this->SlamAlgo->GetSensorTimeOffset());
        double potentialOffset = frameFirstPointTime - frameReceptionPOSIXTime;
        // We exclude the first frame cause frameReceptionPOSIXTime can be badly set
        if (this->SlamAlgo->GetNbrFrameProcessed() > 0 && (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset))
            this->SlamAlgo->SetSensorTimeOffset(potentialOffset);
        
        //TS_START_NIF("getPointCloudFromLidar");
        auto pc = getPointCloudFromLidar(scan) ;
        if(pc == nullptr){
            ofLogError("ofxLidarSlam::onLidarData") << "Got nullptr pointcloud";
            return;
        }
        //TS_STOP_NIF("getPointCloudFromLidar");
        
        //TS_START("AddFrame");
        this->SlamAlgo->AddFrame(pc);
        //TS_STOP("AddFrame");
        
        //TS_START("AddPose");
        this->AddCurrentPoseToTrajectory();
        //TS_STOP("AddPose");
        // ===== SLAM frame and pose =====
        // Output : Current undistorted LiDAR frame in world coordinates
        
        PclToOf(this->SlamAlgo->GetRegisteredFrame(), results.RegisteredMap);
        
        
        //TS_START("SavePose");
        
//        saveMarkersOnly = {"Save Markers Only", true};
//        ofParameter<bool> saveMeshes = {"Save Meshes", false};
//
        
        if(params.accumulateByMode.get() != AccumulateMode::DONT_ACCUM){
            if(currentSavePath.empty()){
                createSaveDir( ofGetTimestampString());
            }
            
            auto& p = trajectory.back();
            glm::vec3 pose (p.x, p.y, p.z);
            bool bAccum = false;
            if(params.accumulateByMode.get() == AccumulateMode::DISTANCE){
                if (glm::distance(lastSavedPose, pose) >= params.accumEveryDistance.get()){
                    bAccum = true;
                }
            }else if(params.accumulateByMode.get() == AccumulateMode::FRAMES){
                if (frameCount % params.accumEveryFrames.get() == 0){
                    bAccum = true;
                }
            }
            
            if(bAccum){
                lastSavedPose = pose;
                p.isMarker = true;
                if(p.mesh.getVertices().size() <= 0){
                    p.copyMesh(results.RegisteredMap);
                }
                if(params.bSaveAccumToDisk){
                    p.save(currentSavePath, frameCount);
                    if(!params.saveMarkersOnly){
                        meshSaver.save(results.RegisteredMap, ofFilePath::join(currentSavePath, ofToString(frameCount, 4,0) + ".ply"));
                    }
                }
                
            }
        }
        //TS_STOP("SavePose");
        
        
//        cout << "Trajectory num verts: " << trajectory.back().mesh.getNumVertices() << endl;
        // Update the output maps if required or if the mode was changed
        bool updateMaps = params.OutputKeypointsMaps != this->PreviousMapOutputMode ||
        (this->SlamAlgo->GetNbrFrameProcessed() - 1) % params.MapsUpdateStep == 0;
        
        // The expected maps can be the whole maps or the submaps
        // If the maps is fixed by the user, the whole map and the submap are equal but the submap is outputed (faster)
        if (updateMaps && (params.OutputKeypointsMaps == OutputKeypointsMapsMode::FULL_MAPS && this->SlamAlgo->GetMapUpdate() != LidarSlam::MappingMode::NONE))
        {
            //TS_START("Get Full Maps");
//            GET_MAP(EdgeMap, LidarSlam::EDGE)
//            GET_MAP(PlanarMap, LidarSlam::PLANE)
//            GET_MAP(BlobMap, LidarSlam::BLOB)
            
            for(auto k : LidarSlam::KeypointTypes){
                if(params.bDrawMaps[k] && params.bDrawMaps[k]->get()) {
                    PclToOf(this->SlamAlgo->GetMap(k), results.maps[k]);
                }
            }
            
            
            
            
//            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
            //TS_STOP("Get Full Maps");
        }
        else if (updateMaps && (params.OutputKeypointsMaps == OutputKeypointsMapsMode::SUB_MAPS || this->SlamAlgo->GetMapUpdate() == LidarSlam::MappingMode::NONE))
        {
            
            
            for(auto k : LidarSlam::KeypointTypes){
                if(params.bDrawMaps[k] && params.bDrawMaps[k]->get()) {
                    PclToOf(this->SlamAlgo->GetTargetSubMap(k), results.maps[k]);
                }
            }
            
            
            
            //TS_START("Get Sub Maps");
//            GET_SUBMAP(EdgeMap, LidarSlam::EDGE)
//            GET_SUBMAP(PlanarMap, LidarSlam::PLANE)
//            GET_SUBMAP(BlobMap, LidarSlam::BLOB)
            //TS_STOP("Get Sub Maps");
            
//            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        }
        // If the output is disabled, reset it.
        else if (params.OutputKeypointsMaps != this->PreviousMapOutputMode && params.OutputKeypointsMaps == OutputKeypointsMapsMode::NONE)
        {
            for(auto & m : results.maps){
                m.clear();
            }
//            results.EdgeMap.clear();
//            results.PlanarMap.clear();
//            results.BlobMap.clear();
            
//            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        }
        
        this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        // ===== Extracted keypoints from current frame =====

        if (params.OutputCurrentKeypoints)
        {
            
            for(auto k : LidarSlam::KeypointTypes){
                if(params.bDrawKeypoints[k] && params.bDrawKeypoints[k]->get()) {
                    PclToOf(this->SlamAlgo->GetKeypoints(k, params.OutputKeypointsInWorldCoordinates.get()), results.keypoints[k]);
                }
            }
            
            
            
            //TS_START("Get KeyPoints");
//            GET_KEYPOINTS(EdgeKeypoints, LidarSlam::EDGE)
//            GET_KEYPOINTS(PlanarKeypoints, LidarSlam::PLANE)
//            GET_KEYPOINTS(BlobKeypoints, LidarSlam::BLOB)
            //TS_STOP("Get KeyPoints");
        }
        results.bValid = true;
        //TS_STOP("processLidarData");
    }
    frameCount++;
    
}
//-----------------------------------------------------------------------------
//void ofxLidarSlam::onImuData(ofxOusterIMUData& data){
//#ifndef OFX_OUSTER_USE_THREAD_CHANNELS
//    toSlamIMU.send(data);
//#endif
//}
//-----------------------------------------------------------------------------
void ofxLidarSlam::processImuData(ofxOusterIMUData & data){
    if(params.bUseImuData){
        LidarSlam::ExternalSensors::GravityMeasurement gravityMeasurement;
        gravityMeasurement.Time = data.accel_timestamp;
        gravityMeasurement.Acceleration.x() = data.accel.x;
        gravityMeasurement.Acceleration.y() = data.accel.y;
        gravityMeasurement.Acceleration.z() = data.accel.z;
        
        this->SlamAlgo->AddGravityMeasurement(gravityMeasurement);
    }
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::threadedFunction(){
    // wait until there's a new frame
    // this blocks the thread, so it doesn't use
    // the CPU at all, until a frame arrives.
    // also receive doesn't allocate or make any copies
    
    while(isThreadRunning()){
    
    ofxOusterIMUData imuData;
    
    if(!lidar) return;
    
    auto * lidarChannel = lidar->getLidarScanChannel();
    auto*  imuChannel = lidar->getImuChannel();
    if(imuChannel && imuChannel->tryReceive(imuData)){
        processImuData(imuData);
    }
    
    ouster::LidarScan scan;

    if(lidarChannel && lidarChannel->receive(scan)){

        ofxLidarSlamResults results;
        processLidarData(scan, results);
        auto scanCopy = scan;
        toRenderer.send(scanCopy);
        if(results.bValid){
            fromSlam.send(std::move(results));
        }
    }
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::_update(ofEventArgs &){

    ouster::LidarScan scan;
    bool bNewFrame = false;
    while(toRenderer.tryReceive(scan)){
        bNewFrame = true;
    }
    if(bNewFrame){
        if(renderer == nullptr && lidar){
            renderer = make_unique<ofxOusterRenderer>(lidar->getSensorInfo());
            renderer->setGuiPosition(params.gui.getShape().getBottomLeft() + glm::vec3(0,20,0));
        }
        if(renderer){
            renderer->render(scan);
        }
    }
    
    while(fromSlam.tryReceive(slamResults)){
        
    }
}




//-----------------------------------------------------------------------------
void ofxLidarSlam::setInitialPoseTranslation(double x, double y, double z)
{
    this->InitPose.x() = x;
    this->InitPose.y() = y;
    this->InitPose.z() = z;
    this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::PoseToIsometry(this->InitPose));
    
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::setInitialPoseRotation(double roll, double pitch, double yaw)
{
    this->InitPose(3) = roll;
    this->InitPose(4) = pitch;
    this->InitPose(5) = yaw;
    this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::PoseToIsometry(this->InitPose));
    
}


// =============================================================================
//   Useful helpers
// =============================================================================

//-----------------------------------------------------------------------------
void ofxLidarSlam::AddCurrentPoseToTrajectory()
{
    // Get current SLAM pose in WORLD coordinates
    LidarSlam::LidarState& currentState = this->SlamAlgo->GetLastState();
    
    
    trajectory.push_back(ofxLidarSlamTrajectoryPoint());
    auto& p = trajectory.back();
    if(trajectory.size()> 1){
        p.prev = &trajectory[trajectory.size()-2];
    }
    
    // Add position
    Eigen::Vector3d translation = currentState.Isometry.translation();
    
    p.x = translation.x();
    p.y = translation.y();
    p.z = translation.z();
    
    // Add orientation as quaternion
    Eigen::Quaterniond quaternion(currentState.Isometry.linear());
    
    p.quaternion.w = quaternion.w();
    p.quaternion.x = quaternion.x();
    p.quaternion.y = quaternion.y();
    p.quaternion.z = quaternion.z();
    
    
    // Add orientation as axis angle
    Eigen::AngleAxisd angleAxis(currentState.Isometry.linear());
    Eigen::Vector3d axis = angleAxis.axis();
    
    p.axisAngle[0] =  axis.x();
    p.axisAngle[1] =  axis.y();
    p.axisAngle[2] =  axis.z();
    p.axisAngle[3] = angleAxis.angle();
    
    
    // Add pose time and covariance
    p.time = currentState.Time;
    for(size_t i = 0; i < 36; i++){
        p.covariance[i] = currentState.Covariance.data()[i];
    }
    
    TrajectoryLine.addVertex(p.x, p.y, p.z);
    
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::markCurrentFrame(){
    bMarkCurrent = true;
}
//-----------------------------------------------------------------------------
LidarSlam::Slam::PointCloud::Ptr ofxLidarSlam::getPointCloudFromLidar(ouster::LidarScan & scan)
{
    if(lidar){
        
        LidarSlam::Slam::PointCloud::Ptr pc = make_shared<LidarSlam::Slam::PointCloud>();

                        
            size_t nbPoints = scan.w * scan.h;
            pc->reserve(nbPoints);
            
            auto timestamps = scan.timestamp();
            auto ids = scan.measurement_id();
            
            
            auto frameTimestamp = timestamps[(params.TimestampFirstPacket?0:(scan.w - 1))];
            pc->header.stamp = frameTimestamp / 1000.0f; // max time in microseconds
            pc->header.frame_id = this->SlamAlgo->GetBaseFrameId();
            pc->header.seq = frameCount;
            auto range = scan.field(ouster::sensor::RANGE);
            auto signal = scan.field(ouster::sensor::ChanField::SIGNAL);//.cast<double>();
            
            auto& lut = lidar->getLut();
            
            
            if (range.cols() * range.rows() != lut.direction.rows()){
                ofLogError("ofxLidarSlam::getPointCloudFromLidar") << "unexpected image dimensions";
                return nullptr;
            }
        
        // it is very important to convert the lidars data from whichever units it is into meters.
        // Otherwise SLAM will not work properly.
        float mm_to_meters_factor = 0.001;
            for(size_t y= 0; y< range.rows(); y++){
                for(size_t x= 0; x< range.cols(); x++){
                    size_t i = y * range.cols() + x;
                    double val = range(y, x);
                    
                    
                    LidarSlam::Slam::Point p;
                    
                    p.x = lut.direction(i, 0) * val * mm_to_meters_factor;
                    p.y = lut.direction(i, 1) * val * mm_to_meters_factor;
                    p.z = lut.direction(i, 2) * val * mm_to_meters_factor;
//                    if(val > 0.0){
//                        p.x += lut.offset(i, 0) * mm_to_meters_factor;
//                        p.y += lut.offset(i, 1) * mm_to_meters_factor;
//                        p.z += lut.offset(i, 2) * mm_to_meters_factor;
//                    }

                                        
                    p.intensity = signal(y, x);
                    p.laser_id = range.rows()  - y - 1;
                    
                    p.device_id = 0;
                    
                    p.time = (timestamps[x] - frameTimestamp)* TimeToSecondsFactor;
                    
                    pc->push_back(p);
                }
            }

            return pc;
        }
    return nullptr;
}

// =============================================================================
//   Getters / setters
// =============================================================================

//-----------------------------------------------------------------------------
void ofxLidarSlam::advancedReturnModeChanged(bool& _arg){
    SlamAlgo->SetOverlapSamplingRatio( ( _arg? params.OverlapSamplingRatio.get() : 0.));
    SlamAlgo->SetTimeWindowDuration( ( _arg? params.TimeWindowDuration.get() : 0.));
    SlamAlgo->SetLoggingTimeout( ( _arg? 1.1 * params.TimeWindowDuration.get() : 0.));
    
}



//-----------------------------------------------------------------------------
void ofxLidarSlam::SetBaseToLidarTranslation(double x, double y, double z)
{
    Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
    baseToLidar.translation() = Eigen::Vector3d(x, y, z);
    this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::SetBaseToLidarRotation(double rx, double ry, double rz)
{
    Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
    baseToLidar.linear() = LidarSlam::Utils::RPYtoRotationMatrix(LidarSlam::Utils::Deg2Rad(rx), LidarSlam::Utils::Deg2Rad(ry), LidarSlam::Utils::Deg2Rad(rz));
    this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
}


//-----------------------------------------------------------------------------
void ofxLidarSlam::SetMapUpdate(unsigned int mode)
{
    LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mode);
    if (mapUpdate != LidarSlam::MappingMode::NONE         &&
        mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
        mapUpdate != LidarSlam::MappingMode::UPDATE)
    {
        ofLogWarning("ofxLidarSlam::SetMapUpdate") << "Invalid mapping mode (" << mode << "), ignoring setting.";
        return;
    }
    if (this->SlamAlgo->GetMapUpdate() != mapUpdate)
    {
        this->SlamAlgo->SetMapUpdate(mapUpdate);
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
{
    this->SlamAlgo->SetVoxelGridLeafSize(k, s);
}


//-----------------------------------------------------------------------------
void ofxLidarSlam::SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int mode)
{
    LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(mode);
    if (sampling != LidarSlam::SamplingMode::FIRST         &&
        sampling != LidarSlam::SamplingMode::LAST          &&
        sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
        sampling != LidarSlam::SamplingMode::CENTER_POINT  &&
        sampling != LidarSlam::SamplingMode::CENTROID)
    {
        return;
    }
    if (this->SlamAlgo->GetVoxelGridSamplingMode(k) != sampling)
    {
        this->SlamAlgo->SetVoxelGridSamplingMode(k, sampling);
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::_overlapSamplingRatioChanged(float& ratio)
{
    // Forward this parameter change to SLAM if Advanced Return Mode is enabled
    if (params.AdvancedReturnMode){
        SlamAlgo->SetOverlapSamplingRatio(params.OverlapSamplingRatio);
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::SetAccelerationLimits(float linearAcc, float angularAcc)
{
    this->SlamAlgo->SetAccelerationLimits({linearAcc, angularAcc});
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::SetVelocityLimits(float linearVel, float angularVel)
{
    this->SlamAlgo->SetVelocityLimits({linearVel, angularVel});
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::_timeWindowDurationChanged(float&)
{
    if (params.AdvancedReturnMode)
    {
        SlamAlgo->SetTimeWindowDuration(params.TimeWindowDuration);
        SlamAlgo->SetLoggingTimeout(1.1 * params.TimeWindowDuration);
    }
}

//-----------------------------------------------------------------------------
ofxSpinningSensorKeypointExtractor& ofxLidarSlam::getKeyPointExtractor(){
    static std::unique_ptr<ofxSpinningSensorKeypointExtractor> extractor = std::make_unique<ofxSpinningSensorKeypointExtractor>();
    return *extractor;
}

//#define DRAW_MESH(mesh) \
if(params.bDraw##mesh) { ofSetColor(params.mesh##Color.get()); slamResults.mesh.draw(); }

void ofxLidarSlam::drawMesh(vector<unique_ptr<ofParameter<bool>>>& bDraw ,  vector<ofVboMesh>& mesh, vector<unique_ptr<ofParameter<ofColor>>>& color){
    for(auto k : LidarSlam::KeypointTypes){
        if(bDraw[k] && bDraw[k]->get()) {
            pointShader.begin();
            pointShader.shader.setUniform3f("offset", 0,0,0);
            if(color[k]) ofSetColor(color[k]->get());
            mesh[k].draw();
            pointShader.end();
        }
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::draw(){
    cam.begin();
    
    if(params.bUseDepthTest){
        ofEnableDepthTest();
    }
  
    ofPushStyle();

    ofPushMatrix();
    ofScale(1000);
    

    if(params.bDrawAccum){
    
        size_t numAccumDrawn = 0;
        for(size_t i = 0; i < trajectory.size() && numAccumDrawn < params.drawAccumMax.get(); i++)
        {
            if(trajectory[i].isMarker && trajectory[i].mesh.getVertices().size() > 0){
                pointShader.begin();
                pointShader.shader.setUniform3f("offset", trajectory[i].x, trajectory[i].y, trajectory[i].z);
                trajectory[i].mesh.draw();
                pointShader.end();
                numAccumDrawn ++;
            }
        }
    }
    if(params.bDrawRegisteredMap) {
        pointShader.begin();
        glm::vec3 offset = {0,0,0};
        if(trajectory.size()){
            offset = {trajectory.back().x, trajectory.back().y, trajectory.back().z};
        }
        pointShader.shader.setUniform3f("offset", offset);
        slamResults.RegisteredMap.draw();
        pointShader.end();
    }

    
    drawMesh(params.bDrawMaps ,  slamResults.maps, params.mapColor);
    if (params.OutputCurrentKeypoints){
        drawMesh(params.bDrawKeypoints ,  slamResults.keypoints, params.keypointColor);
    }
    
    
    
    if(trajectory.size() > 0 && params.bDrawTrajectoryLine) {
        ofSetColor(params.TrajectoryLineColor.get());
        TrajectoryLine.draw();
        ofPushStyle();
        trajectory.back().getNode().transformGL();
        ofSetLineWidth(8);
        ofDrawAxis(0.3);
        trajectory.back().getNode().restoreTransformGL();
        ofPopStyle();
    }
    if(trajectory.size() > 0 && params.bDrawMarkers) {
        ofPushStyle();
        ofSetColor(ofColor::yellow);
        for(auto & t: trajectory){
            if(t.isMarker){
                ofDrawBox(t.x, t.y, t.z, 0.3, 0.3, 0.3);
            }
        }
        ofPopStyle();
    }
    ofPopMatrix();
    ofPopStyle();
    if(params.bDrawLidarFeed){
    
        if(renderer){
            renderer->drawPointCloud();
        }
//        if(lidar) {
            
//            if(trajectory.size() > 0){
//                trajectory.back().getNode(true).transformGL();
//            }
//            lidar->drawPointCloud();
//            if(trajectory.size() > 0){
//                trajectory.back().getNode(true).restoreTransformGL();
//            }
//        }
    }
    
    
  
    
    if(params.bUseDepthTest){
        ofDisableDepthTest();
    }
    cam.end();
    
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::drawGui(){
    params.draw();
//    pointShader.gui.draw();
    if(params.bDrawLidarFeed){
        if(renderer){
            renderer->drawGui();
        }
    }
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::printParams(){
    cout << getCurrentSlamParamsAsString();
}
string ofxLidarSlam::getCurrentSlamParamsAsString(){
    stringstream ss;
    PrintMacro(NbThreads)
    PrintMacro(Verbosity)
    PrintMacro(LoggingTimeout)
    PrintMacro(Latency)
    PrintMacro(G2oFileName)
    PrintMacro(FixFirstVertex)
    PrintMacro(FixLastVertex)
    PrintMacro(BaseFrameId)
    PrintMacro(WorldFrameId)
    PrintMacro(TwoDMode)
    PrintMacro(EgoMotionLMMaxIter)
    PrintMacro(EgoMotionICPMaxIter)
    PrintMacro(EgoMotionMaxNeighborsDistance)
    PrintMacro(EgoMotionEdgeNbNeighbors)
    PrintMacro(EgoMotionEdgeMinNbNeighbors)
    PrintMacro(EgoMotionPlaneNbNeighbors)
    PrintMacro(EgoMotionPlanarityThreshold)
    PrintMacro(EgoMotionEdgeMaxModelError)
    PrintMacro(EgoMotionPlaneMaxModelError)
    PrintMacro(EgoMotionInitSaturationDistance)
    PrintMacro(EgoMotionFinalSaturationDistance)
    PrintMacro(LocalizationLMMaxIter)
    PrintMacro(LocalizationICPMaxIter)
    PrintMacro(LocalizationMaxNeighborsDistance)
    PrintMacro(LocalizationEdgeNbNeighbors)
    PrintMacro(LocalizationEdgeMinNbNeighbors)
    PrintMacro(LocalizationPlaneNbNeighbors)
    PrintMacro(LocalizationPlanarityThreshold)
    PrintMacro(LocalizationEdgeMaxModelError)
    PrintMacro(LocalizationPlaneMaxModelError)
    PrintMacro(LocalizationBlobNbNeighbors)
    PrintMacro(LocalizationInitSaturationDistance)
    PrintMacro(LocalizationFinalSaturationDistance)
    PrintMacro(SensorTimeOffset)
    PrintMacro(SensorTimeThreshold)
    PrintMacro(SensorMaxMeasures)
    PrintMacro(LandmarkWeight)
    PrintMacro(LandmarkSaturationDistance)
    PrintMacro(LandmarkPositionOnly)
    PrintMacro(LandmarkCovarianceRotation)
    PrintMacro(LandmarkConstraintLocal)
    PrintMacro(KfDistanceThreshold)
    PrintMacro(KfAngleThreshold)
    PrintMacro(OverlapSamplingRatio)
    PrintMacro(OverlapEstimation)
    PrintMacro(TotalMatchedKeypoints)
    PrintMacro(AccelerationLimits)
    PrintMacro(VelocityLimits)
    PrintMacro(TimeWindowDuration)
    PrintMacro(ComplyMotionLimits)
    
    return ss.str();
    
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::setParamsListeners(){
    
    
    listeners.push(params.egoMotionMode.newListener([&](uint8_t& mode){
        LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(mode);
        if (this->SlamAlgo->GetEgoMotion() != egoMotion){
            this->SlamAlgo->SetEgoMotion(egoMotion);
        }
    }));
    
    
    listeners.push(params.undistortionMode.newListener([&](uint8_t& mode){
        LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(mode);
        if (this->SlamAlgo->GetUndistortion() != undistortion){
            this->SlamAlgo->SetUndistortion(undistortion);
        }
    }));
    
    
    listeners.push(params.mappingMode.newListener([&](uint8_t& mode){
        LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mode);
        if (this->SlamAlgo->GetMapUpdate() != mapUpdate){
            this->SlamAlgo->SetMapUpdate(mapUpdate);
        }
    }));
    listeners.push(params.reset.newListener([&](){
        this->reset();
    }));
    
    addListenerMacro(NbThreads, int)
    addListenerMacro(TwoDMode, bool)
    addListenerMacro(EgoMotionLMMaxIter, unsigned int)
    addListenerMacro(EgoMotionICPMaxIter, unsigned int)
    addListenerMacro(EgoMotionMaxNeighborsDistance, double)
    addListenerMacro(EgoMotionEdgeNbNeighbors, unsigned int)
    addListenerMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)
    addListenerMacro(EgoMotionPlaneNbNeighbors, unsigned int)
    addListenerMacro(EgoMotionPlanarityThreshold, double)
    addListenerMacro(EgoMotionEdgeMaxModelError, double)
    addListenerMacro(EgoMotionPlaneMaxModelError, double)
    addListenerMacro(EgoMotionInitSaturationDistance, double)
    addListenerMacro(EgoMotionFinalSaturationDistance, double)
    addListenerMacro(LocalizationLMMaxIter, unsigned int)
    addListenerMacro(LocalizationICPMaxIter, unsigned int)
    addListenerMacro(LocalizationMaxNeighborsDistance, double)
    addListenerMacro(LocalizationEdgeNbNeighbors, unsigned int)
    addListenerMacro(LocalizationEdgeMinNbNeighbors, unsigned int)
    addListenerMacro(LocalizationPlaneNbNeighbors, unsigned int)
    addListenerMacro(LocalizationPlanarityThreshold, double)
    addListenerMacro(LocalizationEdgeMaxModelError, double)
    addListenerMacro(LocalizationPlaneMaxModelError, double)
    addListenerMacro(LocalizationBlobNbNeighbors, unsigned int)
    addListenerMacro(LocalizationInitSaturationDistance, double)
    addListenerMacro(LocalizationFinalSaturationDistance, double)
    
    addListenerMacro(KfDistanceThreshold, double)
    addListenerMacro(KfAngleThreshold, double)
    
    addListenerMacro(VoxelGridDecayingThreshold, double)
    addListenerMacro(VoxelGridSize, int )
    addListenerMacro(VoxelGridResolution, double )
    addListenerMacro(VoxelGridMinFramesPerVoxel, unsigned int )
    addListenerMacro(OverlapSamplingRatio,float)
//    addListenerMacro(TimeWindowDuration, float)
    addListenerMacro(GravityWeight, double)
    listeners.push(params.OverlapSamplingRatio.newListener(this, &ofxLidarSlam::_overlapSamplingRatioChanged));
//    addListenerMacro(UseBlobs, bool)
    
    listeners.push(params.TimeWindowDuration.newListener(this, &ofxLidarSlam::_timeWindowDurationChanged));
    
    listeners.push(params.LeafSizeEdges.newListener([&](float& f){
        SetVoxelGridLeafSizeEdges(f);
    }));
    listeners.push(params.LeafSizePlanes.newListener([&](float& f){
        SetVoxelGridLeafSizePlanes(f);
        
    }));
//    listeners.push(params.LeafSizeBlobs.newListener([&](float& f){
//        SetVoxelGridLeafSizeBlobs(f);
//    }));
    
    listeners.push(params.saveMaps.newListener([&](){
        auto ts = ofGetTimestampString();
        slamResults.saveMaps(ts);
        saveRegisteredMeshes(ts);
    }));
    
    
    listeners.push(params.samplingModeEdges.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModeEdges((int)mode);
    }));
    
    listeners.push(params.samplingModePlanes.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModePlanes((int)mode);
    }));
    
    listeners.push(params.samplingModeBlobs.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModeBlobs((int)mode);
    }));
            
    listeners.push(params.angularAccLimit.newListener([&](float&){
        SetAccelerationLimits(params.linearAccLimit.get(), params.angularAccLimit.get());
    }));
    listeners.push(params.angularVelLimit.newListener([&](float&){
        SetVelocityLimits(params.linearVelLimit.get(), params.angularVelLimit.get());
    }));
    listeners.push(params.linearAccLimit.newListener([&](float&){
        SetAccelerationLimits(params.linearAccLimit.get(), params.angularAccLimit.get());
    }));
    listeners.push(params.linearVelLimit.newListener([&](float&){
        SetVelocityLimits(params.linearVelLimit.get(), params.angularVelLimit.get());
    }));
    
    listeners.push(params.verboseLevel.newListener([&](uint8_t& level){
        SlamAlgo->SetVerbosity(level);
    }));

}



//-----------------------------------------------------------------------------
void ofxLidarSlam::setDefaults(){
    
    SetDefaultValueMacro(TwoDMode)
    SetDefaultValueMacro(EgoMotionLMMaxIter)
    SetDefaultValueMacro(EgoMotionICPMaxIter)
    SetDefaultValueMacro(EgoMotionMaxNeighborsDistance)
    SetDefaultValueMacro(EgoMotionEdgeNbNeighbors)
    SetDefaultValueMacro(EgoMotionEdgeMinNbNeighbors)
    SetDefaultValueMacro(EgoMotionPlaneNbNeighbors)
    SetDefaultValueMacro(EgoMotionPlanarityThreshold)
    SetDefaultValueMacro(EgoMotionEdgeMaxModelError)
    SetDefaultValueMacro(EgoMotionPlaneMaxModelError)
    SetDefaultValueMacro(EgoMotionInitSaturationDistance)
    SetDefaultValueMacro(EgoMotionFinalSaturationDistance)
    SetDefaultValueMacro(LocalizationLMMaxIter)
    SetDefaultValueMacro(LocalizationICPMaxIter)
    SetDefaultValueMacro(LocalizationMaxNeighborsDistance)
    SetDefaultValueMacro(LocalizationEdgeNbNeighbors)
    SetDefaultValueMacro(LocalizationEdgeMinNbNeighbors)
    SetDefaultValueMacro(LocalizationPlaneNbNeighbors)
    SetDefaultValueMacro(LocalizationPlanarityThreshold)
    SetDefaultValueMacro(LocalizationEdgeMaxModelError)
    SetDefaultValueMacro(LocalizationPlaneMaxModelError)
    SetDefaultValueMacro(LocalizationBlobNbNeighbors)
    SetDefaultValueMacro(LocalizationInitSaturationDistance)
    SetDefaultValueMacro(LocalizationFinalSaturationDistance)
    SetDefaultValueMacro(SensorTimeOffset)
    SetDefaultValueMacro(SensorTimeThreshold)
    SetDefaultValueMacro(SensorMaxMeasures)

    SetDefaultValueMacro(KfDistanceThreshold)
    SetDefaultValueMacro(KfAngleThreshold)
    SetDefaultValueMacro(OverlapSamplingRatio)
    SetDefaultValueMacro(TimeWindowDuration)
}
