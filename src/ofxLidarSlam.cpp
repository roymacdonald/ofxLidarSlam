//
//  ofxLidarSlam.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/5/22.
//

#include "ofxLidarSlam.h"
// PCL
#include <pcl/common/transforms.h>



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <LidarSlam/Utilities.h>

#define addListenerMacro(param,type) \
listeners.push(params.param.newListener([&](type & _arg){\
this->SlamAlgo->Set##param(_arg);\
}));

#define GET_MAP(map, type) if(params.bDraw##map) {PclToOf(this->SlamAlgo->GetMap(type), results.map);}

#define GET_SUBMAP(map, type) if(params.bDraw##map) {PclToOf(this->SlamAlgo->GetTargetSubMap(type), results.map);}

#define GET_KEYPOINTS(keypoints, type) if(params.bDraw##keypoints) {PclToOf(this->SlamAlgo->GetKeypoints(type, params.OutputKeypointsInWorldCoordinates.get()), results.keypoints);}




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



//void toPCL(shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& dest, const vector<glm::vec3>& points){
//    dest =  make_shared<pcl::PointCloud<pcl::PointXYZ>>(points.size(), 1);
//    for (size_t i = 0; i < points.size(); i++){
//        dest->points[i] = pcl::PointXYZ(points[i].x,points[i].y,points[i].z);
//    }
//}
//#define STRINGIFY(x) #x

//-----------------------------------------------------------------------------
ofxLidarSlam::ofxLidarSlam():pointShader("SLAM")
{
    
    SlamAlgo = std::make_unique<LidarSlam::Slam>();
    
    SlamAlgo->SetKeyPointsExtractor(getKeyPointExtractor().extractor);
    InitPose.setZero();
    
    setDefaults();
    SlamAlgo->SetVerbosity(5);
    
    this->reset();
    
    params.gui.add(getKeyPointExtractor().parameters);
    
    
    cam.setFarClip(1000000);
    cam.setNearClip(0);
    cam.setRelativeYAxis(!cam.getRelativeYAxis());
    
    /// the SLAM library uses meters as its units
    pointShader.setRangeUnit(1);
    
    
    startThread();
    
    
//    static const string shader_header = "#version 150\n";
//
//          static const string fragShader = shader_header + STRINGIFY(
//          in float vcolor;
//          uniform sampler2DRect palette;
//          out vec4 color;
//          void main() {
//              color = texture(palette, vec2(vcolor, 1));
//          }
//          );
//
//          string vertShader = shader_header + STRINGIFY(
//          in vec4 position;
//          uniform mat4 modelViewProjectionMatrix;
//          uniform mat4 transform;
//          uniform float range_max;
//          uniform float colorMapSize;
//
//          out float vcolor;
//          void main(){
//              float range = length(position.xyz);
//              vcolor = (range * colorMapSize)/(226326.f*range_max);
//              gl_Position = modelViewProjectionMatrix * transform * position;
//          }
//          );
//
//
    
    
//    pointShader.loadShader(vertShader, fragShader);
    
    

    
}
//-----------------------------------------------------------------------------
ofxLidarSlam::~ofxLidarSlam()
{
    toSlamScan.close();
    toSlamIMU.close();
    fromSlam.close();

    waitForThread(true);
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::setup(std::shared_ptr<ofxOuster>& lidar){
    this->lidar = lidar;
    setParamsListeners();
    
    
    
    listeners.push(this->lidar->lidarDataEvent.newListener(this, &ofxLidarSlam::onLidarData));
    listeners.push(this->lidar->imuDataEvent.newListener(this, &ofxLidarSlam::onImuData));
    
    listeners.push(ofEvents().update.newListener(this, &ofxLidarSlam::_update));
    
    
    
    pointShader.loadShaderFromFiles(ofToDataPath("shaders/shader.vert"),ofToDataPath("shaders/shader.frag"));
//    pointShader.colorMap.makeColorMap("GRAY SCALE", {ofFloatColor::black, ofFloatColor::white});
    
    pointShader.setGuiPosition( glm::vec2(ofGetWidth() - 600,10) );
    
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::reset()
{
    this->SlamAlgo->Reset(true);
    this->SlamAlgo->ClearSensorMeasurements();
    this->SlamAlgo->SetSensorMaxMeasures(10000);
    
    trajectory.clear();
    
    TrajectoryLine.clear();
    frameCount = 0;
    
    this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::PoseToIsometry(this->InitPose));
    
    // Enable overlap computation only if advanced return mode is activated
    this->SlamAlgo->SetOverlapSamplingRatio((params.AdvancedReturnMode ? params.OverlapSamplingRatio.get() : 0.));
    // Enable motion limitation checks if advanced return mode is activated
    this->SlamAlgo->SetTimeWindowDuration(params.AdvancedReturnMode ? params.TimeWindowDuration.get() : 0.);
    // Log the necessary poses if advanced return mode is activated
    this->SlamAlgo->SetLoggingTimeout(params.AdvancedReturnMode ? 1.1 * params.TimeWindowDuration.get() : 0.);
    
    
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::onLidarData(ouster::LidarScan &scan){
//    cout << "ofxLidarSlam::onLidarData\n";
    toSlamScan.send(scan);
    
}

void ofxLidarSlam::saveRegisteredMeshes(string timestamp){
    
    string dirPath  = "RegisteredMeshes/"+ timestamp;
    cout <<"saveRegisteredMeshes:: saving to: " << dirPath <<"\n";
    
    {
    ofDirectory dir (dirPath);
    dir.create(true);
    }
    {
    ofDirectory dir (dirPath + "/metadata");
    dir.create(true);
    }
    
    
    for(size_t i =0; i < trajectory.size(); i++){
        trajectory[i].save(dirPath, i);
    }
    cout <<"saveRegisteredMeshes:: done!\n";

}

//-----------------------------------------------------------------------------
ofxLidarSlamResults ofxLidarSlam::processLidarData(ouster::LidarScan &scan){
    
    ofxLidarSlamResults results;
    if(params.bEnableSlam){
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
        
        
        auto pc = getPointCloudFromLidar(scan) ;
        if(pc == nullptr){
            ofLogError("ofxLidarSlam::onLidarData") << "Got nullptr pointcloud";
        }
        this->SlamAlgo->AddFrame(pc);
        
        this->AddCurrentPoseToTrajectory();
        
        // ===== SLAM frame and pose =====
        // Output : Current undistorted LiDAR frame in world coordinates
        
        PclToOf(this->SlamAlgo->GetRegisteredFrame(), results.RegisteredMap);
        if(params.bAccumulateRegistered){
            trajectory.back().copyMesh(results.RegisteredMap);
        }
        
        params.startAccumDraw.setMax(trajectory.size());
        params.endAccumDraw.setMax(trajectory.size());
        
//        cout << "Trajectory num verts: " << trajectory.back().mesh.getNumVertices() << endl;
        // Update the output maps if required or if the mode was changed
        bool updateMaps = params.OutputKeypointsMaps != this->PreviousMapOutputMode ||
        (this->SlamAlgo->GetNbrFrameProcessed() - 1) % params.MapsUpdateStep == 0;
        
        // The expected maps can be the whole maps or the submaps
        // If the maps is fixed by the user, the whole map and the submap are equal but the submap is outputed (faster)
        if (updateMaps && (params.OutputKeypointsMaps == OutputKeypointsMapsMode::FULL_MAPS && this->SlamAlgo->GetMapUpdate() != LidarSlam::MappingMode::NONE))
        {
            GET_MAP(EdgeMap, LidarSlam::EDGE)
            GET_MAP(PlanarMap, LidarSlam::PLANE)
            GET_MAP(BlobMap, LidarSlam::BLOB)
            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        }
        else if (updateMaps && (params.OutputKeypointsMaps == OutputKeypointsMapsMode::SUB_MAPS || this->SlamAlgo->GetMapUpdate() == LidarSlam::MappingMode::NONE))
        {
            
            GET_SUBMAP(EdgeMap, LidarSlam::EDGE)
            GET_SUBMAP(PlanarMap, LidarSlam::PLANE)
            GET_SUBMAP(BlobMap, LidarSlam::BLOB)
            
//            if(params.bDrawEdgeMap) {PclToOf(this->SlamAlgo->GetTargetSubMap(LidarSlam::EDGE), results.EdgeMap);}
//            if(params.bDrawPlanarMap) {PclToOf(this->SlamAlgo->GetTargetSubMap(LidarSlam::PLANE), results.PlanarMap);}
//            if(params.bDrawBlobMap) {PclToOf(this->SlamAlgo->GetTargetSubMap(LidarSlam::BLOB), results.BlobMap);}
            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        }
        // If the output is disabled, reset it.
        else if (params.OutputKeypointsMaps != this->PreviousMapOutputMode && params.OutputKeypointsMaps == OutputKeypointsMapsMode::NONE)
        {
            results.EdgeMap.clear();
            results.PlanarMap.clear();
            results.BlobMap.clear();
            this->PreviousMapOutputMode = params.OutputKeypointsMaps.get();
        }
        
        // ===== Extracted keypoints from current frame =====
        if (params.OutputCurrentKeypoints)
        {

            
            GET_KEYPOINTS(EdgeKeypoints, LidarSlam::EDGE)
            GET_KEYPOINTS(PlanarKeypoints, LidarSlam::PLANE)
            GET_KEYPOINTS(BlobKeypoints, LidarSlam::BLOB)
            
//            if(params.bDrawEdgeKeypoints) { PclToOf(this->SlamAlgo->GetKeypoints(LidarSlam::EDGE, wc), results.EdgeKeypoints);}
//            if(params.bDrawPlanarKeypoints) { PclToOf(this->SlamAlgo->GetKeypoints(LidarSlam::PLANE,wc), results.PlanarKeypoints);}
//            if(params.bDrawBlobKeypoints) { PclToOf(this->SlamAlgo->GetKeypoints(LidarSlam::BLOB, wc), results.BlobKeypoints);}
        }
        
        results.bValid = true;
        
    }
    frameCount++;
    return results;
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::onImuData(ofxOusterIMUData& data){
    toSlamIMU.send(data);
//    processImuData(data);
}
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
    ofPixels pixels;
    
//    <> toSlamScan;
    ofxOusterIMUData imuData;
//    toSlamIMU;
    while(toSlamIMU.tryReceive(imuData)){
        processImuData(imuData);
    }
    
    ouster::LidarScan scan;
    
    while(toSlamScan.receive(scan)){
        auto results = processLidarData(scan);

        if(results.bValid){
            fromSlam.send(std::move(results));
        }
    }
}

//-----------------------------------------------------------------------------
void ofxLidarSlam::_update(ofEventArgs &){
//    bool newFrame = false;
    while(fromSlam.tryReceive(slamResults)){
//        newFrame = true;
    }
//    if(newFrame){
     
//    }
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
    
    
    trajectory.push_back(TrajectoryPoint());
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
LidarSlam::Slam::PointCloud::Ptr ofxLidarSlam::getPointCloudFromLidar(ouster::LidarScan & scan)
{
    if(lidar){
        
        LidarSlam::Slam::PointCloud::Ptr pc = make_shared<LidarSlam::Slam::PointCloud>();
        auto r = lidar->getRenderer();
        
        if(r){
            
            
//            float Rpm = getRPMs(lidar->getSensorInfo());
//            size_t nbPoints = r->points.getNumVertices();
            size_t nbPoints = scan.w * scan.h;
            // We take advantage of the fact that ofxOusterRenderer has already dewarped the pointcloud and stored it.
            // Loop over points data
            pc->reserve(nbPoints);
            
            auto timestamps = scan.timestamp();
            auto ids = scan.measurement_id();
            
//            lidar_conversions::Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;
            
//            stringstream ss;
//            ss << "scan.w  : " << scan.w << "  h: " << scan.h << "\n";
//            ss << "nbPoints : " << nbPoints << "\n";
            //            ss << "timestamps : " << timestamps.size() << "\n";
            //            ss << "ids : " << ids.size() << "\n";
            //            uint16_t m = 0;
            //            for(auto& i: ids){
            //                if(i > m){
            //                    m = i;
            //                }
            //            }
            //            ss << "max id: " << m << "\n";
            //
//            cout << ss.str();
            
            auto frameTimestamp = timestamps[(params.TimestampFirstPacket?0:(scan.w - 1))];
            pc->header.stamp = frameTimestamp / 1000.0f; // max time in microseconds
            pc->header.frame_id = this->SlamAlgo->GetBaseFrameId();
            pc->header.seq = frameCount;
            auto range = scan.field(ouster::sensor::RANGE);
            auto signal = scan.field(ouster::sensor::ChanField::SIGNAL);//.cast<double>();
            if(params.destaggerSignal){
//                signal = ouster::destagger<double>(signal.cast<double>(), lidar->getSensorInfo().format.pixel_shift_by_row);
            }
            
            
            auto& lut = r->getLut();
            
        
            
            if (range.cols() * range.rows() != lut.direction.rows()){
                ofLogError("ofxLidarSlam::getPointCloudFromLidar") << "unexpected image dimensions";
                return nullptr;
            }
            
//            auto reshaped = Eigen::Map<const Eigen::Array<ouster::LidarScan::raw_t, -1, 1>>(range.data(), range.cols() * range.rows()).cast<double>();
//            auto nooffset = lut.direction.colwise() * reshaped;
//            return (nooffset.array() == 0.0).select(nooffset, nooffset + lut.offset);
            
//            bool useMM = params.bUseMillimeters.get();
            
            for(size_t y= 0; y< range.rows(); y++){
                for(size_t x= 0; x< range.cols(); x++){
                    size_t i = y * range.cols() + x;
                    double val = range(y, x);
                    
                    
                    LidarSlam::Slam::Point p;
                    p.x = lut.direction(i, 0) * val * 0.001;//transform from millimeters to meters
                    p.y = lut.direction(i, 1) * val * 0.001;//
                    p.z = lut.direction(i, 2) * val * 0.001;//

                    
//                    p.x += lut.offset(i, 0);
//                    p.y += lut.offset(i, 1);
//                    p.z += lut.offset(i, 2);
                    
                    p.intensity = signal(y, x);
                    p.laser_id = range.rows()  - y - 1;
//                    p.time = floor((timestamps[x] - frameTimestamp)/1000.0f);
                    
                    p.device_id = 0;
                    // in case of using more than a single device the following line should be used instead of the prior. otherwise it gives an anoying warning
                    //p.device_id = lidar->getSensorInfo().init_id;
                    
                    
//                    double frameAdvancement = frameAdvancementEstimator(p);
                    //                p.time = (params.TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) ;
//                    p.time = (params.TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / Rpm * 60.;
                    
                    p.time = (timestamps[x] - frameTimestamp)* TimeToSecondsFactor;
                    
//                    cout << "time: " << p.time << "  " << t;
                    
                    
                    pc->push_back(p);
                }
            }
//
////            auto & v = r->points.getVertices();
//            size_t y = scan.h;
//            for (size_t i = 0; i < nbPoints; i++)
//            {
//                if(i%scan.w == 0){
//                    y--;
//                }
//                size_t x = i % scan.w;
//
//                LidarSlam::Slam::Point p;
//                p.x = v[i].x;
//                p.y = v[i].y;
//                p.z = v[i].z;
//                //                p.time = timestamps[x] * TimeToSecondsFactor; // time in seconds
//                p.laser_id = y;
//                p.intensity = signal(i) ;
//
//                double frameAdvancement = frameAdvancementEstimator(p);
//                //                p.time = (params.TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) ;
//                p.time = (params.TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / Rpm * 60.;
//
//
//                pc->push_back(p);
//            }
            return pc;
        }
    }
    return nullptr;
}

// =============================================================================
//   Getters / setters
// =============================================================================

//-----------------------------------------------------------------------------
void ofxLidarSlam::advancedReturnModeChanged(bool& _arg)
{
    
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    
    // If AdvancedReturnMode is being activated
    if (_arg)
    {
        // Add new optional arrays to trajectory, and init past values to 0.
        //      for (const auto& it : debugInfo)
        //      {
        //        auto array = Utils::CreateArray<vtkDoubleArray>(it.first, 1, this->Trajectory->GetNumberOfPoints());
        //        for (vtkIdType i = 0; i < this->Trajectory->GetNumberOfPoints(); i++)
        //          array->SetTuple1(i, 0.);
        //        this->Trajectory->GetPointData()->AddArray(array);
        //      }
        // Enable overlap computation
        SlamAlgo->SetOverlapSamplingRatio(params.OverlapSamplingRatio);
        SlamAlgo->SetTimeWindowDuration(params.TimeWindowDuration);
        SlamAlgo->SetLoggingTimeout(1.1 * params.TimeWindowDuration);
    }
    
    // If AdvancedReturnMode is being disabled
    else
    {
        // Delete optional arrays
        //      for (const auto& it : debugInfo)
        //        this->Trajectory->GetPointData()->RemoveArray(it.first.c_str());
        // Disable overlap computation
        SlamAlgo->SetOverlapSamplingRatio(0.);
        SlamAlgo->SetTimeWindowDuration(0.);
        SlamAlgo->SetLoggingTimeout(0.);
    }
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

#define DRAW_MESH(mesh) \
if(params.bDraw##mesh) { ofSetColor(params.mesh##Color.get()); slamResults.mesh.draw(); }

//-----------------------------------------------------------------------------
void ofxLidarSlam::draw(){
    auto r = lidar->getRenderer();
    if(!r)return;
    cam.begin();
    
    if(params.bUseDepthTest){
        ofEnableDepthTest();
    }
  
    ofPushStyle();

    ofPushMatrix();
    ofScale(1000);
    
    if(params.bDrawRegisteredMap) {
        if(params.bAccumulateRegistered){
            
            size_t end = 0;
            if(trajectory.size() > params.startAccumDraw.get()){
                end = trajectory.size() - 1 -params.startAccumDraw.get();
            }
            size_t start = 0;
            if(end >= params.endAccumDraw.get()){
                start = end - params.endAccumDraw.get();
            }
            if(start < end){
                for(size_t i = start; i < end && i < trajectory.size(); i++)
                {
//                    pointShader.begin();
//                    pointShader.shader.setUniform3f("offset", trajectory[i].x, trajectory[i].y, trajectory[i].z);
                    ofSetColor(255);
//                    t.mesh.disableColors();
                    trajectory[i].mesh.draw();
//                    pointShader.end();
                }
            }
        }else{
//            pointShader.begin();
//            if(trajectory.size()){
//                pointShader.shader.setUniform3f("offset", trajectory.back().x, trajectory.back().y, trajectory.back().z);
//            }
            slamResults.RegisteredMap.draw();
//            pointShader.end();
        }
    }
    pointShader.begin(); pointShader.shader.setUniform3f("offset", 0,0,0);
    DRAW_MESH(EdgeMap)
    pointShader.end();
    
    pointShader.begin(); pointShader.shader.setUniform3f("offset", 0,0,0);
    DRAW_MESH(PlanarMap)
    pointShader.end();
    
    pointShader.begin(); pointShader.shader.setUniform3f("offset", 0,0,0);
    DRAW_MESH(BlobMap)
    pointShader.end();
    
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
    ofPopMatrix();
    ofPopStyle();
    if(params.bDrawLidarFeed){
        if(lidar) {
            if(trajectory.size() > 0){
                trajectory.back().getNode(true).transformGL();
            }
            lidar->drawPointCloud();
            if(trajectory.size() > 0){
                trajectory.back().getNode(true).restoreTransformGL();
            }
        }
    }
    
    
    if(params.bUseDepthTest){
        ofDisableDepthTest();
    }
    cam.end();
    
}
//-----------------------------------------------------------------------------
void ofxLidarSlam::drawGui(){
    params.gui.draw();
    pointShader.gui.draw();
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
    addListenerMacro(UseBlobs, bool)
    
    listeners.push(params.TimeWindowDuration.newListener(this, &ofxLidarSlam::_timeWindowDurationChanged));
    
    listeners.push(params.LeafSizeEdges.newListener([&](float& f){
        SetVoxelGridLeafSizeEdges(f);
    }));
    listeners.push(params.LeafSizePlanes.newListener([&](float& f){
        SetVoxelGridLeafSizePlanes(f);
        
    }));
    listeners.push(params.LeafSizeBlobs.newListener([&](float& f){
        SetVoxelGridLeafSizeBlobs(f);
    }));
    
    listeners.push(params.saveMaps.newListener([&](){
        auto ts = ofGetTimestampString();
        slamResults.saveMaps(ts);
        saveRegisteredMeshes(ts);
    }));
    
    
    //    //-----------------------------------------------------------------------------
    //    void ofxLidarSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
    //    {
    //        //  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting VoxelGridLeafSize to " << s);
    //        this->SlamAlgo->SetVoxelGridLeafSize(k, s);
    //        //  this->ParametersModificationTime.Modified();
    //    }
    
    listeners.push(params.samplingModeEdges.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModeEdges((int)mode);
    }));
    
    listeners.push(params.samplingModePlanes.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModePlanes((int)mode);
    }));
    
    listeners.push(params.samplingModeBlobs.newListener([&](uint8_t& mode){
        SetVoxelGridSamplingModeBlobs((int)mode);
    }));
    
    //
       
    //
    //
    //
        
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
    
    
    //    SetDefaultValueMacro(NbThreads)
    //    SetDefaultValueMacro(Verbosity)
    //    SetDefaultValueMacro(LoggingTimeout)
    //    SetDefaultValueMacro(Latency)
    //    SetDefaultValueMacro(G2oFileName)
    //    SetDefaultValueMacro(FixFirstVertex)
    //    SetDefaultValueMacro(FixLastVertex)
    //    SetDefaultValueMacro(BaseFrameId)
    //    SetDefaultValueMacro(WorldFrameId)
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
    //    SetDefaultValueMacro(LandmarkWeight)
    //    SetDefaultValueMacro(LandmarkSaturationDistance)
    //    SetDefaultValueMacro(LandmarkPositionOnly)
    //    SetDefaultValueMacro(LandmarkCovarianceRotation)
    //    SetDefaultValueMacro(LandmarkConstraintLocal)
    SetDefaultValueMacro(KfDistanceThreshold)
    SetDefaultValueMacro(KfAngleThreshold)
    //    SetDefaultValueMacro(MapUpdate)
    SetDefaultValueMacro(OverlapSamplingRatio)
    //    SetDefaultValueMacro(OverlapEstimation)
    //    SetDefaultValueMacro(TotalMatchedKeypoints)
    //    SetDefaultValueMacro(AccelerationLimits)
    //    SetDefaultValueMacro(VelocityLimits)
    SetDefaultValueMacro(TimeWindowDuration)
    //    SetDefaultValueMacro(ComplyMotionLimits)
}
