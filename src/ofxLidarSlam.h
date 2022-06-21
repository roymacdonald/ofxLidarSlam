//
//  ofxLidarSlam.hpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 6/5/22.
//

#pragma once
#include "ofMain.h"
// LOCAL
#include "LidarSlam/Slam.h"

#include "ouster/lidar_scan.h"
#include "ofxSpinningSensorKeypointExtractor.h"
//! Which keypoints' maps to output
#include "ofxOuster.hpp"
#include "ofxOusterIMU.h"
#include "ofxOusterRenderer.hpp"
#include "ofxLidarSlamParameters.h"
#include "ofxPointShader.h"

#include "ofxGui.h"

enum OutputKeypointsMapsMode : uint8_t
{
    //! No maps output
    NONE = 0,
    //! Output the whole keypoints' maps
    FULL_MAPS = 1,
    //! Output the target sub maps used for the current frame registration
    SUB_MAPS = 2
};

//class Range{
//public:
//    Range(string _name):name(_name){
//
//    }
//    float mn = std::numeric_limits<float>::max();
//    float mx = - std::numeric_limits<float>::max();
//    void check(float val){
//        if(val > mx) mx = val;
//        if(val < mn) mn = val;
//    }
//    string name;
//    void print(){
//        cout << name <<" Range " << mn << " - " << mx << endl;
//    }
//};
//class Range3f{
//public:
//
//    void check(const glm::vec3& v){
//        ranges[0].check(v.x);
//        ranges[1].check(v.y);
//        ranges[2].check(v.z);
//    }
//    Range ranges [3] = {Range("X"),Range("Y"),Range("Z")};
//
//    void print(){
//        ranges[0].print();
//        ranges[1].print();
//        ranges[2].print();
//    }
//
//};

class TrajectoryPoint{
public:
    double x;
    double y;
    double z;
    double time;
    glm::dquat quaternion;
    double axisAngle [4];
    double covariance [36];
    
    TrajectoryPoint * prev = nullptr;
    
    const ofNode& getNode(bool scaled = false){
        if(bNeedsSetNode){
            bNeedsSetNode = false;
            node.setOrientation(quaternion);
            node.setPosition({x, y, z});
            
            scaledNode.setOrientation(quaternion);
            scaledNode.setPosition({x*1000.0, y*1000.0, z*1000.0});
            
        }
        if(scaled ){
            return scaledNode;
        }
        return node;
    }
    
//    glm::mat4 getTransformMatrix(){
//        return getNode().getGlobalTransformMatrix();
//    }
    
//    const glm::dmat4& getTransformAccumulatedMatrix(bool scale = true){
//        if(bNeedsSetTransform){
//            bNeedsSetTransform = false;
//            glm::dvec3 pos(x, y, z);
//            if(scale){
//                pos = pos *1000.0;
//            }
//            accumTransform = glm::translate(glm::dmat4(1.0), pos);
//
//
//            if(prev != nullptr){
//                accumQuat = prev->accumQuat * quaternion;
//            }else{
//                accumQuat = quaternion;
//            }
//            accumTransform = accumTransform * glm::toMat4(accumQuat);
//        }
//        return accumTransform;
//    }
    
    ofVboMesh mesh;
    
    void copyMesh(ofVboMesh& m){
        mesh.setMode(OF_PRIMITIVE_POINTS);
        mesh.addVertices(m.getVertices());
        
        
    }
    
    bool save(string savePath, size_t index){
        if(mesh.getVertices().size()){
            mesh.save(savePath + "/" + ofToString(index) + ".ply");
        }
            ofJson j;
            j["x"] = x;
            j["y"] = y;
            j["z"] = z;
            j["time"] = time;
            j["quaternion"] = {
                        {"w", quaternion.w},
                        {"x", quaternion.x},
                        {"y", quaternion.y},
                        {"z", quaternion.z}};
            j["axisAngle"] = axisAngle;
            j["covariance"] = covariance;
            
        return ofSaveJson(savePath + "/metadata/"+ofToString(index) + ".json", j);
            
    }
    
    
    
protected:
    
    
private:
    
    ofNode node, scaledNode;
    bool bNeedsSetNode = true;
    
//    glm::dquat accumQuat;
//    glm::dmat4 accumTransform;
    
};


class ofxLidarSlamResults{
public:
    TrajectoryPoint trajectoryPoint;
    
    ofVboMesh RegisteredMap;
    ofVboMesh EdgeMap;
    ofVboMesh PlanarMap;
    ofVboMesh BlobMap;

    ofVboMesh EdgeKeypoints;
    ofVboMesh PlanarKeypoints;
    ofVboMesh BlobKeypoints;
    
    bool bValid = false;
    
    void saveMaps(string timestamp){
    
        if(EdgeMap.getVertices().size()) {
            EdgeMap.save("EdgeMap_" + timestamp + ".ply" );
        }
        if(PlanarMap.getVertices().size()) {
            PlanarMap.save("PlanarMap_" + timestamp + ".ply" );
        }
        if(BlobMap.getVertices().size()) {
            BlobMap.save("BlobMap_" + timestamp + ".ply" );
        }
        
    }
    
    
};

class ofxLidarSlam: public ofThread
{
public:
    ofxLidarSlam();
    virtual ~ofxLidarSlam();
    
    void setup(std::shared_ptr<ofxOuster>& lidar);
    //  virtual vtkMTimeType GetMTime() override;
    
    // ---------------------------------------------------------------------------
    //   General stuff and flags
    // ---------------------------------------------------------------------------
    
    void reset();
    
    // Initialization
    void setInitialMap(const std::string& mapsPathPrefix);
    void setInitialPoseTranslation(double x, double y, double z);
    void setInitialPoseRotation(double roll, double pitch, double yaw);
    
    // Getters / Setters
    
    
    
    
    
    // ---------------------------------------------------------------------------
    //   BASE to LIDAR transform
    // ---------------------------------------------------------------------------
    
    void SetBaseToLidarTranslation(double x, double y, double z);
    
    void SetBaseToLidarRotation(double rx, double ry, double rz);
    
   
    
    // ---------------------------------------------------------------------------
    //   Keypoints extractor, Key frames and Maps parameters
    // ---------------------------------------------------------------------------
    
    
    
    
    
    // Set RollingGrid Parameters
    
    unsigned int GetMapUpdate();
    void SetMapUpdate(unsigned int mode);
    
    
    int GetVoxelGridSamplingMode(LidarSlam::Keypoint k);
    void SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int sm);
    
    // For edges
    int GetVoxelGridSamplingModeEdges() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::EDGE); }
    void SetVoxelGridSamplingModeEdges(int sm)  { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::EDGE, sm);  }

    // For planes
    int GetVoxelGridSamplingModePlanes() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE); }
    void SetVoxelGridSamplingModePlanes(int sm) { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE, sm); }

    // For blobs
    int GetVoxelGridSamplingModeBlobs() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB); }
    void SetVoxelGridSamplingModeBlobs(int sm)  { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB, sm);  }
    
    void SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s);
    
    // For edges
    void SetVoxelGridLeafSizeEdges(double s)  { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::EDGE, s);  }

    // For planes
    void SetVoxelGridLeafSizePlanes(double s) { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::PLANE, s); }

    // For blobs
    void SetVoxelGridLeafSizeBlobs(double s)  { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::BLOB, s);  }
    
    
    // ---------------------------------------------------------------------------
    //   Confidence estimator parameters
    // ---------------------------------------------------------------------------
    
    


    void SetAccelerationLimits(float linearAcc, float angularAcc);
    void SetVelocityLimits(float linearVel, float angularVel);
        


    void draw();
    
    void drawGui();
    
    ofxLidarSlamParameters params;
    
    void printParams();
    string getCurrentSlamParamsAsString();
    
    const vector<TrajectoryPoint>& getTrajectory(){return trajectory;};
    
    void saveRegisteredMeshes(string timestamp);
    
protected:
    
    
    void setDefaults();
    
private:
    ofxLidarSlam(const ofxLidarSlam&) = delete;
    void operator=(const ofxLidarSlam&) = delete;
    
    
    
    
    // ---------------------------------------------------------------------------
    //   Useful helpers
    // ---------------------------------------------------------------------------
    
    
    // Add current SLAM pose and covariance in WORLD coordinates to Trajectory.
    void AddCurrentPoseToTrajectory();
    
    
    LidarSlam::Slam::PointCloud::Ptr getPointCloudFromLidar(ouster::LidarScan & scan) ;
    
    // ---------------------------------------------------------------------------
    //   Member attributes
    // ---------------------------------------------------------------------------
    
    
    
protected:
    
    std::unique_ptr<LidarSlam::Slam> SlamAlgo = nullptr;
    
    static ofxSpinningSensorKeypointExtractor& getKeyPointExtractor();
    
    
    /// callback on AdvanceReturnMode change
    void advancedReturnModeChanged(bool& _arg);
    
    void _timeWindowDurationChanged(float&);
    void _overlapSamplingRatioChanged(float&);

    
    
    void onLidarData(ouster::LidarScan &);
    void onImuData(ofxOusterIMUData & );
    
    ofxLidarSlamResults processLidarData(ouster::LidarScan &);
    void processImuData(ofxOusterIMUData & );
    
    
    void _update(ofEventArgs&);
    
    ofThreadChannel<ouster::LidarScan> toSlamScan;
    ofThreadChannel<ofxOusterIMUData> toSlamIMU;
    ofThreadChannel<ofxLidarSlamResults> fromSlam;
    
    void threadedFunction();
    
private:
    
    uint8_t PreviousMapOutputMode = OutputKeypointsMapsMode::FULL_MAPS;
    
    
    
    const double TimeToSecondsFactor = 1e-9;          ///< Coef to apply to TimeArray values to express time in seconds
    
    
    // SLAM initialization
    std::string InitMapPrefix; ///< Path prefix of initial maps
    Eigen::Vector6d InitPose;  ///< Initial pose of the SLAM
    
    
    ofEventListeners listeners;

    
    std::shared_ptr<ofxOuster> lidar = nullptr;
    
    vector<TrajectoryPoint> trajectory;

    ofPolyline TrajectoryLine;

    
    ofxLidarSlamResults slamResults;
    
    
//    ofVboMesh keypointMeshes [LidarSlam::nKeypointTypes];
    
//    void getKeypoints(LidarSlam::Keypoint k);

    
    ofEasyCam cam;

    
    void setParamsListeners();

    
    size_t frameCount = 0;
    
    ofxPointShader pointShader;
    
    
};

