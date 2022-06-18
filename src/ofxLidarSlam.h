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

struct TrajectoryPoint{
    double x;
    double y;
    double z;
    double time;
    glm::quat quaternion;
    double axisAngle [4];
    double covariance [36];
    
    
    glm::mat4 getTransformMatrix(){
        ofNode n;
        n.setOrientation(quaternion);
        n.setPosition({x, y, z});
        return n.getGlobalTransformMatrix();
    }
    
    
    ofVboMesh mesh;
    
    void copyMesh(ofVboMesh& m){
        mesh.setMode(OF_PRIMITIVE_POINTS);
        mesh.addVertices(m.getVertices());
        
        
//        for(auto& v: mesh.getVertices()){
//            range.check(v);
//        }
//        range.print();
    }
//    Range3f range;
    
    
};



class ofxLidarSlam
{
public:
    ofxLidarSlam();
    
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
    ofVboMesh RegisteredMap;
    ofVboMesh EdgeMap;
    ofVboMesh PlanarMap;
    ofVboMesh BlobMap;
    
    ofVboMesh EdgeKeypoints;
    ofVboMesh PlanarKeypoints;
    ofVboMesh BlobKeypoints;
    
//    ofVboMesh keypointMeshes [LidarSlam::nKeypointTypes];
    
//    void getKeypoints(LidarSlam::Keypoint k);

    
    ofEasyCam cam;

    
    void setParamsListeners();

    
    size_t frameCount = 0;
    
    ofxPointShader pointShader;
    
    
};
