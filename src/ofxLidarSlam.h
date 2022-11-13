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
#include "LidarSlam/Enums.h"

#include "ouster/lidar_scan.h"
//#include "ofxSpinningSensorKeypointExtractor.h"
#include <LidarSlam/SpinningSensorKeypointExtractor.h>
//! Which keypoints' maps to output
#include "ofxOuster.hpp"
#include "ofxOusterIMU.h"
#include "ofxOusterRenderer.hpp"
#include "ofxLidarSlamParameters.h"
#include "ofxPointShader.h"

#include "ofxGui.h"

#include "ofxMeshSaver.h"

#ifdef HAS_OFXGRABCAM
#include "ofxGrabCam.h"
#endif

#include "ofxLidarSlamTrajectoryPoint.h"
//
enum OutputKeypointsMapsMode : uint8_t
{
    //! No maps output
    NONE = 0,
    //! Output the whole keypoints' maps
    FULL_MAPS = 1,
    //! Output the target sub maps used for the current frame registration
    SUB_MAPS = 2
};

//enum AccumulateMode : uint8_t{
//    DONT_ACCUM = 0,
//    DISTANCE = 1,
//    FRAMES = 2
//};




class ofxLidarSlamResults{
public:
    ofxLidarSlamResults();
    
    ofxLidarSlamTrajectoryPoint trajectoryPoint;
    
    ofVboMesh RegisteredMap;
    vector<ofVboMesh> maps;

    vector<ofVboMesh> keypoints;
        
    bool bValid = false;
    
    void saveMaps(string timestamp);

    void clear();
};



class ofxLidarSlam: public ofThread
{
public:
    
    // I dont know why they implemented this in the SLAM library. By default it does not use the BLOB type, and its resources are inited in the constructor so it is kinda hard coded and not possible to init post construction without modifying the library, which I'd rather dont.
    static const std::map<LidarSlam::Keypoint, bool> UseKeypoints; //= {{LidarSlam::EDGE, true}, {LidarSlam::INTENSITY_EDGE, true}, {LidarSlam::PLANE, true}, {LidarSlam::BLOB, false}};
    static const std::vector<LidarSlam::Keypoint> UsableKeypoints; //= {LidarSlam::EDGE, LidarSlam::INTENSITY_EDGE, LidarSlam::PLANE};

    
    
    ofxLidarSlam();
    virtual ~ofxLidarSlam();
    
    void setup(std::shared_ptr<ofxOuster>& lidar);
    
    // ---------------------------------------------------------------------------
    //   General stuff and flags
    // ---------------------------------------------------------------------------
    
    void reset();
    
    // Initialization
    void setInitialMap(const std::string& mapsPathPrefix);
    void setInitialPoseTranslation(double x, double y, double z);
    void setInitialPoseRotation(double roll, double pitch, double yaw);
    
   
    
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
    
    void SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s);
  
    
    // ---------------------------------------------------------------------------
    //   Confidence estimator parameters
    // ---------------------------------------------------------------------------
    

    void SetAccelerationLimits(float linearAcc, float angularAcc);
    void SetVelocityLimits(float linearVel, float angularVel);

    // ---------------------------------------------------------------------------
    //   Rendering
    // ---------------------------------------------------------------------------

    void draw();
    
    void drawGui();
    
    // ---------------------------------------------------------------------------
    //   Others
    // ---------------------------------------------------------------------------
    
    unique_ptr<ofxLidarSlamParameters> params = nullptr;
    
    void printParams();
    string getCurrentSlamParamsAsString();
    
    const vector<ofxLidarSlamTrajectoryPoint>& getTrajectory(){return trajectory;};
    
    
    void saveRegisteredMeshes(string timestamp);

    void markCurrentFrame();
    
    std::mutex coutMutex;
    
protected:
    ofxMeshSaver meshSaver;
    
    void setDefaults();
    
private:
    ofxLidarSlam(const ofxLidarSlam&) = delete;
    void operator=(const ofxLidarSlam&) = delete;
    
    string createSaveDir(string timestamp);
    
    
    
    // ---------------------------------------------------------------------------
    //   Useful helpers
    // ---------------------------------------------------------------------------
    
    
    // Add current SLAM pose and covariance in WORLD coordinates to Trajectory.
    void AddCurrentPoseToTrajectory();
    
    
    LidarSlam::Slam::PointCloud::Ptr getPointCloudFromLidar(ouster::LidarScan & scan) ;
    
    // ---------------------------------------------------------------------------
    //   Member attributes
    // ---------------------------------------------------------------------------
    
    
    ofxOusterRenderer* getRenderer();
protected:
    
    std::unique_ptr<LidarSlam::Slam> SlamAlgo = nullptr;
    
    
    std::shared_ptr<LidarSlam::SpinningSensorKeypointExtractor> extractor = nullptr;
    
    /// callback on AdvanceReturnMode change
    void advancedReturnModeChanged(bool& _arg);
    
    void _timeWindowDurationChanged(float&);
    void _overlapSamplingRatioChanged(float&);

    
    
    void processLidarData(ouster::LidarScan &scan, ofxLidarSlamResults & results);
    void processImuData(ofxOusterIMUData & );
    
    
    void _update(ofEventArgs&);

    ofThreadChannel<ofxLidarSlamResults> fromSlam;
    
    void threadedFunction();
    
private:
    
    void drawMesh(vector<unique_ptr<ofParameter<bool>>>& bDraw ,  vector<ofVboMesh>& mesh, vector<unique_ptr<ofParameter<ofColor>>>& color);
    
    
    ofThreadChannel<ouster::LidarScan > toRenderer;
    
    unique_ptr<ofxOusterRenderer> _renderer = nullptr;
    
    std::atomic<bool> bMarkCurrent;
    
    uint8_t PreviousMapOutputMode = OutputKeypointsMapsMode::FULL_MAPS;
    
    const double TimeToSecondsFactor = 1e-9;          ///< Coef to apply to TimeArray values to express time in seconds
    
    
    // SLAM initialization
    
    Eigen::Vector6d InitPose;  ///< Initial pose of the SLAM
    
    
    ofEventListeners listeners;

    
    std::shared_ptr<ofxOuster> lidar = nullptr;
    
    vector<ofxLidarSlamTrajectoryPoint> trajectory;

    ofPolyline TrajectoryLine;
        
    
    ofxLidarSlamResults slamResults;
    
    void updateMaps(ofxLidarSlamResults & results);
    

    unique_ptr<
#ifdef HAS_OFXGRABCAM
    ofxGrabCam
#else
    ofEasyCam
#endif
    > cam = nullptr;

    
    void setParamsListeners();

    
    size_t frameCount = 0;
    
    ofxPointShader pointShader;
    
    glm::vec3 lastSavedPose = {0,0,0};
    
    
    string currentSavePath;
    
    
    vector<int32_t > markedFrames;
    
    
};

