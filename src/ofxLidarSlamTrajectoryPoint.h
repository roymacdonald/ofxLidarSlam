//
//  ofxLidarSlamTrajectoryPoint.hpp
//  example
//
//  Created by Roy Macdonald on 11-10-22.
//

#pragma once
#include "ofMain.h"

class ofxLidarSlamTrajectoryPoint{
public:
    double x;
    double y;
    double z;
    double time;
    glm::dquat quaternion;
    double axisAngle [4];
    double covariance [36];
    bool isMarker = false;
    
    ofxLidarSlamTrajectoryPoint * prev = nullptr;
        
    ofVboMesh mesh;
        
    const ofNode& getNode(bool scaled = false);
    void copyMesh(ofVboMesh& m);
    bool save(string savePath, size_t index, bool saveMesh = false);
    
private:
    
    ofNode node, scaledNode;
    bool bNeedsSetNode = true;
        
};
