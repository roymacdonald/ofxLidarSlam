//
//  ofxLidarSlamTrajectoryPoint.hpp
//  example
//
//  Created by Roy Macdonald on 11-10-22.
//

#pragma once
#include "ofMain.h"
#include "ofxPointShader.h"

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
        

        
    const ofNode& getNode(bool scaled = false);

    bool save(string savePath, size_t index, bool saveMesh = false);
    
private:
    
    ofNode node, scaledNode;
    bool bNeedsSetNode = true;
        
};

class ofxLidarSlamMesh{
public:
    ofxLidarSlamMesh(string name);
    
    glm::vec3 offset;
    
    ofVboMesh mesh;
    void copyMesh(ofVboMesh& m);
    
    ofColor color;
    ofParameter<bool> bDraw = {"", true};
    
    
    void draw(ofxPointShader& shader);
    
};
