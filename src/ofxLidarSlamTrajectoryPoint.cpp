//
//  ofxLidarTrajectoryPoint.cpp
//  example
//
//  Created by Roy Macdonald on 11-10-22.
//

#include "ofxLidarSlamTrajectoryPoint.h"

const ofNode& ofxLidarSlamTrajectoryPoint::getNode(bool scaled){
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




void ofxLidarSlamTrajectoryPoint::copyMesh(ofVboMesh& m){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    mesh.addVertices(m.getVertices());
    
    
}

bool ofxLidarSlamTrajectoryPoint::save(string savePath, size_t index, bool saveMesh){
    if(saveMesh && mesh.getVertices().size()){
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
    j["isMarker"] = isMarker;
    
    
    return ofSaveJson(savePath + "/metadata/"+ofToString(index) + ".json", j);
    
}
