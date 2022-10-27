//
//  ofxLidarSlamUtils.h
//  example
//
//  Created by Roy Macdonald on 26-10-22.
//

#pragma once
#include <pcl/common/transforms.h>

//#include "ofxTimeMeasurements.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ofVboMesh.h"
#include "LidarSlam/Slam.h"

namespace ofxLidarSlamUtils{
static void PclToOf( LidarSlam::Slam::PointCloud::Ptr pc, ofVboMesh& mesh){
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
static void AddPclToOf( LidarSlam::Slam::PointCloud::Ptr pc, ofVboMesh& mesh){
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
}

