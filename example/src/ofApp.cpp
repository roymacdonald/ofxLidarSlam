#include "ofApp.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetBackgroundAuto(false);
    gui.setup();
    gui.add(connect);
    gui.add(point_size);

    lidar = make_shared<ofxOuster>();
    slam.setup(lidar);

//    listeners.push(connect.newListener([&](){
//        lidar->setup("192.168.0.155", "192.168.0.100");
//        lidar->startThread();
//    }));
//
    
    listeners.push(connect.newListener([&](){
    lidar->load("/Users/roy/Desktop/park_stephan/2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.pcap",        "/Users/roy/Desktop/park_stephan/2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.json");
    }));
    
//    listeners.push(connect.newListener([&](){
//        lidar->load("/Users/roy/Downloads/OS0_128_freeway_sample/OS0_128_freeway_sample.pcap",
//                    "/Users/roy/Downloads/OS0_128_freeway_sample/OS0_2048x10_128.json", 47691, 37873);
//    }));
//
    lidar->setGuiPosition(gui.getShape().getBottomLeft() + glm::vec2(0, 20));

    auto r = slam.params.gui.getShape();
    slam.params.gui.setPosition(ofGetWidth() - r.width - 20, 20);
    
    
    listeners.push(point_size.newListener([&](float&){
        glPointSize(point_size.get());
    }));
    
    
    

}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    
    slam.draw();
    
    lidar->drawGui();
    gui.draw();
    slam.drawGui();
    
    
    if(bShowParams){
        ofDrawBitmapStringHighlight(currentParams, 20,20);
    }
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    if(key == 'p'){
        bShowParams ^= true;
        if(bShowParams){
            currentParams = slam.getCurrentSlamParamsAsString();
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
