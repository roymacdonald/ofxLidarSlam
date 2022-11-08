#include "ofApp.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



#define FILE_SETTINGS "file_Settings.json"

string validateFileInJson(ofJson& json, string key){
    if(json.contains(key)){
        if(ofFile::doesFileExist(json[key])){
            return json[key];
        }else{
            ofLogWarning("Opening Saved Datapaths")<<"File does not exist: " << json[key];
        }
    }else{
        ofLogWarning("Opening Saved Datapaths")<<"Invalid key name: " << key;
    }
    return "";
}

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofSetBackgroundAuto(false);
//    ofxGuiSetTextPadding(30);
    
    gui.setup();
    connectParamGroup.add(lidarIp);
    connectParamGroup.add(localIp);
    connectParamGroup.add(connect);
    connectParamGroup.add(bRecord);
    playbackParamGroup.add(openPcapDialogParam);
    playbackParamGroup.add(openPcapParam);
    
    gui.add(connectParamGroup);
    gui.add(playbackParamGroup);
    
    playback.setup();
    auto & pbGui = gui.getGroup(playbackParamGroup.getName());
    
    pbGui.add(&playback);
    pbGui.add(message.setup("",""));


    

    lidar = make_shared<ofxOuster>();
    slam.setup(lidar);

    // create listeners for the buttons
    listeners.push(playback.forwards.newListener([&](){
        if(lidar){
            if(!lidar->isPlaying()){
                lidar->nextFrame();
            }
        }
    }));

    listeners.push(playback.backwards.newListener([&](){
        if(lidar) lidar->firstFrame();
    }));
    
    listeners.push(playback.stop.newListener([&](){
        if(lidar) lidar->stop();
    }));
    
    listeners.push(playback.play.newListener([&](bool& play){
        if(lidar){
            if((play && lidar->isPlaying()) || (!play && !lidar->isPlaying())){
                // this is just in case the gui and player are "out of sync"
                if(lidar->isPlaying()){
                    lidar->pause();
                }else{
                    lidar->play();
                }
            }else{
                if(play){
                    lidar->play();
                }else{
                    lidar->pause();
                }
            }
        }
    }));
    
    listeners.push(connect.newListener([&](){
        // set to the correct IP address of both your computer and the lidar
        lidar->connect(lidarIp.get(), localIp.get());
    }));
    
    
    listeners.push(openPcapParam.newListener(this, &ofApp::tryOpenPcap));
    
    listeners.push(openPcapDialogParam.newListener(this, &ofApp::openPcapDialog));
    

    lidar->setGuiPosition(gui.getShape().getBottomLeft() + glm::vec2(0, 20));

    auto r = slam.params->gui.getShape();
    slam.params->gui.setPosition(ofGetWidth() - r.width - 20, 20);
    
    listeners.push(bRecord.newListener([&](bool&){
        if(lidar){
            if(bRecord.get()){
                auto res = ofSystemSaveDialog("LidarRecording_"+ofGetTimestampString(), "Choose where to save your recording");
            
                if(res.bSuccess){
                    recFile = res.getPath();
                    if(!lidar->recordToPCap(res.getPath())){
                        bRecord  = false;
                    }
                }else{
                    bRecord = false;
                }
            }else{
                if(lidar->isRecording()) lidar->endRecording();
                recFile = "";
            }
        }
    }));
    
    tooltips.registerGui(&gui, "tooltips.json");
    
    
}

//--------------------------------------------------------------
bool ofApp::openPcap(string pcap, string config){
    return lidar->load(pcap,config);
}
//--------------------------------------------------------------
void ofApp::openPcapDialog(){
    string pcap;
    string config;
    auto res = ofSystemLoadDialog("Select .pcap file");
    if(res.bSuccess){
        pcap = res.getPath();
        if(ofFile::doesFileExist(pcap)){
            
            config = ofFilePath::join(ofFilePath::getEnclosingDirectory(pcap),ofFilePath::getBaseName(pcap)) + ".json";
            if(!ofFile::doesFileExist(config)){
                res = ofSystemLoadDialog("Select .json configuration file");
                if(res.bSuccess){
                    config = res.getPath();
                }else{
                    config = "";
                }
            }
            if(openPcap(pcap, config)){
                ofJson json;
                json["PCAP"] = pcap;
                json["Config"] = config;
                
                ofSaveJson(FILE_SETTINGS, json);
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::tryOpenPcap(){
    string pcap, config;
    if(ofFile::doesFileExist(FILE_SETTINGS)){
        ofJson json = ofLoadJson(FILE_SETTINGS);
        
        pcap = validateFileInJson(json, "PCAP");
        config = validateFileInJson(json, "Config");
        
        if(!pcap.empty() && !config.empty() ){
            openPcap(pcap, config);
        }else{
            ofLogWarning("ofApp::openPcap") << "one of the filepaths is empty. Opening dialog";
            openPcapDialog();
        }
    }
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    
    slam.draw();
    
//    lidar->drawGui();
    gui.draw();
    
    slam.drawGui();
    
    
    if(bRecord.get()){
        ofColor bg (ofMap(sin(ofGetElapsedTimef()),-1, 1, 0, 255), 0,0);
        ofDrawBitmapStringHighlight("Recording into:\n" + recFile, gui.getShape().getTopRight() + glm::vec3(10, 0,0), bg);
    }
    
    if(bShowParams){
        ofDrawBitmapStringHighlight(currentParams, 20,20);
    }
    
    tooltips.draw();

    
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
    }else if(key == ' '){
        slam.markCurrentFrame();
//        slam.params.bEnableSlam = false;
//        slam.saveRegisteredMeshes();
        
//        auto & tr = slam.getTrajectory();
//
//        ofMesh mesh;
//        for(auto& t: tr){
//            mesh.addVertices(t.mesh.getVertices());
//        }
//
//        mesh.save(ofGetTimestampString()+".ply");
//
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
