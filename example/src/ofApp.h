#pragma once
#include "ofxGui.h"
#include "ofxOuster.hpp"
#include "ofxLidarSlam.h"
#include "ofxPlaybackGui.h"
#include "ofxGuiTooltip.h"
class ofApp : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();

        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y );
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void mouseEntered(int x, int y);
        void mouseExited(int x, int y);
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);

    

    ofxPanel gui;

    ofParameterGroup connectParamGroup = {"Connect to Lidar"};
    ofParameterGroup playbackParamGroup = {"Playback PCAP file"};

    ofxPlaybackGui playback;

    ofxGuiTooltip tooltips;
    
    ofxLabel message;

    ofEventListeners listeners;

    std::shared_ptr<ofxOuster> lidar = nullptr;

    ofxLidarSlam slam;
    
    ofParameter<void> connect = {"Connect"};

    ofParameter<void> openPcapDialogParam = {"Open PCAP"};
    ofParameter<void> openPcapParam = {"Open Last PCAP"};

    ofParameter<string> lidarIp = {"Lidar's IP", "192.168.0.155"};
    ofParameter<string> localIp = {"Computer's IP", "192.168.0.100"};
    
    ofParameter<bool> bRecord = {"Record to PCAP", false};
    
    void openPcapDialog();
    bool openPcap(string pcap, string config);
    void tryOpenPcap();
    
    
    
    string recFile;

    
};
