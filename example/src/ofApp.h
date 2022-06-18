#pragma once
#include "ofxGui.h"
#include "ofxOuster.hpp"
#include "ofxLidarSlam.h"
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


    ofParameter<float> point_size = {"Point Size", 3, 1, 10};

    ofEventListeners listeners;

    std::shared_ptr<ofxOuster> lidar = nullptr;

    ofxLidarSlam slam;
    
    ofParameter<void> connect = {"Connect"};

    string currentParams;
    bool bShowParams = false;
    
    
};
