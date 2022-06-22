#pragma once
#include "ofMain.h"
struct ofxMeshSaverArgs{
    ofxMeshSaverArgs (){}
    ofxMeshSaverArgs (string _filepath , ofMesh _mesh):
    filepath(_filepath),
    mesh(_mesh)
    {}
    
    string filepath;
    ofMesh mesh;
};

class ofxMeshSaver: public ofThread {
public:
    
    ofxMeshSaver(){
        startThread();
    }

    ~ofxMeshSaver(){
        toSave.close();
        
        waitForThread(true);
    }

    void save(ofMesh & mesh, string filepath){
        
        ofxMeshSaverArgs args(filepath, mesh);
        toSave.send(args);
    }


private:
    void threadedFunction(){
        
        ofxMeshSaverArgs args;
        while(toSave.receive(args)){
            args.mesh.save(args.filepath);
        }
    }

	ofThreadChannel<ofxMeshSaverArgs> toSave;
	
};
