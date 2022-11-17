#pragma once
#include "ofMain.h"
#include "ofxOpen3d.h"

enum MergeState {
    MERGE_NONE = 0,
    MERGE_MERGING,
    MERGE_DOWNSAMPLING,
    MERGE_SAVING,
    MERGE_DONE
};

class MergeInfo{
public:
    string txt;
        
    void setState(MergeState state){
        if(this->state != state){
            this->state = state;
            if(state == MERGE_MERGING){
                txt += "Merging...";
                updateRect();
            }else if(state == MERGE_DOWNSAMPLING){
                txt += " done!.\n";
                txt += "Downsampling...";
                updateRect();
            }else if(state == MERGE_SAVING){
                txt += " done!.\n";
                txt += "Saving...";
                updateRect();
            }else if(state == MERGE_DONE){
                txt += " done!.\n";
                doneTime = ofGetElapsedTimeMillis();
                updateRect();
            }
        }
    }
    
    bool shouldKill(){
        return (state == MERGE_DONE && doneTime > 0 && (ofGetElapsedTimeMillis() - doneTime > 2000) );
    }
    
    void draw(){
        int margin = 20;
        int x = ((ofGetWidth() - txtRect.width)*0.5) - margin;
        int y = 15;
        
        int alpha = 255;
        if(state == MERGE_DONE){
            alpha = ofMap(ofGetElapsedTimeMillis(), doneTime + 500, doneTime+ 2000, 255, 0, true);
            
        }
        ofSetColor(ofColor::red, alpha);
        ofDrawRectRounded(x, y, txtRect.width + 2 * margin, txtRect.height + 2 * margin, 5);
        
        ofSetColor(ofColor::white, alpha);
        
        ofDrawBitmapString(txt, x + margin, y + margin - txtRect.y);
    }
    
    
    
private:
    
    void updateRect(){
        ofBitmapFont bf;
        txtRect = bf.getBoundingBox(txt, 0,0);
    }
    
    ofRectangle txtRect;
    
    uint64_t doneTime = 0;
        
        
    MergeState state = MERGE_NONE;
};


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
        _bIsDone = false;
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

    bool isDone(){
        return _bIsDone.load();
    }
    

private:
    void threadedFunction(){
        
        ofxMeshSaverArgs args;
        
        while(toSave.receive(args)){
            args.mesh.save(args.filepath);
            if(toSave.size() == 0){
                _bIsDone = true;
            }
        }
    }

	ofThreadChannel<ofxMeshSaverArgs> toSave;
    std::atomic<bool> _bIsDone;
};


class ofxMeshMerger: public ofThread {
public:
    
    ofxMeshMerger(){
        startThread();
    }

    ~ofxMeshMerger(){
        toMerge.close();
        
        waitForThread(true);
    }

    void clear(){
        std::unique_lock<std::mutex> lock(mutex);
        mergedMesh.clear();
        _bIsMergeDone = false;
    }
    
    void add(ofMesh & mesh){
        toMerge.send(mesh);
    }

    bool isMergeDone(){
        return _bIsMergeDone.load();
    }
    
    
    const ofMesh& getMergedMesh(){
        std::unique_lock<std::mutex> lock(meshMutex);
        return mergedMesh;
    }

private:
    
    ofMesh mergedMesh;
    
    void threadedFunction(){
        
        ofMesh mesh;
        
        while(toMerge.receive(mesh)){
            auto& mm = mergedMesh.getVertices();
            auto &mv = mesh.getVertices();
            if(mv.size() > 0){
                mm.insert(mm.end(), mv.begin(), mv.end());
            }
            if(toMerge.size() == 0){
                _bIsMergeDone = true;
            }
        }
    }

    ofThreadChannel<ofMesh> toMerge;
    std::atomic<bool> _bIsMergeDone = false;
    std::mutex mutex;
    std::mutex meshMutex;
};



class ofxMeshDownsampler: public ofThread {
public:
    
    ofxMeshDownsampler(){
        outputVoxelSize = 1;
        startThread();
    }

    ~ofxMeshDownsampler(){
        toDownsample.close();
        fromDownsample.close();
        waitForThread(true);
    }

    
    void downsample(ofMesh & mesh, float outputVoxelSize){
        this->outputVoxelSize = outputVoxelSize;
        toDownsample.send(mesh);
    }

    
    ofThreadChannel<ofMesh> fromDownsample;
    
private:
    void threadedFunction(){
        
        ofMesh mesh;
        while(toDownsample.receive(mesh)){
            
            PclPointCloud pcl_mesh, pcl_mesh_down;
            ofxOpen3d::toOpen3d(mesh, pcl_mesh );
            pcl_mesh_down = pcl_mesh->VoxelDownSample(outputVoxelSize.load() );
            ofMesh downsampledMesh;
            ofxOpen3d::toOF(pcl_mesh_down, downsampledMesh);
            fromDownsample.send(std::move(downsampledMesh));
        }
    }
    
    ofThreadChannel<ofMesh> toDownsample;
    

    std::atomic<float> outputVoxelSize;
    
};



