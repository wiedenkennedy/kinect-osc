#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	ofxCvColorImage colorImg;
	ofxCvGrayscaleImage grayImageA; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNearA; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFarA; // the far thresholded image
	ofxCvContourFinder contourFinderA;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
	ofxCvGrayscaleImage grayImageB; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNearB; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFarB; // the far thresholded image
	ofxCvContourFinder contourFinderB;
#endif
	
	ofxOscSender oscOut;

	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	float closestObject;
	
	int angle;
	
    // used for viewing the point cloud
	ofEasyCam easyCam;
};
