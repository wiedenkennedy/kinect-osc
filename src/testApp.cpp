#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
    // enable depth->rgb image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init(true);
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImageA.allocate(kinect.width, kinect.height);
	grayThreshNearA.allocate(kinect.width, kinect.height);
	grayThreshFarA.allocate(kinect.width, kinect.height);
	
	nearThreshold = 255;
	farThreshold = 0;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 10;
	kinect.setCameraTiltAngle(angle);
	
#ifdef USE_TWO_KINECTS
	kinect2.setCameraTiltAngle(angle);
#endif
	
	// start from the front
	bDrawPointCloud = false;
	// OSC Setup
	// open an outgoing connection to HOST:PORT
	//oscOut.setup( "10.3.4.23", 9000 );
	oscOut.setup( "127.0.0.1", 9001 );
	
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	

	
	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// reset the closest Object
		closestObject = 10000;
		
		// load grayscale depth image from the kinect source
		grayImageA.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNearA = grayImageA;
			grayThreshFarA = grayImageA;
			grayThreshNearA.threshold(nearThreshold, true);
			grayThreshFarA.threshold(farThreshold);
			cvAnd(grayThreshNearA.getCvImage(), grayThreshFarA.getCvImage(), grayImageA.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImageA.getPixels();
			
			int numPixels = grayImageA.getWidth() * grayImageA.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImageA.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinderA.findContours(grayImageA, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
	
	if(kinect2.isFrameNew()) {
		
		// reset the closest Object
		//closestObject = 10000;
		
		// load grayscale depth image from the kinect source
		grayImageB.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
	/*
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNearB = grayImageB;
			grayThreshFarB = grayImageB;
			grayThreshNearB.threshold(nearThreshold, true);
			//grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNearB.getCvImage(), grayThreshFarB.getCvImage(), grayImageB.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImageB.getPixels();
			
			int numPixels = grayImageB.getWidth() * grayImageB.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		*/
		// update the cv images
		grayImageB.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		//contourFinderB.findContours(grayImageB, 10, (kinect2.width*kinect2.height)/2, 20, false);
	}
#endif

	ofxOscMessage m;
	m.setAddress( "/presence" );
	bool presence = false;
	
	if(contourFinderA.blobs.size() > 0) {
		
		m.addIntArg( 1 );
		oscOut.sendMessage(m);
		m.clear();
		m.setAddress( "/numPeople");
		m.addIntArg( contourFinderA.blobs.size() );
		oscOut.sendMessage( m );
		m.clear();
		presence = true;
		
		for (int blobNum = 0; blobNum < contourFinderA.blobs.size(); blobNum++) {
			
			ofPoint* blobCentre = &contourFinderA.blobs[blobNum].centroid;
			float blobDistance = kinect.getDistanceAt(*blobCentre);
			closestObject = (closestObject < blobDistance) ? closestObject : blobDistance;
			
		}
		m.setAddress( "/closestObject" );
		m.addIntArg( closestObject );
		oscOut.sendMessage( m );
		m.clear();
		
	} else {
		/*m.addIntArg( 0 );
		oscOut.sendMessage( m );
		m.clear();*/
	}
	
#ifdef USE_TWO_KINECTS

	if(contourFinderB.blobs.size() > 0) {
		for (int blobNum = 0; blobNum < contourFinderB.blobs.size(); blobNum++) {
			
			ofPoint* blobCentre = &contourFinderB.blobs[blobNum].centroid;
			float blobDistance = kinect.getDistanceAt(*blobCentre);
			closestObject = (closestObject < blobDistance) ? closestObject : blobDistance;
			
		}
	}
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		//kinect.drawDepth(10, 10, 400, 300);
		//kinect.draw(420, 10, 400, 300);
		
		kinect.draw(10, 10, 400, 300);
		
		//grayImage.draw(10, 320, 400, 300);
		grayImageA.draw(10, 320, 400, 300);
		contourFinderA.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 10, 400, 300);
		//kinect2.draw(420, 320, 400, 300);
		grayImageB.draw(420, 320, 400, 300);
		//contourFinderA.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "p = point cloud" << endl
	<< "space = toggle opencv threshold = " << bThreshWithOpenCV << endl
	<< "near threshold " << nearThreshold << " (press: + -)" << endl
	<< "far threshold " << farThreshold << " (press: < >)" << endl
	<< "num blobs found " << contourFinderA.blobs.size() << endl
	<< "fps: " << ofGetFrameRate() << endl
	<< "c/o close open conn: " << kinect.isConnected() << endl
	<< "UP/DOWN tilt angle: " << angle << " degrees" << endl
	<< "Closest Object: " << closestObject << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect.setCameraTiltAngle(0);
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
