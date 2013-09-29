#pragma once

#include "ofxKinectSdk.h"
#include "ofMain.h"

class testApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();

private:
	ofxKinectSdk kinect;
};
