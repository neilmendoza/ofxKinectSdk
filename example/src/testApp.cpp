#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
	ofBackground(0);
	kinect.init();
}

//--------------------------------------------------------------
void testApp::update()
{
	kinect.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	kinect.drawSkeletons(true);
}