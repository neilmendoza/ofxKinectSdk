/*
 *  KinectPlayer.cpp
 *
 *  Copyright (c) 2012, Neil Mendoza, http://www.neilmendoza.com
 *  All rights reserved. 
 *  
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met: 
 *  
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of Neil Mendoza nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission. 
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *  POSSIBILITY OF SUCH DAMAGE. 
 *
 */
#include "KinectPlayer.h"

namespace itg
{
	KinectPlayer::KinectPlayer() : playing(false)
	{}

	void KinectPlayer::loadSkeletalData(const string& fileName)
	{
		for (unsigned i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) recordedTracked[i] = false;
		if (skeletonPlayStream.is_open()) skeletonPlayStream.close();
		string path = ofToDataPath(fileName);
		ofLogNotice() << "Playing skeleton data from " << path;
		skeletonPlayStream.open(path.c_str());
	}

	void KinectPlayer::loadDepthMovie(const string& fileName)
	{
		ofLogFatalError() << "This function is not implemented yet";
	}

    void KinectPlayer::update()
	{
		if (skeletonPlayStream.is_open() && playing)
		{
			string line;
			if (!getline(skeletonPlayStream, line))
			{
				skeletonPlayStream.clear();
				skeletonPlayStream.seekg(ifstream::beg);
				if (!getline(skeletonPlayStream, line)) ofLogError() << "Error trying to loop skeleton playback, is file empty?";
			}
			if (!line.empty()) parsePlayback(line);
		}
	}

	void KinectPlayer::stop()
	{
		playing = false;
	}

	void KinectPlayer::play()
	{
		playing = true;
	}

	void KinectPlayer::parsePlayback(const string& line)
	{
		vector<string> split = ofSplitString(line, ":");
		for (unsigned i = 0; i < split.size(); ++i)
		{
			if (split[i].empty()) recordedTracked[i] = false;
			else
			{
				recordedTracked[i] = true;
				vector<string> xyz = ofSplitString(split[i], ",");
				recordedPositions[i].set(atof(xyz[0].c_str()), atof(xyz[1].c_str()), atof(xyz[2].c_str()));
			}
		}
	}

	void KinectPlayer::drawSkeletons()
	{
		// Render Torso
		drawBone(NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
		drawBone(NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
		drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
		drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);

		// Left Arm
		drawBone(NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
		drawBone(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
		drawBone(NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);

		// Right Arm
		drawBone(NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
		drawBone(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
		drawBone(NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

		// Left Leg
		drawBone(NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
		drawBone(NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
		drawBone(NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);

		// Right Leg
		drawBone(NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
		drawBone(NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
		drawBone(NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
	}

	void KinectPlayer::drawBone(unsigned joint0, unsigned joint1)
	{
		if (recordedTracked[joint0] && recordedTracked[joint1])
		{
			ofPushStyle();
			ofSetColor(0, 255, 0);
			ofLine(recordedPositions[joint0], recordedPositions[joint1]);
			ofPopStyle();
		}
	}
}