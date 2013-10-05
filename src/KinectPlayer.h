/*
 *  KinectPlayer.h
 *
 *  Copyright (c) 2013, Neil Mendoza, http://www.neilmendoza.com
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
#pragma once

#include "ofMain.h"
#include "BaseKinectData.h"

#ifndef _NUI_SKELETON_POSITION_INDEX_
#define _NUI_SKELETON_POSITION_INDEX_
typedef 
	enum _NUI_SKELETON_POSITION_INDEX
{	NUI_SKELETON_POSITION_HIP_CENTER	= 0,
NUI_SKELETON_POSITION_SPINE	= ( NUI_SKELETON_POSITION_HIP_CENTER + 1 ) ,
NUI_SKELETON_POSITION_SHOULDER_CENTER	= ( NUI_SKELETON_POSITION_SPINE + 1 ) ,
NUI_SKELETON_POSITION_HEAD	= ( NUI_SKELETON_POSITION_SHOULDER_CENTER + 1 ) ,
NUI_SKELETON_POSITION_SHOULDER_LEFT	= ( NUI_SKELETON_POSITION_HEAD + 1 ) ,
NUI_SKELETON_POSITION_ELBOW_LEFT	= ( NUI_SKELETON_POSITION_SHOULDER_LEFT + 1 ) ,
NUI_SKELETON_POSITION_WRIST_LEFT	= ( NUI_SKELETON_POSITION_ELBOW_LEFT + 1 ) ,
NUI_SKELETON_POSITION_HAND_LEFT	= ( NUI_SKELETON_POSITION_WRIST_LEFT + 1 ) ,
NUI_SKELETON_POSITION_SHOULDER_RIGHT	= ( NUI_SKELETON_POSITION_HAND_LEFT + 1 ) ,
NUI_SKELETON_POSITION_ELBOW_RIGHT	= ( NUI_SKELETON_POSITION_SHOULDER_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_WRIST_RIGHT	= ( NUI_SKELETON_POSITION_ELBOW_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_HAND_RIGHT	= ( NUI_SKELETON_POSITION_WRIST_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_HIP_LEFT	= ( NUI_SKELETON_POSITION_HAND_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_KNEE_LEFT	= ( NUI_SKELETON_POSITION_HIP_LEFT + 1 ) ,
NUI_SKELETON_POSITION_ANKLE_LEFT	= ( NUI_SKELETON_POSITION_KNEE_LEFT + 1 ) ,
NUI_SKELETON_POSITION_FOOT_LEFT	= ( NUI_SKELETON_POSITION_ANKLE_LEFT + 1 ) ,
NUI_SKELETON_POSITION_HIP_RIGHT	= ( NUI_SKELETON_POSITION_FOOT_LEFT + 1 ) ,
NUI_SKELETON_POSITION_KNEE_RIGHT	= ( NUI_SKELETON_POSITION_HIP_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_ANKLE_RIGHT	= ( NUI_SKELETON_POSITION_KNEE_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_FOOT_RIGHT	= ( NUI_SKELETON_POSITION_ANKLE_RIGHT + 1 ) ,
NUI_SKELETON_POSITION_COUNT	= ( NUI_SKELETON_POSITION_FOOT_RIGHT + 1 ) 
} 	NUI_SKELETON_POSITION_INDEX;

#endif _NUI_SKELETON_POSITION_INDEX_

namespace itg
{
	// Hacky Kinect Data Player
	// Plays back hacky kinect data skeletal data which at the moment
	// just consists of one skeleton, doesn not depend on Kinect SDK
	// to allow for playback on other platforms
	class KinectPlayer : public BaseKinectData
	{
	public:
		KinectPlayer();

		void loadSkeletalData(const string& fileName);

		void loadDepthMovie(const string& fileName);

		void update();
		
		void play();

		void stop();

		void drawSkeletons();
        
        ofVec3f getPosition(NUI_SKELETON_POSITION_INDEX joint);
        ofVec3f getPosition(unsigned joint);
        
	private:
		void drawBone(unsigned joint0, unsigned joint1);

		void parsePlayback(const string& line);
		
		ofVec3f recordedPositions[NUI_SKELETON_POSITION_COUNT];
		bool recordedTracked[NUI_SKELETON_POSITION_COUNT];

		bool playing;

		ifstream skeletonPlayStream;
	};
}