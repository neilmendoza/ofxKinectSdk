/*
 *  KinectSdk.h
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
#pragma once

#include <Windows.h>
#include <NuiApi.h>
#include "ofMain.h"
#include "ofxCv.h"

namespace itg
{
	

	class KinectSdk
	{
	public:
		enum Unit
		{
			M,
			CM,
			MM,

			NUM_UNITS
		};

		static const unsigned DEPTH_BYTES_PER_PIXEL = 2;

		static const float UNIT_SCALARS[NUM_UNITS];
		static ofVec3f toOf(const Vector4& ms);

		KinectSdk();

		bool init(bool useSkeleton = true, bool useDepth = false);
		void update();
		void drawSkeletons(bool screenSpace = true);
		void drawSkeleton(const NUI_SKELETON_DATA& skeletonData, int width, int height, bool screenSpace = true);
		void drawBone(const NUI_SKELETON_DATA& skeletonData, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1, bool screenSpace = true);
		unsigned getNumSkeletons(NUI_SKELETON_TRACKING_STATE trackingState = NUI_SKELETON_TRACKED);
		vector<unsigned> getSkeletonIndices(NUI_SKELETON_TRACKING_STATE trackingState = NUI_SKELETON_TRACKED);
		NUI_SKELETON_POSITION_TRACKING_STATE getSkeletonPosition(ofVec3f& position, unsigned skeletonIdx, NUI_SKELETON_POSITION_INDEX joint);

		NUI_SKELETON_FRAME& getSkeletonFrameRef() { return skeletonFrame; }

		bool isFrameNew();
		//void depthToMat(cv::Mat& mat);

		Unit getUnit() const { return unit; }
		void setUnit(Unit unit);

		void setDepthClipping(float nearClip=500, float farClip=4000);

		ofTexture& getDepthTextureRef() { return depthTex; }

		short rawDepth(unsigned x, unsigned y);
		short rawDepth(unsigned idx);

		unsigned getDepthW() const { return depthW; }
		unsigned getDepthH() const { return depthH; }

		ofVec3f getWorldCoordinateAt(int cx, int cy);

	private:
		void updateDepthPixels();
		vector<unsigned char> depthLookupTable;
		void updateDepthLookupTable();
		float nearClipping, farClipping;
		bool nearWhite;
		ofTexture depthTex;

		Unit unit;
		float unitScalar;

		INuiSensor* sensor;
		HANDLE nextSkeletonEvent; // HANDLE is void*
		HANDLE nextDepthFrameEvent; // HANDLE is void*
		HANDLE depthStreamHandle; // HANDLE is void*

		NUI_SKELETON_FRAME skeletonFrame;
		
		//cv::Mat depthMat;
		ofVec2f screenPoints[NUI_SKELETON_POSITION_COUNT];
		ofVec2f skeletonToScreen(Vector4 skeletonPoint, int width, int height);
		bool frameNew;
		bool useSkeleton, useDepth;
		NUI_IMAGE_FRAME depthFrame;

		ofShortPixels depthPixelsRaw;
		ofPixels depthPixels;
		unsigned char* depthBufferRaw;

		NUI_IMAGE_RESOLUTION depthResolution;
		unsigned depthW, depthH;
	};
}