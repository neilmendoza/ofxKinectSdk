/*
 *  KinectSdk.cpp
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
#include "KinectSdk.h"

namespace itg
{
	const float KinectSdk::UNIT_SCALARS[NUM_UNITS] = {1.f, 100.f, 1000.f};

	ofVec3f KinectSdk::toOf(const Vector4& ms)
	{
		return ofVec3f(ms.x, ms.y, ms.z);
	}

	KinectSdk::KinectSdk() : frameNew(false), depthResolution(NUI_IMAGE_RESOLUTION_640x480), nearWhite(true), depthStreamHandle(INVALID_HANDLE_VALUE)
	{
		setUnit(M);
		setDepthClipping();
	}

	bool KinectSdk::init(bool useSkeleton, bool useDepth)
	{
		INuiSensor* pNuiSensor;

		int iSensorCount = 0;
		HRESULT hr = NuiGetSensorCount(&iSensorCount);
		if (FAILED(hr)) return false;

		// Look at each KinectSdk sensor
		for (int i = 0; i < iSensorCount; ++i)
		{
			// Create the sensor so we can check status, if we can't create it, move on to the next
			hr = NuiCreateSensorByIndex(i, &pNuiSensor);
			if (FAILED(hr))
			{
				continue;
			}

			// Get the status of the sensor, and if connected, then we can initialize it
			hr = pNuiSensor->NuiStatus();
			if (S_OK == hr)
			{
				sensor = pNuiSensor;
				break;
			}

			// This sensor wasn't OK, so release it since we're not using it
			pNuiSensor->Release();
		}

		if (NULL != sensor)
		{
			// Initialize the KinectSdk and specify that we'll be using skeleton
			hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_DEPTH); 
			if (SUCCEEDED(hr) && useSkeleton)
			{
				// Create an event that will be signaled when skeleton data is available
				nextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

				// Open a skeleton stream to receive skeleton data
				hr = sensor->NuiSkeletonTrackingEnable(nextSkeletonEvent, 0); 
			}

			if (SUCCEEDED(hr) && useDepth)
			{
				nextDepthFrameEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

				hr = sensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_DEPTH,
					depthResolution,
					0,
					2,
					nextDepthFrameEvent,
					&depthStreamHandle);
			}
			
			DWORD w, h;
			NuiImageResolutionToSize(depthResolution, w, h);
			depthW = w;
			depthH = h;

			depthPixelsRaw.allocate(depthW, depthH, 1);
			depthPixels.allocate(depthW, depthH, 1);

			depthPixelsRaw.set(0);
			depthPixels.set(0);

			depthTex.allocate(depthW, depthH, GL_LUMINANCE);
		}

		if (NULL == sensor || FAILED(hr))
		{
			ofLogError() << "No ready KinectSdk found!";
			return E_FAIL;
		}

		return hr;
	}

	void KinectSdk::setUnit(Unit unit)
	{
		unitScalar = UNIT_SCALARS[unit];
		this->unit = unit;
	}

	bool KinectSdk::isFrameNew()
	{
		bool temp = frameNew;
		frameNew = false;
		return temp;
	}

    void KinectSdk::update()
	{
		if (!sensor) return;

		// Wait for 0ms, just quickly test if it is time to process a skeleton
		if (useSkeleton && WAIT_OBJECT_0 == WaitForSingleObject(nextSkeletonEvent, 0) )
		{
			HRESULT hr = sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
			if ( FAILED(hr) ) return;

			// smooth out the skeleton data
			sensor->NuiTransformSmooth(&skeletonFrame, NULL);

			//sensor->Nuigetd

			frameNew = true;
		}

		// Wait for 0ms, just quickly test if it is time to process a skeleton
		if (useDepth && WAIT_OBJECT_0 == WaitForSingleObject(nextDepthFrameEvent, 0) )
		{
			HRESULT hr = sensor->NuiImageStreamGetNextFrame(
				depthStreamHandle,
				0,
				&depthFrame
			);
			INuiFrameTexture *pTexture = depthFrame.pFrameTexture;

			NUI_LOCKED_RECT lockedRect;

			pTexture->LockRect(0, &lockedRect, NULL, 0);
			if( lockedRect.Pitch != 0 )
			{
				depthPixelsRaw.setFromPixels((unsigned short*)lockedRect.pBits, depthW, depthH, 1);
				//memcpy((unsigned char *)depthPixelsRaw.getPixels(), lockedRect.pBits, depthW * depthH * DEPTH_BYTES_PER_PIXEL);
				//memcpy(depthBufferRawChars, lockedRect.pBits, depthW * depthH * DEPTH_BYTES_PER_PIXEL);
			}
			pTexture->UnlockRect( 0 );

			sensor->NuiImageStreamReleaseFrame( depthStreamHandle, &depthFrame );

			updateDepthPixels();
			depthTex.loadData(depthPixels.getPixels(), depthW, depthH, GL_LUMINANCE);

			frameNew = true;
		}
	}

	/*
	void KinectSdk::depthToMat(cv::Mat& mat)
	{

	}*/


	NUI_SKELETON_POSITION_TRACKING_STATE KinectSdk::getSkeletonPosition(ofVec3f& position, unsigned skeletonIdx, NUI_SKELETON_POSITION_INDEX joint)
	{
		NUI_SKELETON_POSITION_TRACKING_STATE jointState = skeletonFrame.SkeletonData[skeletonIdx].eSkeletonPositionTrackingState[joint];

		if (jointState != NUI_SKELETON_POSITION_NOT_TRACKED)
			position = unitScalar * toOf(skeletonFrame.SkeletonData[skeletonIdx].SkeletonPositions[joint]);

		return jointState;
	}

	vector<unsigned> KinectSdk::getSkeletonIndices(NUI_SKELETON_TRACKING_STATE trackingState)
	{
		vector<unsigned> indices;
		for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
		{
			if (trackingState == skeletonFrame.SkeletonData[i].eTrackingState) indices.push_back(i);
		}
		return indices;
	}

	unsigned KinectSdk::getNumSkeletons(NUI_SKELETON_TRACKING_STATE trackingState)
	{
		return getSkeletonIndices().size();
	}

	void KinectSdk::drawSkeletons(bool screenSpace)
	{
		for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
		{
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

			if (trackingState == NUI_SKELETON_TRACKED)
			{
				// We're tracking the skeleton, draw it
				drawSkeleton(skeletonFrame.SkeletonData[i], ofGetWidth(), ofGetHeight(), screenSpace);
			}
			else if (NUI_SKELETON_POSITION_ONLY == trackingState)
			{
			}
		}
	}

	void KinectSdk::drawSkeleton(const NUI_SKELETON_DATA& skeletonData, int width, int height, bool screenSpace)
	{
		for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
		{
			screenPoints[i] = skeletonToScreen(skeletonData.SkeletonPositions[i], width, height);
		}

		// Render Torso
		drawBone(skeletonData, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, screenSpace);

		// Left Arm
		drawBone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT, screenSpace);

		// Right Arm
		drawBone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT, screenSpace);

		// Left Leg
		drawBone(skeletonData, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, screenSpace);

		// Right Leg
		drawBone(skeletonData, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, screenSpace);
		drawBone(skeletonData, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, screenSpace);

		/*
		// Draw the joints in a different color
		for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
		{
			D2D1_ELLIPSE ellipse = D2D1::Ellipse( m_Points[i], g_JointThickness, g_JointThickness );

			if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED )
			{
				m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointInferred);
			}
			else if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED )
			{
				m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
			}
		}*/
	}

	ofVec2f KinectSdk::skeletonToScreen(Vector4 skeletonPoint, int width, int height)
	{
		LONG x, y;
		USHORT depth;

		// Calculate the skeleton's position on the screen
		// NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
		NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

		float screenPointX = static_cast<float>(x * width) / 320.f;
		float screenPointY = static_cast<float>(y * height) / 240.f;

		return ofVec2f(screenPointX, screenPointY);
	}

	
	void KinectSdk::drawBone(const NUI_SKELETON_DATA& skeletonData, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1, bool screenSpace)
	{
		NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skeletonData.eSkeletonPositionTrackingState[joint0];
		NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skeletonData.eSkeletonPositionTrackingState[joint1];

		// If we can't find either of these joints, exit
		if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
		{
			return;
		}

		// Don't draw if both points are inferred
		if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
		{
			return;
		}

		ofPushStyle();
		// We assume all drawn bones are inferred unless BOTH joints are tracked
		if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED) ofSetColor(0, 255, 0);
		else ofSetColor(255, 0, 0);
		if (screenSpace) ofLine(screenPoints[joint0], screenPoints[joint1]);
		else ofLine(toOf(skeletonData.SkeletonPositions[joint0]), toOf(skeletonData.SkeletonPositions[joint1]));
		ofPopStyle();
	}

	void KinectSdk::updateDepthLookupTable()
	{
		unsigned char nearColor = nearWhite ? 255 : 0;
		unsigned char farColor = nearWhite ? 0 : 255;
		unsigned int maxDepthLevels = 10001;
		depthLookupTable.resize(maxDepthLevels);
		depthLookupTable[0] = 0;
		for(unsigned int i = 1; i < maxDepthLevels; i++) {
			depthLookupTable[i] = ofMap(i, nearClipping, farClipping, nearColor, farColor, true);
		}
	}

	void KinectSdk::setDepthClipping(float nearClip, float farClip)
	{
		nearClipping = nearClip;
		farClipping = farClip;
		updateDepthLookupTable();
	}

	void KinectSdk::updateDepthPixels()
	{
		int n = depthW * depthH;
		for(int i = 0; i < n; i++)
		{
			short raw = depthPixelsRaw[i];
			if (raw > depthLookupTable.size() - 1) depthPixels[i] = depthLookupTable.back();
			else depthPixels[i] = depthLookupTable[raw];
		}
	}
}