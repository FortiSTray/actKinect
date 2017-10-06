#pragma once
#ifndef __ACT_KINECT
#define __ACT_KINECT

#include "kinect.h"
#include "opencv2\opencv.hpp"
#include <time.h>

//Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface* &pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class ActKinect
{
public:
	ActKinect();
	~ActKinect();
	
	void initDepthSensor();
	void initColorSensor();

	void updateDepth();
	void updateColor();

	void coordinateMapping();

	void detectBall();


private:
	IKinectSensor*     pKinectSensor;
	IDepthFrameReader* pDepthReader;
	IColorFrameReader* pColorReader;
	IDepthFrame*       pDepthFrame;
	IColorFrame*       pColorFrame;

	ICoordinateMapper* pCoordinateMapper;

	cv::Mat depthTemp;
	cv::Mat colorTemp;
	cv::Mat depthImage;
	cv::Mat colorImage;
	cv::Mat depthToColor;

	std::vector<cv::Vec3f> circles;

	int depthHeight = 0, depthWidth = 0;
	int colorHeight = 0, colorWidth = 0;
};

#endif //__ACT_KINECT_V2
