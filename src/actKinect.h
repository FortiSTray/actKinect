#pragma once
#ifndef __ACT_KINECT
#define __ACT_KINECT




#include "kinect.h"
#include "opencv2\opencv.hpp"

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
	
	void InitDepthSensor();
	void InitColorSensor();
	void InitCoordinateMap();


	void updateDepth();
	void updateColor();



private:
	IKinectSensor*     pKinectSensor;
	IDepthFrameReader* pDepthReader;
	IColorFrameReader* pColorReader;
	IDepthFrame*       pDepthFrame;
	IColorFrame*       pColorFrame;
//
	ICoordinateMapper * mapper;
	
	UINT32 DF_2_CSCount;
	PointF*DF_2_CSTable;
	UINT16*ary16;
	ColorSpacePoint *colorSpacePoints;
//
	cv::Mat depthTemp;
	cv::Mat colorTemp;

	cv::Mat depthImage;
	cv::Mat colorImage;
	int depthHeight = 0, depthWidth = 0;
	int colorHeight = 0, colorWidth = 0;
};

#endif //__ACT_KINECT_V2
