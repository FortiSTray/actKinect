#include "actKinect.h"

ActKinect::ActKinect() : pKinectSensor(NULL), pDepthReader(NULL), pColorReader(NULL), 
						 pDepthFrame(NULL), pColorFrame(NULL), pCoordinateMapper(NULL)
{
	//open kinect sensor
	GetDefaultKinectSensor(&pKinectSensor);
	pKinectSensor->Open();

	//get coordinate mapper
	pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
}
ActKinect::~ActKinect()
{
	SafeRelease(pCoordinateMapper);

	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);

	if (pKinectSensor)
	{
		pKinectSensor->Close();
	}
	SafeRelease(pKinectSensor);
}

void ActKinect::initDepthSensor()
{
	//get frame source
	IDepthFrameSource* pDepthSource = nullptr;
	pKinectSensor->get_DepthFrameSource(&pDepthSource);

	//get height and width
	IFrameDescription *depthDescription = nullptr;
	pDepthSource->get_FrameDescription(&depthDescription);
	depthDescription->get_Height(&depthHeight);
	depthDescription->get_Width(&depthWidth);
	depthDescription->Release();

	//open reader
	pDepthSource->OpenReader(&pDepthReader);

	//initialize depth image
	depthTemp  = cv::Mat::zeros(depthHeight, depthWidth, CV_16UC1);
	depthImage = cv::Mat::zeros(depthHeight, depthWidth, CV_8UC1 );
}

void ActKinect::initColorSensor()
{
	//get frame source
	IColorFrameSource* pColorSource = nullptr;
	pKinectSensor->get_ColorFrameSource(&pColorSource);

	//get height and width
	IFrameDescription *colorDescription = nullptr;
	pColorSource->get_FrameDescription(&colorDescription);
	colorDescription->get_Height(&colorHeight);
	colorDescription->get_Width(&colorWidth);
	colorDescription->Release();

	//open reader
	pColorSource->OpenReader(&pColorReader);

	//initialize color image
	colorTemp = cv::Mat::zeros(colorHeight, colorWidth, CV_8UC4);
	colorImage = cv::Mat::zeros(colorHeight, colorWidth, CV_8UC3);
}

void ActKinect::updateDepth()
{
	if (pDepthReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
	{
		pDepthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)depthTemp.data);
		depthTemp.convertTo(depthImage, CV_8UC1, 255.0 / 4500);
		cv::imshow("Depth", depthImage);
		pDepthFrame->Release();
	}
}

void ActKinect::updateColor()
{
	if (pColorReader->AcquireLatestFrame(&pColorFrame) == S_OK)
	{
		pColorFrame->CopyConvertedFrameDataToArray(colorHeight*colorWidth * 4, colorTemp.data, ColorImageFormat_Bgra);
		cv::cvtColor(colorTemp, colorImage, CV_BGRA2BGR);
		cv::imshow("Color", colorImage);
		pColorFrame->Release();
	}
}

void ActKinect::coordinateMapping()
{
	//map coordinate from depth image to color image
	ColorSpacePoint* pColorCoordinates = new ColorSpacePoint[depthHeight * depthWidth];
	pCoordinateMapper->MapDepthFrameToColorSpace(depthHeight * depthWidth, (UINT16*)depthTemp.data,
		depthHeight * depthWidth, pColorCoordinates);

	//draw depthToColor image
	std::vector<BYTE> depthBuffer(depthWidth * depthHeight * 4);
	Concurrency::parallel_for(0, depthHeight, [&](const int depthY) {
		const unsigned int depthOffset = depthY * depthWidth;
		for (int depthX = 0; depthX < depthWidth; depthX++) {
			unsigned int depthIndex = depthOffset + depthX;
			const int colorX = static_cast<int>(pColorCoordinates[depthIndex].X + 0.5f);
			const int colorY = static_cast<int>(pColorCoordinates[depthIndex].Y + 0.5f);
			if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
				const unsigned int colorIndex = (colorY * colorWidth + colorX) * 4;
				depthIndex = depthIndex * 4;
				depthBuffer[depthIndex + 0] = colorTemp.data[colorIndex + 0];
				depthBuffer[depthIndex + 1] = colorTemp.data[colorIndex + 1];
				depthBuffer[depthIndex + 2] = colorTemp.data[colorIndex + 2];
				depthBuffer[depthIndex + 3] = colorTemp.data[colorIndex + 3];
			}
		}
	});
	cv::Mat depthToColor = cv::Mat(depthHeight, depthWidth, CV_8UC4, &depthBuffer[0]).clone();
	cv::imshow("DtoC", depthToColor);

	delete[] pColorCoordinates;
}