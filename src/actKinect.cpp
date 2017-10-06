#include "actKinect.h"

ActKinect::ActKinect() : pKinectSensor(NULL), pDepthReader(NULL), pColorReader(NULL), 
						 pDepthFrame(NULL), pColorFrame(NULL)
{
	GetDefaultKinectSensor(&pKinectSensor);
	pKinectSensor->Open();
	pKinectSensor->get_CoordinateMapper(&mapper);

}
ActKinect::~ActKinect()
{
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);

	if (pKinectSensor)
	{
		pKinectSensor->Close();
	}
	SafeRelease(pKinectSensor);
}

void ActKinect::InitDepthSensor()
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

void ActKinect::InitColorSensor()
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

void ActKinect::InitCoordinateMap()
{
	
	//
	mapper->GetDepthFrameToCameraSpaceTable(&DF_2_CSCount, &DF_2_CSTable);
	
	colorSpacePoints = new ColorSpacePoint[depthHeight*depthWidth];
	
	//mapper->GetDepthFrameToCameraSpaceTa)
	//
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
		//
		mapper->MapDepthFrameToColorSpace(depthHeight*depthWidth, (UINT16*)depthTemp.data, depthHeight*depthWidth, colorSpacePoints);
		cv::Mat gray;
		cv::cvtColor(colorImage, gray, CV_BGR2GRAY);
	/*	for (int i = 0; i < depthHeight*depthWidth; i++)
		{
			colorImage.data[(int)(colorSpacePoints[i].X + colorSpacePoints[i].Y)]=gray.data[i];
		}*/
		std::vector<BYTE> buffer(depthWidth * depthHeight * 4);

		for (int depthY = 0; depthY < depthHeight; depthY++) {
			const unsigned int depthOffset = depthY * depthWidth;
			for (int depthX = 0; depthX < depthWidth; depthX++) {
				unsigned int depthIndex = depthOffset + depthX;
				const int colorX = static_cast<int>(colorSpacePoints[depthIndex].X + 0.5f);
				const int colorY = static_cast<int>(colorSpacePoints[depthIndex].Y + 0.5f);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					const unsigned int colorIndex = (colorY * colorWidth + colorX) * 4;
					depthIndex = depthIndex * 4;
					buffer[depthIndex + 0] = colorTemp.data[colorIndex + 0];
					buffer[depthIndex + 1] = colorTemp.data[colorIndex + 1];
					buffer[depthIndex + 2] = colorTemp.data[colorIndex + 2];
					buffer[depthIndex + 3] = colorTemp.data[colorIndex + 3];
				}
			}
		}
		colorImage = cv::Mat(depthHeight, depthWidth, CV_8UC4, &buffer[0]).clone();
		//
		cv::imshow("Color", colorImage);
		pColorFrame->Release();
	}
}