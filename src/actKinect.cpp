#include "actKinect.h"

using namespace std;
using namespace cv;

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
		//cv::imshow("Depth", depthImage);
		pDepthFrame->Release();
	}
}

void ActKinect::updateColor()
{
	if (pColorReader->AcquireLatestFrame(&pColorFrame) == S_OK)
	{
		pColorFrame->CopyConvertedFrameDataToArray(colorHeight*colorWidth * 4, colorTemp.data, ColorImageFormat_Bgra);
		cv::cvtColor(colorTemp, colorImage, CV_BGRA2BGR);
		//cv::imshow("Color", colorImage);
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

void ActKinect::detectBall()
{
	getForeground();

	cvtColor(fgImage, fgImage, COLOR_BGR2GRAY);

	cv::Mat equaImage(fgImage.rows, fgImage.cols, CV_8UC1);
	equalizeHist(fgImage, equaImage);
	imshow("equa", equaImage);

	GaussianBlur(fgImage, fgImage, Size(5, 5), 2, 2);
	medianBlur(equaImage, equaImage, 5);
	imshow("bulr", equaImage);

	cv::Mat cannyImage(fgImage.rows, fgImage.cols, CV_8UC1);
	Canny(equaImage, cannyImage, 60, 120);
	imshow("canny", cannyImage);


	cv::HoughCircles(fgImage, circles, CV_HOUGH_GRADIENT, 2.5, 3000, 170, 40, 5, 40);

	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		circle(depthImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);
	}
	imshow("ballPosition", depthImage);
}

void ActKinect::getForeground()
{
	cvtColor(depthToColor, depthToColor, COLOR_BGRA2BGR);
	Mat realImage = depthToColor.clone();
	//resize(depthToColor, realImage, Size(320, 240));

	if (fgImage.empty())
	{
		fgImage.create(realImage.size(), realImage.type());
	}

	//update the model
	cvtColor(realImage, realImage, CV_BGRA2BGR);

	bgModel(realImage, fgMask, record ? -1 : 0);

	fgImage = Scalar::all(0);

	realImage.copyTo(fgImage, fgMask);

	Mat bgImage;
	bgModel.getBackgroundImage(bgImage);

	//imshow("image", realImage);
	//imshow("foreground mask", fgMask);
	imshow("foreground image", fgImage);
	//if (!bgImage.empty())
	//{
	//	imshow("mean background image", bgImage);
	//}
}