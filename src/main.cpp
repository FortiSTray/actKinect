#include <iostream>

#include "opencv2\opencv.hpp"
#include "actKinect.h"

using namespace std;
using namespace cv;

int main()
{
	ActKinect actKinect;

	actKinect.InitDepthSensor();
	actKinect.InitColorSensor();

	while (true)
	{
		actKinect.updateDepth();
		actKinect.updateColor();

		if (waitKey(30) == VK_ESCAPE)
			break;
	}

	return 0;
}