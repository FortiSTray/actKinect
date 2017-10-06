#include <iostream>

#include "opencv2\opencv.hpp"
#include "actKinect.h"

using namespace std;
using namespace cv;

int main()
{
	ActKinect actKinect;

	actKinect.initDepthSensor();
	actKinect.initColorSensor();

	while (true)
	{
		actKinect.updateDepth();
		actKinect.updateColor();

		actKinect.coordinateMapping();

		if (waitKey(15) == VK_ESCAPE)
			break;
	}

	return 0;
}