#include <iostream>

#include "opencv2\opencv.hpp"
#include "actKinect.h"

using namespace std;
using namespace cv;

//#define DEBUG
#ifdef NDEBUG
#define NODE /##/
#else
#define NODE 
#endif // DEBUG


int main()
{
	ActKinect actKinect;

	actKinect.InitDepthSensor();
	actKinect.InitColorSensor();
	actKinect.InitCoordinateMap();

	while (true)
	{
		actKinect.updateDepth();
		actKinect.updateColor();

		if (waitKey(30) == VK_ESCAPE)
			break;
	}

	return 0;
}