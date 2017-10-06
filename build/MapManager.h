#pragma once
#include <Kinect.h>
class MapManager
{
public:
	MapManager();
	~MapManager();
private:
	ICoordinateMapper* mapper = nullptr;
	mySensor->get_CoordinateMapper(&mapper);

	UINT32 corTag;
	PointF*pointfs;
	mapper->GetDepthFrameToCameraSpaceTable(&corTag, &pointfs);
};

