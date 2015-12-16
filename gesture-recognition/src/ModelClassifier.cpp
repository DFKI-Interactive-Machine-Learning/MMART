#include "ModelClassifier.hpp"

using namespace nui::events;
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

inline void createEvent(NUIEvent* evt, bool stream)
{
	evt->timestamp = timestamp_now();
	if (stream)
	{
		evt->processingtype = OUTPUT_EVENT;
	}
	else
	{
		evt->processingtype = INTERNAL_EVENT;
	}
}

void ModelClassifier::reconfigure()
{
}

void ModelClassifier::process(nui::events::LeapTrackingEvent& evt)
{
	
}

void ModelClassifier::process(nui::events::MyoEvent& evt)
{

}
NUISTATUS ModelClassifier::initialize()
{

	return NUI_SUCCESS;
}

NUISTATUS ModelClassifier::unInitialize()
{
	return NUI_SUCCESS;
}

