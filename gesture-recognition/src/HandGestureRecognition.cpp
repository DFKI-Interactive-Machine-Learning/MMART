#include "HandGestureRecognition.hpp"

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
	evt->device = PMDNANO_DEVICE;
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

void HandGestureRecognition::reconfigure()
{
	m_stream_gestures = nui::config()->get("Gestures.Stream.raw", false);
}

void HandGestureRecognition::process(nui::events::PMDNanoEvent& evt)
{
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<pcl::PointXYZ> >(
			new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(evt.filteredCloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(evt.filteredCloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);
	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size()
			<< std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size()
			<< " points." << endl;
	std::cout << "These are the indices of the points of the initial"
			<< std::endl << "cloud that belong to the first cluster:"
			<< std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;
}

NUISTATUS HandGestureRecognition::initialize()
{

	return NUI_SUCCESS;
}

NUISTATUS HandGestureRecognition::unInitialize()
{
	return NUI_SUCCESS;
}

