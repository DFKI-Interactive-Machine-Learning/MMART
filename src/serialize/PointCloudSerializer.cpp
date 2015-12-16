#include "serialize/PointCloudSerializer.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/date_time/posix_time/posix_time.hpp>

void pcl_serialize(PointCloud::Ptr& cloud, const bfs::path path)
{
	 pcl::io::savePCDFileBinaryCompressed (path.string(), *cloud.get());
}

void pcl_serialize(PointCloudRGBA::Ptr& cloud, const bfs::path path)
{
	pcl::io::savePCDFileBinaryCompressed (path.string(), *cloud.get());
	
}