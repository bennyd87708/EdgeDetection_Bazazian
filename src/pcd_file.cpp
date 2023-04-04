#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcd_file.h"

namespace pcd_file
{
	int write_file()
	{
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;

		// Fill in the cloud data
		cloud.width = 50;
		cloud.height = 50;
		cloud.is_dense = false;
		cloud.resize(cloud.width * cloud.height);

		for (auto& point : cloud)
		{
			point.x = 1024 * rand() / (RAND_MAX + 1.0f);
			point.y = 1024 * rand() / (RAND_MAX + 1.0f);
			point.z = 1024 * rand() / (RAND_MAX + 1.0f);
			point.r = 255 * rand();
			point.g = 255 * rand();
			point.b = 255 * rand();
			point.a = 255;
		}

		pcl::io::savePCDFileASCII("../../../../data/test_pcd.pcd", cloud);
		std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

		for (const auto& point : cloud)
			std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

		return 0;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr read_file(std::string filename)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../../../../data/" + filename, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file\n");
			exit(0);
		}
		std::cout << "Loaded "
				  << cloud->width * cloud->height
				  << " data points from "
				  << filename
				  << std::endl;

		return cloud;
	}

	void print_points(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
	{
		for (const auto& point : *cloud)
			std::cout << "    " << point.x
			<< " " << point.y
			<< " " << point.z << std::endl;
	}
}
