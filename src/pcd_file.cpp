#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcd_file.h"

namespace pcd_file
{
	int write_file()
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;

		// Fill in the cloud data
		cloud.width = 5;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.resize(cloud.width * cloud.height);

		for (auto& point : cloud)
		{
			point.x = 1024 * rand() / (RAND_MAX + 1.0f);
			point.y = 1024 * rand() / (RAND_MAX + 1.0f);
			point.z = 1024 * rand() / (RAND_MAX + 1.0f);
		}

		pcl::io::savePCDFileASCII("../../../../data/test_pcd.pcd", cloud);
		std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

		for (const auto& point : cloud)
			std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

		return 0;
	}

	int read_file()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../../../data/test_pcd.pcd", *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file\n");
			return (-1);
		}
		std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;

		for (const auto& point : *cloud)
			std::cout << "    " << point.x
			<< " " << point.y
			<< " " << point.z << std::endl;

		return 0;
	}
}
