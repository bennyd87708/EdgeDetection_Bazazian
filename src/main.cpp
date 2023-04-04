// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcd_file.h"

using namespace std;

int main()
{
	pcd_file::write_file();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = pcd_file::read_file("test_pcd.pcd");
	pcd_file::print_points(cloud);
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}

	cin.get();
	return 0;
}
