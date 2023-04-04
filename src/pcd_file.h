#pragma once

namespace pcd_file
{
    int write_file();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr read_file(std::string filename);
    void print_points(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
}