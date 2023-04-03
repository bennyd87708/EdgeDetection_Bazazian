// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcd_file.h"

using namespace std;

int main()
{
	pcd_file::write_file();
	pcd_file::read_file();
	cin.get();
	return 0;
}
