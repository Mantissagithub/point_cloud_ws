#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(){
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    cloud.push_back(pcl::PointXYZ(7.0, 8.0, 9.0));
    cloud.push_back(pcl::PointXYZ(10.0, 11.0, 12.0));
    cloud.push_back(pcl::PointXYZ(13.0, 14.0, 15.0));

    string path = "/home/pradheep/pc_ws/src/point_cloud_processing/point_clouds/plane.pcd";

    pcl::io::savePCDFileASCII(path, cloud);
    cout<<cloud.size();
    return 0;
}