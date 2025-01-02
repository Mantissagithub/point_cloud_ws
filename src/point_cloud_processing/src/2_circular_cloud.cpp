#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    double radius = 3.0;
    int num_points = 50;
    double angular_set_size = 2*M_PI/num_points;

    // pcl::PointXYZRGB point;
    // point.x = 1.0;
    // point.y = 2.0;
    // point.z = 3.0;

    // point.r = 255;
    // point.g = 255;
    // point.b = 0;

    // cloud.push_back(point);

    for(int i=0;i<num_points;i++){
        pcl::PointXYZRGB point;
        double angle = i*angular_set_size;
        point.x = radius * cos(angle);
        point.y = radius * sin(angle);
        point.z = 1.0;

        point.r = 255*cos(angle);
        point.g = 255*sin(angle);
        point.b = 255*cos(angle + M_PI_2);
        cloud.push_back(point);
    }

    string path = "/home/pradheep/pc_ws/src/point_cloud_processing/point_clouds/circular_cloud.pcd";
    pcl::io::savePCDFileASCII(path, cloud);

    return 0;
}