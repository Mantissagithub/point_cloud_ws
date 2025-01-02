// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>

// using namespace std;
// using namespace pcl;
// int main(){

//     pcl::PointCloud2::Ptr cloud (new PointCloud2());
//     pcl::PCDReader cloud_reader;

//     std::path = "/home/pradheep/pc_ws/src/point_cloud_processing/point_clouds/table_scene.pcd"
//     cloud_Reader.read(path, *cloud);

//     std::cout<<"Number of point: "<< cloud->width * cloud->height << endl;
//     return 0;
// }

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main() {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    PCDReader cloud_reader;

    string path = "/home/pradheep/pc_ws/src/point_cloud_processing/point_clouds/table_scene.pcd";

    if (cloud_reader.read(path, *cloud) == -1) {
        cerr << "Error loading PCD file: " << path << endl;
        return -1;
    }

    cout << "Number of points: " << cloud->width * cloud->height << endl;
    return 0;
}
