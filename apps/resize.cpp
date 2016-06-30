#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main(int argc, char ** argv)
{
    using namespace std;

    if(argc!=2)
    {
        cout << "input argument error, exit." << endl;
        return EXIT_FAILURE;
    }

    string file = argv[1];
    string filename = file.substr(0, file.find('.'));

    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::PLYReader reader;
    reader.read(file.c_str(), *cloud);

    for(size_t i=0; i<cloud->points.size(); ++i)
    {
        cloud->points[i].x /= 1000.0;
        cloud->points[i].y /= 1000.0;
        cloud->points[i].z /= 1000.0;
    }

    string fout = filename+".pcd";
    pcl::PCDWriter writer;
    writer.write(fout.c_str(), *cloud);

    return 0;
}
