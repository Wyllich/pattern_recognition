#include <iostream>
#include "../include/cluster.h"

using namespace cloud_io;

int main(int argc, char** argv)
{
    /* 
    * Loading the input cloud and creating a container
    */
    
    std::string path_in{argv[1]};
    pcl::PointCloud<pcl::PointXYZ>::Ptr in = load_cloud_XYZ(path_in);
    
    cluster::Cluster_XYZ c{};
    c.set_cloud(path_in);
    
    /* 
    * Set the floor at z = 0 
    */
     
    feature::Feature_XYZ zmin = c.calc_Zmin();
    c.add_feature("minZ", zmin);
    
    feature::Feature_XYZ x_g = c.calc_centerX();
    c.add_feature("center_X", x_g);
    
    feature::Feature_XYZ y_g = c.calc_centerY();
    c.add_feature("center_Y", y_g);
    
    pcl::PointCloud<pcl::PointXYZ> out;
    out.width = in->width;
    out.height = 1;
    out.is_dense = false;
    out.points.resize(out.width*out.height);
    
    for(size_t i=0; i<out.points.size(); i++)
    {
        out.points[i].x = in->points[i].x - x_g.get_value();
        out.points[i].y = in->points[i].y - y_g.get_value();
        out.points[i].z = in->points[i].z - zmin.get_value();
    }
    
    std::string s = "/home/wyllich/c.pcd";
    
    pcl::io::savePCDFileASCII (s, out);
    
    return 0;
}
