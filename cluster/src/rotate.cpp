#include <iostream>
#include "../include/cluster.h"
#include <math.h>

#define PI 3.14159265

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
    * Create a fixed number of rotated subclouds
    */
     
    int const NUM_ANGLE{12};
    std::vector<float> rot_angle(NUM_ANGLE);
    for(int i=0; i<NUM_ANGLE; ++i)
        rot_angle[i] = 2*i*PI/NUM_ANGLE;
    
    for(auto angle : rot_angle)
    {       
        pcl::PointCloud<pcl::PointXYZ> out;
        out.width = in->width;
        out.height = 1;
        out.is_dense = false;
        out.points.resize(out.width*out.height);
        
        for(size_t i=0; i<out.points.size(); i++)
        {   
            out.points[i].x = ((in->points[i].x)*cos(angle)) - ((in->points[i].y)*sin(angle));
            out.points[i].y = ((in->points[i].x)*sin(angle)) + ((in->points[i].y)*cos(angle));
            out.points[i].z = in->points[i].z;
        }
        
        std::string s = "/home/wyllich/c_"+std::to_string(angle)+".pcd";
        std::cout << s << std::endl;
        
        pcl::io::savePCDFileASCII (s, out);
    }       
    
    return 0;
}
