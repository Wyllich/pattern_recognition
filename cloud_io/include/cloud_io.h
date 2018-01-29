#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

namespace cloud_io
{
    /*
    Read and save
    */
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud_XYZ(std::string cloud_path);
    
    pcl::PointCloud<pcl::PointXYZL>::Ptr load_cloud_XYZL(std::string cloud_path);

    pcl::PointCloud<pcl::PointNormal>::Ptr load_cloud_Normal(std::string cloud_path);

    void save_cloud_XYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string 
    cloud_path);
    
    void save_cloud_XYZL(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, std::string 
    cloud_path);

    void save_cloud_Normal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string 
    cloud_path);

}

#endif
