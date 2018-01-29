#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

/*
    Read, write and save
 */
namespace cloud_io
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud_XYZ(std::string cloudfile);

    pcl::PointCloud<pcl::PointNormal>::Ptr load_cloud_Normal(std::string cloudfile);

    void save_cloud_XYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string 
    path);

    void save_cloud_Normal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string 
    path);

}

#endif
