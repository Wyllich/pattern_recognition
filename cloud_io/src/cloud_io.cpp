#include "../include/cloud_io.h"
#include <pcl/io/pcd_io.h>

/*
    Read, write and save
 */
namespace cloud_io
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud_XYZ(std::string cloudfile)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(cloudfile, *cloud)==-1)
        {
            PCL_ERROR ("Error while reading file. \n");
        }
        std::cout << "Successfully loaded cloud from " << cloudfile << std::endl;
        return cloud; 
    };

    pcl::PointCloud<pcl::PointNormal>::Ptr load_cloud_Normal(std::string cloudfile)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        if(pcl::io::loadPCDFile<pcl::PointNormal>(cloudfile, *cloud)==-1)
        {
            PCL_ERROR ("Error while reading file. \n");
        }
        std::cout << "Successfully loaded cloud from " << cloudfile << std::endl;
        return cloud; 
    };

    void save_cloud_XYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string 
    path)
    {
        pcl::io::savePCDFileASCII (path, *cloud);
    };

    void save_cloud_Normal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string 
    path)
    {
        pcl::io::savePCDFileASCII (path, *cloud);
    };

}
