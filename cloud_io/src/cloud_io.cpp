#include "../include/cloud_io.h"
#include <iostream>

namespace cloud_io
{
    
    /*
    *  Load cloud from a path
    *  Supported point types are PointXYZ, PointXYZL, PointNormal
    */
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud_XYZ(std::string cloud_path)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, *cloud)==-1)
        {
            PCL_ERROR ("Error while reading file. \n");
        }
        std::cout << "Successfully loaded XYZ cloud from " << cloud_path << std::endl;
        return cloud; 
    };
    
    pcl::PointCloud<pcl::PointXYZL>::Ptr load_cloud_XYZL(std::string cloud_path)
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
        if(pcl::io::loadPCDFile<pcl::PointXYZL>(cloud_path, *cloud)==-1)
        {
            PCL_ERROR ("Error while reading file. \n");
        }
        std::cout << "Successfully loaded XYZL cloud from " << cloud_path << std::endl;
        return cloud; 
    };

    pcl::PointCloud<pcl::PointNormal>::Ptr load_cloud_Normal(std::string cloud_path)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        if(pcl::io::loadPCDFile<pcl::PointNormal>(cloud_path, *cloud)==-1)
        {
            PCL_ERROR ("Error while reading file. \n");
        }
        std::cout << "Successfully loaded Normal cloud from " << cloud_path << std::endl;
        return cloud; 
    };

    /*
    *  Save cloud into a path
    *  Supported point types are PointXYZ, PointXYZL, PointNormal
    */
    
    void save_cloud_XYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string 
    cloud_path)
    {
        pcl::io::savePCDFileASCII (cloud_path, *cloud);
    };

    void save_cloud_XYZL(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, std::string 
    cloud_path)
    {
        pcl::io::savePCDFileASCII (cloud_path, *cloud);
    };

    void save_cloud_Normal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string 
    cloud_path)
    {
        pcl::io::savePCDFileASCII (cloud_path, *cloud);
    };

}
