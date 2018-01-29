#include "../include/feature.h"

/*
    * Implementation Feature_XYZ
*/

namespace feature
{

    Feature_XYZ::Feature_XYZ() : mValue(0), mCloud(nullptr) {}

    Feature_XYZ::Feature_XYZ(float pValue, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){mValue = pValue; mCloud = pCloud;}

    Feature_XYZ::Feature_XYZ(float pValue, std::string cloudfile){mValue = pValue; mCloud = cloud_io :: load_cloud_XYZ(cloudfile);}

    float Feature_XYZ::get_value(){return mValue;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr Feature_XYZ::get_cloud(){return mCloud;}

    void Feature_XYZ::set_value(float pValue){mValue = pValue;}

    void Feature_XYZ::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){mCloud = pCloud;}

    void Feature_XYZ::set_cloud(std::string cloudfile){mCloud = cloud_io :: load_cloud_XYZ(cloudfile);}

    std::ostream& operator<< (std::ostream& os, Feature_XYZ& f)
    {
        os << f.get_value() << std::endl;
        return os;
    }

    /*
        * Implementation Feature_Normal
    */


    Feature_Normal :: Feature_Normal() : mValue(0), mCloud(nullptr) {}

    Feature_Normal :: Feature_Normal(float pValue, pcl::PointCloud<pcl::PointNormal>::Ptr pCloud){mValue = pValue; mCloud = pCloud;}

    Feature_Normal :: Feature_Normal(float pValue, std::string cloudfile){mValue = pValue; mCloud = cloud_io ::load_cloud_Normal(cloudfile);}

    float Feature_Normal ::get_value(){return mValue;}

    pcl::PointCloud<pcl::PointNormal>::Ptr Feature_Normal ::get_cloud(){return mCloud;}

    void Feature_Normal ::set_value(float pValue){mValue = pValue;}

    void Feature_Normal ::set_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloud){mCloud = pCloud;}

    void Feature_Normal ::set_cloud(std::string cloudfile){mCloud = cloud_io ::load_cloud_Normal(cloudfile);}
            
    std::ostream& operator<< (std::ostream& os, Feature_Normal& f)
    {
        os << f.get_value() << std::endl;
        return os;
    }

}
