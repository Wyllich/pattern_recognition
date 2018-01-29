#ifndef FEATURE_H
#define FEATURE_H

#include "/home/wyllich/projet3A/cloud_io/include/cloud_io.h"

namespace feature
{

    class Feature_XYZ
    {
        float mValue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
        
        public :
            Feature_XYZ();
            Feature_XYZ(float pValue, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
            Feature_XYZ(float pValue, std::string cloudfile);
            float get_value();
            pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud();
            void set_value(float pValue);
            void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
            void set_cloud(std::string cloudfile);
    };

    std::ostream& operator<< (std::ostream& os, Feature_XYZ& f);

    class Feature_Normal
    {
        float mValue;
        pcl::PointCloud<pcl::PointNormal>::Ptr mCloud;
        
        public :
            Feature_Normal();
            Feature_Normal(float pValue, pcl::PointCloud<pcl::PointNormal>::Ptr pCloud);
            Feature_Normal(float pValue, std::string cloudfile);
            float get_value();
            pcl::PointCloud<pcl::PointNormal>::Ptr get_cloud();
            void set_value(float pValue);
            void set_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloud);
            void set_cloud(std::string cloudfile);
    };

    std::ostream& operator<< (std::ostream& os, Feature_Normal& f);

}
#endif
