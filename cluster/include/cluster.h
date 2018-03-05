#ifndef CLUSTER_H
#define CLUSTER_H

#include "../../feature/include/feature.h"
#include <map>

namespace cluster{

    class Cluster_XYZ
    {
        private :
        pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
        std::map<std::string, feature::Feature_XYZ> mFeats;
        
        public :
            /*
                Constructors, Getters, setters
             */
            Cluster_XYZ();
            void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
            void set_cloud(std::string cloudfile);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud();
            
            void add_feature(std::string pName, feature::Feature_XYZ pFeature);
            
            void modify_feature(std::string pName, feature::Feature_XYZ pFeature);
            
            feature::Feature_XYZ get_feature(std::string pName);
            
            void print_features();
            
            /*
                Geometrical features
             */
             
            feature::Feature_XYZ calc_centerX();
            
            feature::Feature_XYZ calc_centerY();
            
            feature::Feature_XYZ calc_centerZ();
            
            feature::Feature_XYZ calc_nb_points();
            
            feature::Feature_XYZ calc_Xmax();
            
            feature::Feature_XYZ calc_Ymax();
            
            feature::Feature_XYZ calc_Zmax();
            
            feature::Feature_XYZ calc_Xmin();
            
            feature::Feature_XYZ calc_Ymin();
            
            feature::Feature_XYZ calc_Zmin();
            
    };

    class Cluster_Normal
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr mCloud;
        std::map<std::string, feature::Feature_Normal> mFeats;
        
        public :
             /*
                Contructors, Modifiers, Readers
             */
            Cluster_Normal();
            void set_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloud);
            void set_cloud(std::string cloudfile);
            
            void add_feature(std::string pName, feature::Feature_Normal pFeature);
            
            void modify_feature(std::string pName, feature::Feature_Normal pFeature);
            
            feature::Feature_Normal get_feature(std::string pName);
            
            void print_features();
            
    };
}
#endif
