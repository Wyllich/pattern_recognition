#include "../include/cluster.h"

namespace cluster{
    
    /*
    * Constructors, getters, setters
    */
    
    Cluster_XYZ :: Cluster_XYZ() : mCloud(nullptr), mFeats() {}
    void Cluster_XYZ :: set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){mCloud = pCloud;}
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cluster_XYZ :: get_cloud(){return mCloud;}
    void Cluster_XYZ :: set_cloud(std::string cloudfile){mCloud = cloud_io :: load_cloud_XYZ(cloudfile);}

    void Cluster_XYZ :: add_feature(std::string pName, feature::Feature_XYZ pFeature)
    {
        if(mFeats.find(pName) != mFeats.end())
            std::cout << "Warning : Previous value of " << pName << " overwritten" << '\n';
        mFeats[pName]=pFeature;
    }

    void Cluster_XYZ :: modify_feature(std::string pName, feature::Feature_XYZ pFeature)
    {
        if(mFeats.find(pName) == mFeats.end())
            std::cout << "Warning : Value of " << pName << " created" << '\n';
        mFeats[pName]=pFeature;
    }

    feature::Feature_XYZ Cluster_XYZ :: get_feature(std::string pName)
    {
        if(mFeats.find(pName) != mFeats.end())
            return mFeats[pName];
        else
            throw "Non existent feature";
    }

    void Cluster_XYZ :: print_features()
    {
        std::cout << "The features of the cluster are : \n";
        for(auto p : mFeats)
            std::cout << "  " << p.first << " : " << p.second;
    }

    /*
        Geometrical features
     */
     
    feature::Feature_XYZ Cluster_XYZ :: calc_centerX()
    {
        float x_coords{.0f};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            x_coords += mCloud->points[i].x;
        
        x_coords /= mCloud->points.size();
        
        feature::Feature_XYZ feat{};
        feat.set_value(x_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_centerY()
    {
        float y_coords{.0f};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            y_coords += mCloud->points[i].y;
        
        y_coords /= mCloud->points.size();
        
        feature::Feature_XYZ feat{};
        feat.set_value(y_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_centerZ()
    {
        float z_coords{.0f};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            z_coords += mCloud->points[i].z;
        
        z_coords /= mCloud->points.size();
        
        feature::Feature_XYZ feat{};
        feat.set_value(z_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_nb_points()
    {
        float numb{.0f};
        feature::Feature_XYZ feat{};
        feat.set_value(mCloud->points.size());
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Xmax()
    {
        float x_coords{mCloud->points[0].x};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            x_coords = ((mCloud->points[i].x > x_coords)?(mCloud->points[i].x):x_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(x_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Ymax()
    {
        float y_coords{mCloud->points[0].y};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            y_coords = ((mCloud->points[i].y > y_coords)?(mCloud->points[i].y):y_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(y_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Zmax()
    {
        float z_coords{mCloud->points[0].z};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            z_coords = ((mCloud->points[i].z > z_coords)?(mCloud->points[i].z):z_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(z_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Xmin()
    {
        float x_coords{mCloud->points[0].x};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            x_coords = ((mCloud->points[i].x < x_coords)?(mCloud->points[i].x):x_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(x_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Ymin()
    {
        float y_coords{mCloud->points[0].y};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            y_coords = ((mCloud->points[i].y < y_coords)?(mCloud->points[i].y):y_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(y_coords);
        return feat;
    }

    feature::Feature_XYZ Cluster_XYZ :: calc_Zmin()
    {
        float z_coords{mCloud->points[0].z};
        for(size_t i=0; i<mCloud->points.size(); ++i)
            z_coords = ((mCloud->points[i].z < z_coords)?(mCloud->points[i].z):z_coords);
        feature::Feature_XYZ feat{};
        feat.set_value(z_coords);
        return feat;
    }
        

     /*
        Contructors, Modifiers, Readers
     */
    Cluster_Normal :: Cluster_Normal() : mCloud(nullptr), mFeats() {}
    void Cluster_Normal :: set_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloud){mCloud = pCloud;}
    void Cluster_Normal :: set_cloud(std::string cloudfile){mCloud = cloud_io :: load_cloud_Normal(cloudfile);}
    
    void Cluster_Normal :: add_feature(std::string pName, feature::Feature_Normal pFeature)
    {
        if(mFeats.find(pName) != mFeats.end())
            std::cout << "Warning : Previous value of " << pName << " overwritten" << '\n';
        mFeats[pName]=pFeature;
    }
    
    void Cluster_Normal :: modify_feature(std::string pName, feature::Feature_Normal pFeature)
    {
        if(mFeats.find(pName) == mFeats.end())
            std::cout << "Warning : Value of " << pName << " created" << '\n';
        mFeats[pName]=pFeature;
    }
    
    feature::Feature_Normal Cluster_Normal :: get_feature(std::string pName)
    {
        if(mFeats.find(pName) != mFeats.end())
            return mFeats[pName];
        else
            throw "Non existent feature";
    }
    
    void Cluster_Normal :: print_features()
    {
        std::cout << "The features of the cluster are : \n";
        for(auto p : mFeats)
            std::cout << p.first << " : " << p.second;
    }
        
};
