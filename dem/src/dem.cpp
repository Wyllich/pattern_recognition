#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <unistd.h> //getopt
#include <vector>
#include <iomanip> //setting precision of
#include <string>
#include "../../cloud_io/include/cloud_io.h"
#include "../include/dem.h"
#include "../../math/include/math.h"
#include "../../constant/include/constant.h"

using namespace cloud_io;

/*
 * Help functions
 */
 
int show_help(const char *s)
{
    std::cout << "Usage :  " << s << " [-option] [argument]" <<std::endl;
    std::cout << "Option : " << "-h : show help information" << std::endl
                             << "-o : Blind modelisation of groud using one seed point. \n Format : input.pcd output.pcd radius_search max_slope" << std::endl
                             << "-m : Blind modelisation of groud using multiple seed points. \n Format : input.pcd output.pcd radius_search max_slope num_seed" << std::endl;
    
    return 0;
}


int dem_one_seed(std::string cloud_path, std::string save_path, const float& radius_search, const float& max_slope)
{
    std::cout << "1. Loading input cloud ... \n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
    cloud = load_cloud_XYZL(cloud_path);
    std::cout << "Successfully loaded the input cloud from " << cloud_path << std::endl;
    
    
    std::cout << "2. Building the kdtree ... \n";
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);
    std::cout << "Kdtree successfully built." << std::endl;

    std::cout << "3. Calculating erosion values with parameters radius search = "<< radius_search 
                                                      << " and maximum slope = " << max_slope << " ..." << std::endl;   
    std::vector <float> erosion(cloud->points.size());
    for(size_t i=0; i<cloud->points.size(); ++i)
    {
        if(i%1000==0)
          std::cout << 100*i/cloud->points.size() << " percent completion.\n";

        float erosionValue(10000.0f);
        std::vector <int> indexRadiusSearch;
        std::vector <float> distancesRadiusSearch;
        
        if(kdtree.radiusSearch(cloud->points[i], radius_search, indexRadiusSearch, distancesRadiusSearch)>0)
        {
            float tempErosionValue = 10000.0f;
            for(size_t j=1; j<indexRadiusSearch.size(); ++j)
            {
                tempErosionValue = cloud->points[ indexRadiusSearch[j] ].z + 
                                   max_slope*sqrt( pow(cloud->points[i].x - cloud->points[indexRadiusSearch[j]].x,2)+
                                   pow(cloud->points[i].y - cloud->points[indexRadiusSearch[j]].y,2) ); 
                if(tempErosionValue < erosionValue)
                    erosionValue = tempErosionValue; 
            }
        }
        
        erosion[i] = erosionValue;
    }
    std::cout << "End of erosion calculation." <<std::endl;
    
    
    std::cout << "4. Labeling ground points (label = 2) and non ground points (label = 27) ...\n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud (new pcl::PointCloud<pcl::PointXYZL>);
    labeledCloud = load_cloud_XYZL(cloud_path);  
    for(size_t i=0; i<labeledCloud->points.size(); ++i)
        labeledCloud->points[i].label = ((labeledCloud->points[i].z < erosion[i]) ? GROUND_LABEL : NON_GROUND_LABEL);
    std::cout << "Successfully labeled " << labeledCloud->points.size() << " points." << std::endl;
    
    
    std::cout << "5. Saving labeled cloud ...\n";
    save_path.append("radius");
    save_path.append(std::to_string(radius_search));
    save_path.append("max_slope");
    save_path.append(std::to_string(max_slope));
    save_path.append(".pcd");
    save_cloud_XYZL(labeledCloud, save_path);
    std::cout << "Save the cloud into " << save_path << std::endl;
    
    
    return 0;
}


int dem_multiple_seeds(std::string cloud_path, std::string save_path, const float& radius_search, const float& max_slope, const int& n_seed)
{
    std::cout << "1. Loading input cloud ... \n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
    cloud = load_cloud_XYZL(cloud_path);
    std::cout << "Successfully loaded the input cloud from " << cloud_path << std::endl;
    
    
    std::cout << "2. Building the kdtree ... \n";
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);
    std::cout << "Kdtree successfully built." << std::endl;

    
    // 0 stands for non-ground objects, 1 for ground objects
    std::vector<int> decision;
    for(int i=0; i<cloud->points.size(); i++)
        decision.push_back(0);
  
    for(int k=1; k<=n_seed; ++k)
    {
        std::vector <float> erosion(cloud->points.size());
        std::cout << "3. (" << k << " out of " <<n_seed << "). Calculating erosion values with parameters radius_search = "<< radius_search << " and max_slope = " << max_slope << " ..." << std::endl;
  
    std::vector<int> cloudPointsIndexes;
    for(size_t i=0; i<cloud->points.size(); ++i)
        cloudPointsIndexes.push_back(i);
    
    std::random_shuffle(cloudPointsIndexes.begin(), cloudPointsIndexes.end());
    
    int numberPointsProcessed(0);
    for(std::vector<int>::iterator it=cloudPointsIndexes.begin(); it!=cloudPointsIndexes.end(); ++it)
    { 
        if(numberPointsProcessed%10000==0)
            std::cout << std::setprecision(5) << 100*(numberPointsProcessed)/cloud->points.size() << " percent of points processed." << std::endl;

        std::vector<float> maxCoords = calculateMaxCoords(cloud);
        float erosionValue = maxCoords[2];
        std::vector <int> indexRadiusSearch;
        std::vector <float> distancesRadiusSearch;
        if(kdtree.radiusSearch(cloud->points[*it], radius_search, indexRadiusSearch, distancesRadiusSearch)>0)
        {
            float tempErosionValue = maxCoords[2];
            for(size_t j=1; j<indexRadiusSearch.size(); ++j)
            {
                tempErosionValue = cloud->points[indexRadiusSearch[j]].z + 
                                   max_slope*sqrt(pow(cloud->points[*it].x - cloud->points[indexRadiusSearch[j]].x,2)+
                                                     pow(cloud->points[*it].y - cloud->points[indexRadiusSearch[j]].y,2)); 
                if(tempErosionValue < erosionValue)
                    erosionValue = tempErosionValue; 
            }
        }
        erosion[*it] = erosionValue;
        numberPointsProcessed++;
    }
    std::cout << "End of erosion calculation." <<std::endl;
    
    for(std::vector<int>::iterator it=cloudPointsIndexes.begin(); it!=cloudPointsIndexes.end(); ++it)
        if(cloud->points[*it].z < erosion[*it])
            decision[*it]+=1;
    }
    
    
    std::cout << "4. Labeling ground points (label = 2) and non ground points (label = 27) ...\n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud (new pcl::PointCloud<pcl::PointXYZL>);
    labeledCloud = load_cloud_XYZL(cloud_path);  
    for(size_t i=0; i<labeledCloud->points.size(); ++i)
        labeledCloud->points[i].label = ((decision[i]>=(n_seed/2)) ? GROUND_LABEL : NON_GROUND_LABEL);
    std::cout << "Successfully labeled " << labeledCloud->points.size() << " points." << std::endl;
    
    
    std::cout << "5. Saving labeled cloud ...\n";
    save_path.append("radius");
    save_path.append(std::to_string(radius_search));
    save_path.append("max_slope");
    save_path.append(std::to_string(max_slope));
    save_path.append("num_seeds");
    save_path.append(std::to_string(n_seed));
    save_path.append(".pcd");
    save_cloud_XYZL(labeledCloud, save_path);
    std::cout << "Save the cloud into " << save_path << std::endl;
    
    
    return 0;
}


int main (int argc, char** argv){
  
  if(argc==1)
  {
    show_help(argv[0]);
    return 0;
  }
  
  char opt;
  while((opt=getopt(argc,argv, "ho:m:"))!=-1)
  {
    switch(opt)
    {
        case 'h':
            show_help(argv[0]);
            break;
            
        case 'o':
            {   
                std::string cloud_path = argv[2];
                std::string save_path = argv[3];
                const float radius_search = atof(argv[4]);
                const float max_slope = atof(argv[5]);
                dem_one_seed(cloud_path, save_path, radius_search ,max_slope);
                break;
            }
            
            
        case 'm':
            {
                std::string cloud_path = argv[2];
                std::string save_path = argv[3];
                const float radius_search = atof(argv[4]);
                const float max_slope = atof(argv[5]);
                const int n_seed = atoi(argv[6]);
                dem_multiple_seeds(cloud_path, save_path, radius_search ,max_slope, n_seed);
                break;
            }
            
        default :
            show_help(argv[0]);
            break;
    }
  }
  
  return 0;
}
