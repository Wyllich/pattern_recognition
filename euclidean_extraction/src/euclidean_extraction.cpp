#include <pcl/kdtree/kdtree_flann.h> //kdtree
#include <iostream>
#include <vector> //vector
#include <stdlib.h>
#include <unistd.h>
#include "../../cloud_io/include/cloud_io.h"
#include "../include/euclidean_extraction.h"


using namespace cloud_io;

//////////////////////////////////////////////////////////////////////////////////////////////

int show_help(char *s)
{
    std::cout << "Usage :  " << s << " [-option] [argument]" <<std::endl;
    std::cout << "Option : " << "-h : show help information\n"
                             << "-o : norm 1 extraction. Privileges cubes. \n Format : input.pcd output.pcd search_rad\n"
                             << "-e : euclidean extraction. \n Format : input.pcd output.pcd search_rad\n"
                             << "-z : euclidean extraction with z priviledged in clustering. \n Format : input.pcd output.pcd search_rad" << std::endl;

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////

int extraction(std::string cloud_path, std::string save_path, const float& max_rad, Norm norm)
{
  //Loading Data from a PCD File
  std::cout << "1. Loading input cloud...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
  cloud = load_cloud_XYZL(cloud_path);
  std::cout << "Successfully loaded " << cloud->points.size() << " points from the input cloud." <<std::endl;
  
  //Label settings
  int indexNextCluster = 1;
  int indexQueue = -1;
  int indexNeutral = 0;
  
  //Queue settings
  std::vector<int> queueIndexes;
  queueIndexes.reserve(cloud->points.size());
  
  //Resetting the labels of every non ground points
  std::cout << "2. Resetting labels..." << std::endl;
  for(size_t i=0; i<cloud->points.size(); ++i)
    cloud->points[i].label = indexNeutral;
  std::cout << "Data cleared." << std::endl;
  
  //Creating the kdtree
  std::cout << "3. Creating the kdtree..." << std::endl;
  pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
  kdtree.setInputCloud(cloud);
  std::cout << "Kdtree successfully built." << std::endl;
      
  //Main Loop
  std::cout << "4. Clustering..." <<std::endl;
  for(size_t i=0; i < cloud->points.size(); ++i)
  {
    if(i%10000 == 0)
        std::cout << 100*i/cloud->points.size() << " percent completion." << std::endl;
        
    if(std::find(queueIndexes.begin(), queueIndexes.end(), i) != queueIndexes.end())
        queueIndexes.push_back(i);
        
    for(std::vector<int>::iterator it=queueIndexes.begin(); it<queueIndexes.end(); ++it)
    {
        std::vector<float> currentPoint = {cloud->points[*it].x,
                                           cloud->points[*it].y,
                                           cloud->points[*it].z};
                                       
        std::vector<int> indexRadiusSearch;
        std::vector<float> distancesRadiusSearch;
        if(kdtree.radiusSearch(cloud->points[*it], max_rad, indexRadiusSearch, distancesRadiusSearch))
        {
            for(size_t j=0; j < indexRadiusSearch.size(); ++j)
            {
                std::vector<float> comparedPoint = {cloud->points[ indexRadiusSearch[j] ].x,
                                                    cloud->points[ indexRadiusSearch[j] ].y,
                                                    cloud->points[ indexRadiusSearch[j] ].z};
                if((cloud->points[  indexRadiusSearch[j] ].label == indexNeutral) && (calculateDistance(norm, currentPoint, comparedPoint) <= max_rad) )
                {
                    cloud->points[ indexRadiusSearch[j] ].label = indexNextCluster;  
                    if(std::find(queueIndexes.begin(), queueIndexes.end(), i) != queueIndexes.end())
                        queueIndexes.push_back( indexRadiusSearch[j] );
                }
            }
        }
     }
     
     bool isProcessed = 1;
     for(std::vector<int>::iterator it=queueIndexes.begin(); it< queueIndexes.end(); ++it)
     {  
        if(cloud->points[*it].label==indexNeutral)
            isProcessed = 0;
     }
     

     if(isProcessed)
     {  
        queueIndexes.erase(queueIndexes.begin(), queueIndexes.end());
        indexNextCluster++;
     }
  }
  std::cout << "End of clustering." << std::endl;
  
  std::cout << "5. Labeling..." << std::endl;
  save_path.append(norm_to_string(norm));
  save_path.append("rad");
  save_path.append(std::to_string(max_rad));
  save_path.append(".pcd");
  save_cloud_XYZL(cloud, save_path);
  std::cout << "Successfully labeled " << cloud->points.size() << " points." << std::endl;
  
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  
  if(argc==1)
  {
    show_help(argv[0]);
    return 0;
  }
  
  char opt;
  while((opt=getopt(argc,argv, "he:z:o:"))!=-1)
  {
    switch(opt)
    {
        case 'h':
            show_help(argv[0]);
            break;
        case 'o':
            {
                float max_rad = atof(argv[4]);
                extraction(argv[2], argv[3], max_rad, norm1);
                break;
            }
        case 'e':
            {
                float max_rad = atof(argv[4]);
                extraction(argv[2], argv[3], max_rad, norm2);
                break;
            }
        case 'z':
            {
                float max_rad = atof(argv[4]);
                extraction(argv[2], argv[3], max_rad, norm2z);
                break;
            }
        default :
            show_help(argv[0]);
            break;
    }
  }
  
  
  return(0);
}
