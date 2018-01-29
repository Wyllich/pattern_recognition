#include <pcl/kdtree/kdtree_flann.h> //kdtree
#include "../../cloud_io/include/cloud_io.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h> //getopt
#include <vector> //vector
#include <iomanip> //setting precision of
#include <string>
#include <fstream>
#include "../dem.h"


//////////////////////////////////////////////////////////////////////////////////////////////

int const DIM_VECT(3);
int const GROUND_LABEL(2), NON_GROUND_LABEL(27);
int const NUM_DESIRED_POINTS(100);

//////////////////////////////////////////////////////////////////////////////////////////////

int showHelpInfo(const char *s)
{
    std::cout << "Usage :  " << s << " [-option] [argument]" <<std::endl;
    std::cout << "Option : " << "-h : show help information" << std::endl
                             << "-e : Extract some features of the image (extremal coordinates, point density). \n Format : cloudfile.pcd" << std::endl
                             << "-b : Blind modelisation of groud using one seed point. \n Format : cloudfile.pcd radiusSearch maximumSlope" << std::endl
                             << "-d : Deep modelisation of ground using some image features and one seed point. \n Format : cloudfile.pcd" <<std::endl
                             << "-m : Deep modelisation of ground using some image features and multiples seed points. \n Format : cloudfile.pcd numberSeedPoints" <<std::endl
                             << "-p : Writes into a .txt file the mean slope of every cloud point. \n Format : cloudfile.pcd" << std::endl;
    
    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////

int blindGroundModelisation(const char* cloudfile, const float& radiusSearch, const float& maximumSlope)
{
  //Loading Data from a PCD File
  std::cout << "1. Loading input cloud...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
  cloud = loadCloud(cloudfile);
  std::cout << "Successfully loaded the input cloud." <<std::endl;
   
  //Calculating the erosion value of every point cloud. 
  //  i) The values are compared with those obtained with points within a set distance (radiusSearch) of the observed point. 
  //  ii) The maximumSlope is set by default to 1.
  std::cout << "2. Building the kdtree...\n";
  pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
  kdtree.setInputCloud(cloud);
  std::cout << "Kdtree successfully built." << std::endl;
  
  std::vector <float> erosion(cloud->points.size());
  std::cout << "3. Calculating erosion values with parameters radiusSearch = "<< radiusSearch << " and maximumSlope = " << maximumSlope << " ..." << std::endl; 
  for(size_t i=0; i<cloud->points.size(); ++i)
  {
    if(i%1000==0)
        std::cout << 100*i/cloud->points.size() << " percent completion.\n";
    float erosionValue(10000.0f);
    std::vector <int> indexRadiusSearch;
    std::vector <float> distancesRadiusSearch;
    if(kdtree.radiusSearch(cloud->points[i], radiusSearch, indexRadiusSearch, distancesRadiusSearch)>0)
    {
        float tempErosionValue = 10000.0f;
        for(size_t j=1; j<indexRadiusSearch.size(); ++j)
        {
            tempErosionValue = cloud->points[ indexRadiusSearch[j] ].z + 
                               maximumSlope*sqrt( pow(cloud->points[i].x - cloud->points[indexRadiusSearch[j]].x,2)+
                                                  pow(cloud->points[i].y - cloud->points[indexRadiusSearch[j]].y,2) ); 
            if(tempErosionValue < erosionValue)
                erosionValue = tempErosionValue; 
        }
    }
    erosion[i] = erosionValue;
  }
  std::cout << "End of erosion calculation." <<std::endl;
  
  //Discriminate ground points from non ground points.
  std::cout << "4. Labeling ground points (label = 2) and non ground points (label = 27) ...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud (new pcl::PointCloud<pcl::PointXYZL>);
  labeledCloud = loadCloud(cloudfile);
  
  for(size_t i=0; i<labeledCloud->points.size(); ++i)
    labeledCloud->points[i].label = ((labeledCloud->points[i].z < erosion[i]) ? GROUND_LABEL : NON_GROUND_LABEL);
  
  //The output file contains a PCD file with the colorised cloud.
  std::stringstream savePath;
  savePath << "/home/wyllich/data/results/dem/blindDemRad";
  savePath << std::setprecision(3) << radiusSearch;
  savePath << "MaxSlope";
  savePath << std::setprecision(3) << maximumSlope;
  savePath << ".pcd";
  pcl::io::savePCDFileASCII (savePath.str(), *labeledCloud);
  std::cout << "Successfully labeled " << labeledCloud->points.size() << " points." << std::endl;
  
  return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////

int deepGroundModelisationOne(const char* cloudfile)
{
  //Loading Data from a PCD File
  std::cout << "1. Loading input cloud...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
  cloud = loadCloud(cloudfile);
  std::cout << "Successfully loaded the input cloud." <<std::endl;
  
  //Calculating the erosion value of every point cloud. 
  //  i) The values are compared with those obtained with points within a set distance (radiusSearch) of the observed point. 
  //  ii) The maximumSlope is set by default to 1.
  std::cout << "3. Building the kdtree...\n";
  pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
  kdtree.setInputCloud(cloud);
  std::cout << "Kdtree successfully built." << std::endl;
  
  std::vector <float> erosion(cloud->points.size());
  std::cout << "4. Calculating erosion values with parameters radiusSearch = "<< radiusSearch << " and maximumSlope = " << maximumSlope << " ..." << std::endl; 
  for(size_t i=0; i<cloud->points.size(); ++i)
  { 
    if(i%10000==0)
        std::cout << std::setprecision(5) << 100*i/cloud->points.size() << " percent of points processed." << std::endl;

    float erosionValue = maxCoords[2];
    std::vector <int> indexRadiusSearch;
    std::vector <float> distancesRadiusSearch;
    if(kdtree.radiusSearch(cloud->points[i], radiusSearch, indexRadiusSearch, distancesRadiusSearch)>0)
    {
        float tempErosionValue = maxCoords[2];
        for(size_t j=1; j<indexRadiusSearch.size(); ++j)
        {
            tempErosionValue = cloud->points[indexRadiusSearch[j]].z + 
                               maximumSlope*sqrt(pow(cloud->points[i].x - cloud->points[indexRadiusSearch[j]].x,2)+
                                                 pow(cloud->points[i].y - cloud->points[indexRadiusSearch[j]].y,2)); 
            if(tempErosionValue < erosionValue)
                    erosionValue = tempErosionValue; 
        }
    }
    erosion[i] = erosionValue;
  }
  std::cout << "End of erosion calculation." <<std::endl;
  
  //Discriminate ground points from non ground points.
  std::cout << "5. Labeling ground points (label = 2) and non ground points (label = 27) ...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud (new pcl::PointCloud<pcl::PointXYZL>);
  labeledCloud = loadCloud(cloudfile);
  
  for(size_t i=0; i<labeledCloud->points.size(); ++i)
    labeledCloud->points[i].label = ((labeledCloud->points[i].z < erosion[i]) ? GROUND_LABEL : NON_GROUND_LABEL);
  
  //The output file contains a PCD file with the colorised cloud.
  std::stringstream savePath;
  savePath << "/home/wyllich/data/results/dem/deepDem.pcd";
  pcl::io::savePCDFileASCII (savePath.str(), *labeledCloud);
  std::cout << "Successfully labeled " << labeledCloud->points.size() << " points." << std::endl;
  
  return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////

int deepGroundModelisationMultiple(char* cloudfile, const int& numberSeeds)
{
  //Loading Data from a PCD File
  std::cout << "1. Loading input cloud...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
  cloud = loadCloud(cloudfile);
  std::cout << "Successfully loaded the input cloud." <<std::endl;


  std::cout << "2. Extracting different features...\n";
  //Extract maximum and minimum coordinates
  std::vector<float> maxCoords = calculateMaxCoords(cloud);
  std::vector<float> minCoords = calculateMinCoords(cloud);  
  
  std::cout << "The maximum coordinates are : ";
  for(std::vector<float>::iterator it = maxCoords.begin(); it < maxCoords.end(); ++it)
    std::cout << maxCoords[*it] << " ";
  std::cout << std::endl;
  
  std::cout << "The minimum coordinates are : ";
  for(std::vector<float>::iterator it = minCoords.begin(); it < minCoords.end(); ++it)
    std::cout << minCoords[*it] << " ";
  std::cout << std::endl;
  
  //Extract approximate density
  float approxDensity = calculateApproxDensity(cloud->points.size(), maxCoords, minCoords);
  std::cout << "The approximate density is : " << approxDensity << "points per m^2. \n";
  std::cout << "End of extracting features." << std::endl;
  
  float radiusSearch = cbrt(NUM_DESIRED_POINTS/approxDensity);
  float maximumSlope = 1.0f;
  
  //Calculated structure to calculate nearest neighbour
  std::cout << "3. Building the kdtree...\n";
  pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
  kdtree.setInputCloud(cloud);
  std::cout << "Kdtree successfully built." << std::endl;
  
  //Dem multiples times
  // 0 stands for non-ground objects, 1 for ground objects
  std::vector<int> decision;
  for(int i=0; i<cloud->points.size(); i++)
    decision.push_back(0);
  
  assert(numberSeeds >= 1 && "The number of seeds is incorrect");
  
  for(int k=1; k<=numberSeeds; ++k)
  {
    std::vector <float> erosion(cloud->points.size());
    std::cout << "4. (" << k << " out of " <<numberSeeds << "). Calculating erosion values with parameters radiusSearch = "<< radiusSearch << " and maximumSlope = " << maximumSlope << " ..." << std::endl;
  
    std::vector<int> cloudPointsIndexes;
    for(size_t i=0; i<cloud->points.size(); ++i)
        cloudPointsIndexes.push_back(i);
    
    std::random_shuffle(cloudPointsIndexes.begin(), cloudPointsIndexes.end());
    
    int numberPointsProcessed(0);
    for(std::vector<int>::iterator it=cloudPointsIndexes.begin(); it!=cloudPointsIndexes.end(); ++it)
    { 
        if(numberPointsProcessed%10000==0)
            std::cout << std::setprecision(5) << 100*(numberPointsProcessed)/cloud->points.size() << " percent of points processed." << std::endl;

        float erosionValue = maxCoords[2];
        std::vector <int> indexRadiusSearch;
        std::vector <float> distancesRadiusSearch;
        if(kdtree.radiusSearch(cloud->points[*it], radiusSearch, indexRadiusSearch, distancesRadiusSearch)>0)
        {
            float tempErosionValue = maxCoords[2];
            for(size_t j=1; j<indexRadiusSearch.size(); ++j)
            {
                tempErosionValue = cloud->points[indexRadiusSearch[j]].z + 
                                   maximumSlope*sqrt(pow(cloud->points[*it].x - cloud->points[indexRadiusSearch[j]].x,2)+
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
  
  //Discriminate ground points from non ground points.
  std::cout << "5. Labeling ground points (label = 2) and non ground points (label = 27) ...\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud (new pcl::PointCloud<pcl::PointXYZL>);
  labeledCloud = loadCloud(cloudfile);
  
  for(size_t i=0; i<labeledCloud->points.size(); ++i)
    labeledCloud->points[i].label = ((decision[i]>=(numberSeeds/2)) ? GROUND_LABEL : NON_GROUND_LABEL);
  
  //The output file contains a PCD file with the colorised cloud.
  std::stringstream savePath;
  savePath << "/home/wyllich/data/results/dem/deepDem";
  savePath << numberSeeds;
  savePath << "seeds.pcd";
  pcl::io::savePCDFileASCII (savePath.str(), *labeledCloud);
  std::cout << "Successfully labeled " << labeledCloud->points.size() << " points." << std::endl;
  
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////

int meanSlope(char * cloudfile)
{
    //Loading Data from a PCD File
    std::cout << "1. Loading input cloud...\n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);
    cloud = loadCloud(cloudfile);
    std::cout << "Successfully loaded the input cloud." <<std::endl;
    
    //Calculating the slopes for each of the points
    std::cout << "2. Calculating the mean slope for every point...\n";
    std::vector<float> slopes;
    slopes.reserve(cloud->points.size());
    for(size_t i=0; i<cloud->points.size(); ++i)
    {
        if(i%1000==0)
            std::cout << 100*i/cloud->points.size() << " percent of points completed.\n";
        std::vector<float> currentPoint = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        std::vector<float> tempSlopes;
        std::vector<float> tempDistances;
        tempSlopes.reserve(cloud->points.size());
        tempDistances.reserve(cloud->points.size());
        for(size_t j=0; j<cloud->points.size(); ++j)
        {
            std::vector<float> comparedPoint = {cloud->points[j].x, cloud->points[j].y, cloud->points[j].z};
            if(j!=i)
            {
                tempSlopes.push_back(calculateSlope(currentPoint, comparedPoint));
                tempDistances.push_back(calculateDistance(currentPoint, comparedPoint));
            }
        }
        
        float num(0), denom(0);
        for(size_t k=0; k<tempSlopes.size(); k++)
        {
            num += tempSlopes[k]*tempDistances[k];
            denom += tempDistances[k];
        }
        
        slopes.push_back(num/denom);
        
    }
    
    std::cout << "Calculation of the slopes completed" << std::endl;
    
    std::ofstream file("/home/wyllich/data/results/dem/slopes.txt", std::ios::out|std::ios::app);
    
    if(file)
    {
        for(int i=0; i<cloud->points.size(); ++i)
            file << cloud->points[i].x << ' ' << cloud->points[i].y << ' ' << slopes[i] << '\n';
        file.close();
    }
    
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv){
  
  srand(time(NULL));
  
  if(argc==1)
  {
    showHelpInfo(argv[0]);
    return 0;
  }
  
  char opt;
  while((opt=getopt(argc,argv, "heb:d:m:p:"))!=-1)
  {
    switch(opt)
    {
        case 'h':
            showHelpInfo(argv[0]);
            break;
            
        case 'e':
            extractFeatures(argv[2]);
            break;
            
        case 'b':
            {   
                char* cloudfile = argv[2];
                const float radiusSearch = atof(argv[3]);
                const float maximumSlope = atof(argv[4]);
                blindGroundModelisation(cloudfile,radiusSearch,maximumSlope);
                break;
            }
            
        case 'd':
            {
                char* cloudfile = argv[2];
                deepGroundModelisationOne(cloudfile);
                break;
            }
            
        case 'm':
            {
                char* cloudfile = argv[2];
                const int numberSeed = atoi(argv[3]);
                deepGroundModelisationMultiple(cloudfile, numberSeed);
                break;
            }
            
        case 'p':
            {
                char* cloudfile = argv[2];
                meanSlope(cloudfile);
                break;
            }
            
        default :
            showHelpInfo(argv[0]);
            break;
    }
  }
  
  return 0;
}

