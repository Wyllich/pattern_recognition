#include "../math.h"
#include <string>
#include <vector>
#include <assert.h>
#include <pcl/io/pcd_io.h>
#include "../include/math.h"
#include "../../constant/include/constant.h"

//////////////////////////////////////////////////////////////////////////////////////////////

std::string norm_to_string(Norm& norm)
{
    switch(norm)
    {
        case norm1:
            return "norm1";
            break;
        case norm2:
            return "norm2";
            break;
        case norm2z:
            return "norm2z";
            break;
        default:
            return "undefinednorm";
            break;
    }
}

////////////////////////////////////////////////////////////////////

float calculateDistance(Norm norm, std::vector<float>& point1, std::vector<float>& point2)
{
    assert(point1.size() == DIM_VECT && point2.size() == DIM_VECT && "Incorrect vector size.");
    
    float distance(0);
    
    switch(norm)
    {
        case norm1:
        {
            distance = abs(point1[0]-point2[0])+abs(point1[1]-point2[1])+abs(point1[2]-point2[2]);
            break;
        }
        case norm2:
        {
            distance = sqrt(pow(point1[0]-point2[0],2)+pow(point1[1]-point2[1],2)+pow(point1[2]-point2[2],2));
            break;
        }
        case norm2z:
        {
            distance = sqrt(pow(point1[0]-point2[0],2)+pow(point1[1]-point2[1],2)+pow(point1[2]-point2[2],2));
            break;
        }
        default:
            std::cout << "Non-specified norm. Euclidean norm chosen by default" << std::endl;
            distance = sqrt(pow(point1[0]-point2[0],2)+pow(point1[1]-point2[1],2)+pow(point1[2]-point2[2],2));
            break;
    }
    
    assert(distance >=0 && "Non positive distance");
    return distance;
}

////////////////////////////////////////////////////////////////////

float calculateDistance(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2)
{
    // Correct Dimensions
    assert( coordinatesPoint1.size()==DIM_VECT && coordinatesPoint2.size()==DIM_VECT && "1 of the 2 vectors");
    
    float dist(0.0f);
    dist = sqrt( pow(coordinatesPoint2[0]-coordinatesPoint1[0],2)+
                 pow(coordinatesPoint2[1]-coordinatesPoint1[1],2) );
                 
    // NaN value
    assert(dist==dist && "The distance calculated is not a number");
    
    return dist;
}

//////////////////////////////////////////////////////////////////////////////////////////////

float calculateSlope(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2)
{
    // Correct Dimensions
    assert( coordinatesPoint1.size()==DIM_VECT && coordinatesPoint2.size()==DIM_VECT && "1 of the 2 vectors does not have the correct dimension"); 
    // 2 Different points
    assert( coordinatesPoint1 != coordinatesPoint2 && "Calculation of slope for 2 identical points");
    // Infinite slope for 2 points one under another
    assert( (coordinatesPoint1[0]!=coordinatesPoint2[0]) ||
            (coordinatesPoint1[1]!=coordinatesPoint2[1]) || 
            (coordinatesPoint1[0]!=coordinatesPoint2[0]) ||
                "Calculation of an infinite slope" );
                
    float slope(0.0f);
    slope = (coordinatesPoint2[2]-coordinatesPoint1[2]) / calculateDistance(coordinatesPoint1, coordinatesPoint2);

    // NaN value
    assert(slope==slope && "The slope calculated is not a number");

    return slope;
}

//////////////////////////////////////////////////////////////////////////////////////////////

std::vector<float> calculateMaxCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud)
{
    std::vector<float> maxCoords = {cloud->points[0].x, cloud->points[0].y, cloud->points[0].z};
    

    
    for(size_t i=0; i<cloud->points.size(); i++)
    {
        maxCoords[0] = ((cloud->points[i].x>maxCoords[0]) ? cloud->points[i].x : maxCoords[0]),
        maxCoords[1] = ((cloud->points[i].y>maxCoords[1]) ? cloud->points[i].y : maxCoords[1]),
        maxCoords[2] = ((cloud->points[i].z>maxCoords[2]) ? cloud->points[i].z : maxCoords[2]);
    }
    
    assert(maxCoords.size() == DIM_VECT && "The vector is not of dimension 3");
    
    return maxCoords;
}

//////////////////////////////////////////////////////////////////////////////////////////////

std::vector<float> calculateMinCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud)
{
    std::vector<float> minCoords = {cloud->points[0].x, cloud->points[0].y, cloud->points[0].z};
    
    for(size_t i=0; i<cloud->points.size(); i++)
    {
        minCoords[0] = ((cloud->points[i].x<minCoords[0]) ? cloud->points[i].x : minCoords[0]),
        minCoords[1] = ((cloud->points[i].y<minCoords[1]) ? cloud->points[i].y : minCoords[1]),
        minCoords[2] = ((cloud->points[i].z<minCoords[2]) ? cloud->points[i].z : minCoords[2]);
    }
    
    assert(minCoords.size() == DIM_VECT && "The vector is not of dimension 3");
    
    return minCoords;
}

//////////////////////////////////////////////////////////////////////////////////////////////

float calculateApproxDensity(int cloudsize, const std::vector<float>& maxCoords, const std::vector<float>& minCoords)
{
    float approxDensity = (float) cloudsize;
    for(size_t i=0; i<DIM_VECT; ++i)
        approxDensity /= (maxCoords[i]-minCoords[i]);
    
    assert(approxDensity >=0 && "The calculated approximate density is a positive real number.");
    
    return approxDensity;
}
