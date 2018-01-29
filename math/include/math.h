#ifndef MATH_H
#define MATH_H

float calculateDistance(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2);

float calculateSlope(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2);

std::vector<float> calculateMaxCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

std::vector<float> calculateMinCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

float calculateApproxDensity(int cloudsize, const std::vector<float>& maxCoords, const std::vector<float>& minCoords);

float minimumDistance(pcl::)
#endif
