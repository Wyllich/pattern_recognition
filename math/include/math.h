#ifndef MATH_H
#define MATH_H

enum Norm {norm1, norm2, norm2z};

std::string norm_to_string(Norm& norm);

float calculateDistance(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2);

float calculateDistance(Norm norm, std::vector<float>& point1, std::vector<float>& point2);

float calculateSlope(const std::vector<float>& coordinatesPoint1, const std::vector<float>& coordinatesPoint2);

std::vector<float> calculateMaxCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

std::vector<float> calculateMinCoords(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

float calculateApproxDensity(int cloudsize, const std::vector<float>& maxCoords, const std::vector<float>& minCoords);

#endif
