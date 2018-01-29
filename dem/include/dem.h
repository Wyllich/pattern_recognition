#ifndef DEM_H
#define DEM_H

//////DEM FUNCTIONS

int show_help(const char *s);

int blind_dem(std::string path, const float& radius_search, const float& max_slope);

int dem_one(std::string path);

int dem_multiple(std::string path, const int& n_seed);

#endif
