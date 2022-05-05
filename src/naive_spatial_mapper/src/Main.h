#include "SpatialMapper.h"
#include "Topics.h"

#include "naive_spatial_mapper/get_spatial_map.h"
#include <pcl_conversions/pcl_conversions.h>

SpatialMapper* spatialMapper;

int main(int argc, char **argv);

void pointCloudFrameCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

void clearSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg);
void saveSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg);
void smoothenSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg);
void findPlanesCallback(const std_msgs::Bool::ConstPtr& msg);

bool getSpatialMapCallback(
    naive_spatial_mapper::get_spatial_map::Request &req,
    naive_spatial_mapper::get_spatial_map::Response &res);