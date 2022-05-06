#include "Topics.h"
#include "SpatialMapper.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

SpatialMapper* spatialMapper;

int main(int argc, char **argv);

void pointCloudFrameCallback(const sensor_msgs::PointCloud2ConstPtr& msg);