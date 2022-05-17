#include "Topics.h"
#include "SpatialMapper.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "hololens_depth_data_receiver_msgs/PointCloudFrame.h"

SpatialMapper* spatialMapper;

int main(int argc, char **argv);

void pointCloudFrameCallback(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg);