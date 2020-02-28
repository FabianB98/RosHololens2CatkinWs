#include "SpatialMapper.h"
#include "Topics.h"

#include <hololens_point_cloud_receiver/get_spatial_map.h>
#include <pcl_conversions/pcl_conversions.h>

SpatialMapper* spatialMapper;

int main(int argc, char **argv);

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

void clearPointCloudCallback(const std_msgs::Bool::ConstPtr& msg);
void savePointCloudCallback(const std_msgs::Bool::ConstPtr& msg);
void smoothenPointCloudCallback(const std_msgs::Bool::ConstPtr& msg);
void findPlanesCallback(const std_msgs::Bool::ConstPtr& msg);

bool getSpatialMapCallback(
    hololens_point_cloud_receiver::get_spatial_map::Request &req,
    hololens_point_cloud_receiver::get_spatial_map::Response &res);