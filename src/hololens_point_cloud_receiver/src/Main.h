#include "PointCloudReceiver.h"

PointCloudReceiver* pointCloudReceiver;

int main(int argc, char **argv);

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);