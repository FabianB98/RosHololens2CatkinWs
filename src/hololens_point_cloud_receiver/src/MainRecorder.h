#include "DepthFrameRecorder.h"
#include "Topics.h"

DepthFrameRecorder* depthFrameRecorder;

int main(int argc, char **argv);

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

void recordCallback(const std_msgs::Bool::ConstPtr& msg);