#include "MainRecorder.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "depth_frame_recorder");

    ros::NodeHandle n;

    ros::Subscriber shortThrowDepthSubscriber = n.subscribe(SHORT_THROW_DEPTH_TOPIC, 10, shortThrowDepthFrameCallback);
    ros::Subscriber longThrowDepthSubscriber = n.subscribe(LONG_THROW_DEPTH_TOPIC, 10, longThrowDepthFrameCallback);
    ros::Subscriber shortThrowDirectionsSubscriber = n.subscribe(SHORT_THROW_PIXEL_DIRECTIONS_TOPIC, 10, shortThrowPixelDirectionsCallback);
    ros::Subscriber longThrowDirectionsSubscriber = n.subscribe(LONG_THROW_PIXEL_DIRECTIONS_TOPIC, 10, longThrowPixelDirectionsCallback);
    ros::Subscriber recordSubscriber = n.subscribe(CLEAR_POINT_CLOUD_TOPIC, 10, recordCallback);

    depthFrameRecorder = new DepthFrameRecorder();

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete depthFrameRecorder;

    return 0;
}

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    depthFrameRecorder->handleShortThrowDepthFrame(msg);
}

void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    depthFrameRecorder->handleLongThrowDepthFrame(msg);
}

void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    depthFrameRecorder->handleShortThrowPixelDirections(msg);
}

void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    depthFrameRecorder->handleLongThrowPixelDirections(msg);
}

void recordCallback(const std_msgs::Bool::ConstPtr& msg)
{
    depthFrameRecorder->handleRecord(msg);
}