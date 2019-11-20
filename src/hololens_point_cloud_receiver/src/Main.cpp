#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "PointCloudListener");

    ros::NodeHandle n;

    ros::Subscriber shortThrowDepthSubscriber = n.subscribe(SHORT_THROW_DEPTH_TOPIC, 10, shortThrowDepthFrameCallback);
    ros::Subscriber longThrowDepthSubscriber = n.subscribe(LONG_THROW_DEPTH_TOPIC, 10, longThrowDepthFrameCallback);
    ros::Subscriber shortThrowDirectionsSubscriber = n.subscribe(SHORT_THROW_PIXEL_DIRECTIONS_TOPIC, 10, shortThrowPixelDirectionsCallback);
    ros::Subscriber longThrowDirectionsSubscriber = n.subscribe(LONG_THROW_PIXEL_DIRECTIONS_TOPIC, 10, longThrowPixelDirectionsCallback);
    ros::Subscriber clearPointCloudSubscriber = n.subscribe(CLEAR_POINT_CLOUD_TOPIC, 10, clearPointCloudCallback);
    ros::Subscriber savePointCloudSubscriber = n.subscribe(SAVE_POINT_CLOUD_TOPIC, 10, savePointCloudCallback);

    pointCloudReceiver = new PointCloudReceiver(n);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete pointCloudReceiver;

    return 0;
}

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    pointCloudReceiver->handleShortThrowDepthFrame(msg);
}

void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    pointCloudReceiver->handleLongThrowDepthFrame(msg);
}

void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    pointCloudReceiver->handleShortThrowPixelDirections(msg);
}

void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    pointCloudReceiver->handleLongThrowPixelDirections(msg);
}

void clearPointCloudCallback(const std_msgs::Bool::ConstPtr& msg)
{
    pointCloudReceiver->clearPointCloud();
}

void savePointCloudCallback(const std_msgs::Bool::ConstPtr& msg)
{
    pointCloudReceiver->savePointCloud();
}