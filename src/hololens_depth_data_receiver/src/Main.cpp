#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "depth_data_receiver");

    ros::NodeHandle n;

    ros::Subscriber shortThrowDepthSubscriber = n.subscribe(SHORT_THROW_DEPTH_TOPIC, 10, shortThrowDepthFrameCallback);
    ros::Subscriber longThrowDepthSubscriber = n.subscribe(LONG_THROW_DEPTH_TOPIC, 10, longThrowDepthFrameCallback);
    ros::Subscriber shortThrowDirectionsSubscriber = n.subscribe(SHORT_THROW_PIXEL_DIRECTIONS_TOPIC, 10, shortThrowPixelDirectionsCallback);
    ros::Subscriber longThrowDirectionsSubscriber = n.subscribe(LONG_THROW_PIXEL_DIRECTIONS_TOPIC, 10, longThrowPixelDirectionsCallback);

    depthDataReceiver = initializeDepthDataReceiver(n);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete depthDataReceiver;

    return 0;
}

void shortThrowDepthFrameCallback(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    depthDataReceiver->handleShortThrowDepthFrame(msg);
}

void longThrowDepthFrameCallback(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    depthDataReceiver->handleLongThrowDepthFrame(msg);
}

void shortThrowPixelDirectionsCallback(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    depthDataReceiver->handleShortThrowPixelDirections(msg);
}

void longThrowPixelDirectionsCallback(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    depthDataReceiver->handleLongThrowPixelDirections(msg);
}
