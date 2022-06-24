#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "stereo_image_receiver");

    ros::NodeHandle n;

    ros::Subscriber stereoImageSubscriber = n.subscribe(STEREO_IMAGE_TOPIC, 10, stereoImageCallback);

    stereoImageReceiver = new StereoImageReceiver(n);

    dynamic_reconfigure::Server<hololens_stereo_image_receiver::StereoImageReceiverConfig> server;
    dynamic_reconfigure::Server<hololens_stereo_image_receiver::StereoImageReceiverConfig>::CallbackType f;
    f = boost::bind(&StereoImageReceiver::handleReconfiguration, stereoImageReceiver, _1, _2);
    server.setCallback(f);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete stereoImageReceiver;

    return 0;
}

void stereoImageCallback(const hololens_msgs::StereoCameraFrame::ConstPtr& msg)
{
    stereoImageReceiver->handleStereoCameraFrame(msg);
}
