#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "octomap_based_spatial_mapper");

    ros::NodeHandle n;

    ros::Subscriber shortThrowPointCloudSubscriber = n.subscribe(SHORT_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10, pointCloudFrameCallback);
    ros::Subscriber longThrowPointCloudSubscriber = n.subscribe(LONG_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10, pointCloudFrameCallback);

    spatialMapper = new SpatialMapper(n);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete spatialMapper;
}

void pointCloudFrameCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    spatialMapper->handlePointCloud(msg);
}