#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "octomap_based_spatial_mapper");

    ros::NodeHandle n;

    ros::Subscriber shortThrowPointCloudSubscriber = n.subscribe(SHORT_THROW_POINT_CLOUD_FRAME_TOPIC, 10, pointCloudFrameCallback);
    ros::Subscriber longThrowPointCloudSubscriber = n.subscribe(LONG_THROW_POINT_CLOUD_FRAME_TOPIC, 10, pointCloudFrameCallback);
    ros::Subscriber enableSpatialMapUpdatesSubscriber = n.subscribe(ENABLE_SPATIAL_MAP_UPDATES_TOPIC, 10, enableSpatialMapUpdatesCallback);
    ros::Subscriber clearSpatialMapSubscriber = n.subscribe(CLEAR_SPATIAL_MAP_TOPIC, 10, clearSpatialMapCallback);

    spatialMapper = new SpatialMapper(n);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete spatialMapper;
}

void pointCloudFrameCallback(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg)
{
    spatialMapper->handlePointCloudFrame(msg);
}

void enableSpatialMapUpdatesCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->setUpdateSpatialMap(msg->data);
}

void clearSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->clearSpatialMap();
}