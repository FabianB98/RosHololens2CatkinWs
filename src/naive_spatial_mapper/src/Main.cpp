#include "Main.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "naive_spatial_mapper");

    ros::NodeHandle n;

    ros::Subscriber shortThrowPointCloudSubscriber = n.subscribe(SHORT_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10, pointCloudFrameCallback);
    ros::Subscriber longThrowPointCloudSubscriber = n.subscribe(LONG_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10, pointCloudFrameCallback);
    ros::Subscriber clearSpatialMapSubscriber = n.subscribe(CLEAR_SPATIAL_MAP_TOPIC, 10, clearSpatialMapCallback);
    ros::Subscriber saveSpatialMapSubscriber = n.subscribe(SAVE_SPATIAL_MAP_TOPIC, 10, saveSpatialMapCallback);
    ros::Subscriber smoothenSpatialMapSubscriber = n.subscribe(SMOOTHEN_SPATIAL_MAP_TOPIC, 10, smoothenSpatialMapCallback);
    ros::Subscriber findPlanesSubscriber = n.subscribe(FIND_PLANES_TOPIC, 10, findPlanesCallback);
    ros::ServiceServer getSpatialMapService = n.advertiseService(GET_SPATIAL_MAP_SERVICE, getSpatialMapCallback);

    spatialMapper = new SpatialMapper(n);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete spatialMapper;

    return 0;
}

void pointCloudFrameCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    spatialMapper->handlePointCloud(msg);
}

void clearSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->clearSpatialMap();
}

void saveSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->saveSpatialMap();
}

void smoothenSpatialMapCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->smoothenSpatialMap();
}

void findPlanesCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->detectPlanes();
}

bool getSpatialMapCallback(
    naive_spatial_mapper::get_spatial_map::Request &req,
    naive_spatial_mapper::get_spatial_map::Response &res)
{
    sensor_msgs::PointCloud2 spatialMap;
    pcl::toROSMsg(*spatialMapper->getSpatialMap(req.doPostProcessing), spatialMap);
    spatialMap.header.seq = 1;
    spatialMap.header.stamp = ros::Time::now();
    spatialMap.header.frame_id = "hololens_world";
    res.spatialMap = spatialMap;
    return true;
}
