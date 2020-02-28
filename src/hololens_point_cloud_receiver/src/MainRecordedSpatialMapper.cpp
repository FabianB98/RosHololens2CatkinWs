#include "MainRecordedSpatialMapper.h"

int main(int argc, char **argv)
{
    // Initialize everything.
    ros::init(argc, argv, "recorded_spatial_mapper");

    ros::NodeHandle n;

    ros::Subscriber loadRecordingSubscriber = n.subscribe(LOAD_RECORDING_TOPIC, 10, loadRecordingCallback);
    ros::Subscriber clearPointCloudSubscriber = n.subscribe(CLEAR_POINT_CLOUD_TOPIC, 10, clearPointCloudCallback);
    ros::Subscriber savePointCloudSubscriber = n.subscribe(SAVE_POINT_CLOUD_TOPIC, 10, savePointCloudCallback);
    ros::Subscriber smoothenPointCloudSubscriber = n.subscribe(SMOOTHEN_POINT_CLOUD_TOPIC, 10, smoothenPointCloudCallback);
    ros::Subscriber findPlanesSubscriber = n.subscribe(FIND_PLANES_TOPIC, 10, findPlanesCallback);
    ros::ServiceServer getSpatialMapService = n.advertiseService(GET_SPATIAL_MAP_SERVICE, getSpatialMapCallback);

    spatialMapper = new SpatialMapper(n);
    SpatialMapper* mappers[] = {spatialMapper};
    depthFrameReader = new DepthFrameReader(mappers, 1);

    // Perform the update loop using a multi threaded spinner with an amount of threads equal to the core count of the CPU.
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Clean up.
    delete depthFrameReader;
    delete spatialMapper;

    return 0;
}

void loadRecordingCallback(const std_msgs::String::ConstPtr& msg)
{
    depthFrameReader->processRecording(msg);
}

void clearPointCloudCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->clearPointCloud();
}

void savePointCloudCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->savePointCloud();
}

void smoothenPointCloudCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->smoothenPointCloud();
}

void findPlanesCallback(const std_msgs::Bool::ConstPtr& msg)
{
    spatialMapper->detectPlanes();
}

bool getSpatialMapCallback(
    hololens_point_cloud_receiver::get_spatial_map::Request &req,
    hololens_point_cloud_receiver::get_spatial_map::Response &res)
{
    sensor_msgs::PointCloud2 spatialMap;
    pcl::toROSMsg(*spatialMapper->getSpatialMap(req.doPostProcessing), spatialMap);
    spatialMap.header.seq = 1;
    spatialMap.header.stamp = ros::Time::now();
    spatialMap.header.frame_id = "map";
    res.spatialMap = spatialMap;
    return true;
}