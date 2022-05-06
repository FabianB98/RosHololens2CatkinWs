#include "SpatialMapper.h"

SpatialMapper::SpatialMapper(ros::NodeHandle n)
{
    ROS_INFO("Creating SpatialMapper...");

    // TODO: The topic for this publisher should be a better suited name and should be defined as a topic constant.
    octomapPublisher = n.advertise<octomap_msgs::Octomap>("octomap", 10);
}

void SpatialMapper::handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    octomap::Pointcloud pointCloudOctomap = octomap::Pointcloud();
    octomap::pointCloud2ToOctomap(*msg, pointCloudOctomap);

    // TODO: Sensor origin needs to be fetched from the hololens_depth_data_receiver package.
    ros::Time pointCloudTimestamp = msg->header.stamp;
    tf::StampedTransform transform;
    try
    {
        tfListener.waitForTransform("hololens_world", "hololens_cam", pointCloudTimestamp, ros::Duration(3.0));
        tfListener.lookupTransform("hololens_world", "hololens_cam", pointCloudTimestamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Couldn't fetch the HoloLens's position!");
        ROS_ERROR("%s", ex.what());
    }
    octomap::point3d sensorOrigin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

    // TODO: leafSize should be a configurable parameter.
    // TODO: maxRange should be fetched from the hololens_depth_data_receiver package.
    double leafSize = 0.1;
    double maxRange = -1.0;

    octomap::OcTree octree(leafSize);
    octree.insertPointCloud(pointCloudOctomap, sensorOrigin, maxRange);
    //octree.updateInnerOccupancy();
    //octree.toMaxLikelihood();
    //octree.prune();

    octomap_msgs::Octomap octomapMsg;
    octomap_msgs::fullMapToMsg(octree, octomapMsg);
    octomapMsg.header.seq = octomapSequenceNumber++;
    octomapMsg.header.stamp = ros::Time::now();
    octomapMsg.header.frame_id = "hololens_world";
    octomapPublisher.publish(octomapMsg);
}