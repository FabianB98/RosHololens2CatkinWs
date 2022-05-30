#pragma once

#include <cmath>
#include <stdlib.h>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "Base64.h"
#include "DepthMap.h"
#include "Pixel.h"
#include "Topics.h"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

#include "hololens_msgs/DepthFrame.h"
#include "hololens_msgs/PixelDirection.h"
#include "hololens_msgs/PixelDirections.h"
#include "hololens_msgs/Point.h"

#include "hololens_depth_data_receiver_msgs/PointCloudFrame.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_broadcaster.h>

class DepthDataReceiver
{
public:
    DepthDataReceiver() {}
    virtual ~DepthDataReceiver() {}

    // Callbacks for handling the incoming depth frames and pixel directions.
    virtual void handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg) = 0;
    virtual void handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg) = 0;
    virtual void handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg) = 0;
    virtual void handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg) = 0;

protected:
    // Computes the transformation matrix for transforming from camera space to world space.
    Eigen::Matrix4f computeCamToWorldFromDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame);

    // Downsamples a given point cloud.
    // I don't understand why, but as soon as I try to use any PCL filter or algorithm (such as the VoxelGrid used for
    // downsampling for example) inside a templated function I get a ton of compiler errors stating that the filter
    // either is no member of pcl or is not templated (even though the PCL documentation literally states VoxelGrid to
    // be templated as VoxelGrid<PointT>...). These errors don't make any sense at all and I don't know how to resolve
    // them, so I'll just have to resort to duplicating these functions...
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
        const float leafSize);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToDownsample,
        const float leafSize);
    
    // Removes outliers from a given point cloud with a radius outlier removal filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersRadius(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float radiusSearch,
        const int minNeighborsInRadius);

    // Removes outliers from a given point cloud with a statistical outlier removal filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersStatistical(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float numNeighborsToCheck,
        const float standardDeviationMultiplier);

    // Removes outliers from a given point cloud by clustering the points and discarding too small clusters.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const double clusterTolerance,
        const int minClusterSize);

    // Methods for publishing the results.
    void pointCloudToMsg(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        sensor_msgs::PointCloud2& message,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
    void pointCloudToMsg(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        sensor_msgs::PointCloud2& message,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
    void publishPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
    void publishPointCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
    void publishPointCloudFrame(
        const ros::Publisher& publisher,
        sensor_msgs::PointCloud2& pointCloudWorldSpaceMsg,
        sensor_msgs::PointCloud2& artificialEndpointsWorldSpaceMsg,
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const float maxReliableDepth);
    void publishHololensPosition(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishHololensCamToWorldTf(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        tf::TransformBroadcaster& publisher,
        const ros::Time& timestamp);
};