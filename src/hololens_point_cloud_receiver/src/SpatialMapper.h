#include <stdlib.h>
#include <string>

#include "Base64.h"
#include "DepthMap.h"
#include "Topics.h"

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/PixelDirection.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>

#define SHORT_THROW_MIN_RELIABLE_DEPTH 0.2f
#define SHORT_THROW_MAX_RELIABLE_DEPTH 1.0f

#define LONG_THROW_MIN_RELIABLE_DEPTH 0.5f
#define LONG_THROW_MAX_RELIABLE_DEPTH 4.0f

class SpatialMapper
{
public:
    SpatialMapper(ros::NodeHandle n);

    void handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

    void clearPointCloud();
    void savePointCloud();

private:
    void handleDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame, 
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minReliableDepth,
        const float maxReliableDepth,
        const ros::Publisher& imagePublisher,
        uint32_t* sequenceNumber);

    pcl::PointCloud<pcl::PointXYZ>::Ptr computePointCloudFromDepthMap(
        const DepthMap depthMap, 
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minReliableDepth,
        const float maxReliableDepth);

    Eigen::Matrix4f computeCamToWorldFromDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
        const float leafSize);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float numNeighborsToCheck,
        const float standardDeviationMultiplier);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registerPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace,
        Eigen::Matrix4f camToWorld);

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void publishHololensPosition(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);
    void publishHololensCamToWorldTf(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);
    void publishDepthImage(
        const DepthMap depthMap, 
        const ros::Publisher& publisher, 
        uint32_t* sequenceNumber,
        const float maxReliableDepth);

private:
    hololens_point_cloud_msgs::PixelDirections::ConstPtr shortThrowDirections;
    hololens_point_cloud_msgs::PixelDirections::ConstPtr longThrowDirections;

    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher pointCloudPublisher;
    ros::Publisher hololensPositionPublisher;

    tf::TransformBroadcaster hololensCamPublisher;

    uint32_t shortThrowSequenceNumber;
    uint32_t longThrowSequenceNumber;
    uint32_t pointCloudSequenceNumber;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;

    boost::mutex pointCloudMutex;
};