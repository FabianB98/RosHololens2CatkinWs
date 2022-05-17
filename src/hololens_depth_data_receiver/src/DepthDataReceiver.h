#include <cmath>
#include <stdlib.h>
#include <string>

#include "Base64.h"
#include "DepthMap.h"
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
    // Constructors.
    DepthDataReceiver(
        ros::NodeHandle n, 
        const std::string shortThrowImageTopic = SHORT_THROW_IMAGE_TOPIC, 
        const std::string longThrowImageTopic = LONG_THROW_IMAGE_TOPIC,
        const std::string shortThrowPointCloudCamSpaceUnfilteredTopic = SHORT_THROW_POINT_CLOUD_CAM_SPACE_UNFILTERED,
        const std::string longThrowPointCloudCamSpaceUnfilteredTopic = LONG_THROW_POINT_CLOUD_CAM_SPACE_UNFILTERED,
        const std::string shortThrowPointCloudCamSpaceTopic = SHORT_THROW_POINT_CLOUD_CAM_SPACE,
        const std::string longThrowPointCloudCamSpaceTopic = LONG_THROW_POINT_CLOUD_CAM_SPACE,
        const std::string shortThrowPointCloudWorldSpaceTopic = SHORT_THROW_POINT_CLOUD_WORLD_SPACE,
        const std::string longThrowPointCloudWorldSpaceTopic = LONG_THROW_POINT_CLOUD_WORLD_SPACE,
        const std::string shortThrowArtificialEndpointsCamSpaceTopic = SHORT_THROW_ARTIFICIAL_ENDPOINTS_CAM_SPACE,
        const std::string longThrowArtificialEndpointsCamSpaceTopic = LONG_THROW_ARTIFICIAL_ENDPOINTS_CAM_SPACE,
        const std::string shortThrowArtificialEndpointsWorldSpaceTopic = SHORT_THROW_ARTIFICIAL_ENDPOINTS_WORLD_SPACE,
        const std::string longThrowArtificialEndpointsWorldSpaceTopic = LONG_THROW_ARTIFICIAL_ENDPOINTS_WORLD_SPACE,
        const std::string hololensPositionTopic = HOLOLENS_POSITION_TOPIC,
        const std::string shortThrowPointCloudFrameTopic = SHORT_THROW_POINT_CLOUD_FRAME,
        const std::string longThrowPointCloudFrameTopic = LONG_THROW_POINT_CLOUD_FRAME);

    // Callbacks for handling the incoming depth frames and pixel directions.
    void handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg);

private:
    // Handles the arrival of a new pixel directions message.
    void handlePixelDirections(
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
        hololens_msgs::PixelDirections::Ptr* pixelDirectionsTarget);

    // Handles the arrival of a new depth frame.
    void handleDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame, 
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        const ros::Publisher& pointCloudCamSpaceUnfilteredPublisher,
        const ros::Publisher& pointCloudCamSpacePublisher,
        const ros::Publisher& pointCloudWorldSpacePublisher,
        const ros::Publisher& artificialEndpointsCamSpacePublisher,
        const ros::Publisher& artificialEndpointsWorldSpacePublisher,
        const ros::Publisher& pointCloudFramePublisher,
        uint32_t* sequenceNumber);

    // Applies a median filter onto a given depth map.
    void applyMedianFilter(DepthMap& depthMap);

    // Computes a point cloud (in camera space) from a given depth map.
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> computePointCloudFromDepthMap(
        const DepthMap depthMap,
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth);

    // Computes the transformation matrix for transforming from camera space to world space.
    Eigen::Matrix4f computeCamToWorldFromDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame);

    // Downsamples a given point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
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
    void publishPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
    void publishHololensPosition(const hololens_msgs::DepthFrame::ConstPtr& depthFrame, const ros::Time& timestamp);
    void publishHololensCamToWorldTf(const hololens_msgs::DepthFrame::ConstPtr& depthFrame, const ros::Time& timestamp);
    void publishDepthImage(
        const DepthMap depthMap,
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
        const ros::Publisher& publisher, 
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth);

    // Switches for whether short throw and/or long throw depth frames should be used for calculating the point cloud.
    bool useShortThrow;
    bool useLongThrow;

    // Switches indicating which filter algorithms should be executed.
    bool doMedianFiltering;
    bool doDownsampling;
    bool doOutlierRemovalRadius;
    bool doOutlierRemovalStatistical;
    bool doOutlierRemovalClustering;

    // Switches indicating which results should be published.
    bool doPublishDepthImage;
    bool doPublishPointCloudCamSpaceUnfiltered;
    bool doPublishPointCloudCamSpace;
    bool doPublishPointCloudWorldSpace;
    bool doPublishArtificialEndpointsCamSpace;
    bool doPublishArtificialEndpointsWorldSpace;
    bool doPublishHololensPosition;
    bool doPublishPointCloudFrame;

    // Hyper parameters used for median filtering.
    int medianFilterWindowSize;

    // Hyper parameters used for downsampling.
    float downsamplingLeafSize;

    // Hyper parameters used for outlier removal.
    double outlierRemovalRadiusSearch;
    int outlierRemovalMinNeighborsInRadius;
    int outlierRemovalNeighborsToCheck;
    double outlierRemovalStdDeviationMultiplier;
    double outlierRemovalClusterTolerance;
    int outlierRemovalMinClusterSize;

    // Sensor intrinsics of the short throw depth sensor.
    float shortThrowMinDepth;
    float shortThrowMinReliableDepth;
    float shortThrowMaxReliableDepth;
    float shortThrowMaxDepth;

    // Sensor intrinsics of the long throw depth sensor.
    float longThrowMinDepth;
    float longThrowMinReliableDepth;
    float longThrowMaxReliableDepth;
    float longThrowMaxDepth;

    // Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a rectangular region
    // of interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
    bool discardNoisyPixelsRect;
    float noisyPixelRemovalRectCenterX;
    float noisyPixelRemovalRectCenterY;
    float noisyPixelRemovalRectWidth;
    float noisyPixelRemovalRectHeight;

    // Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a circular region of
    // interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
    bool discardNoisyPixelsCircle;
    float noisyPixelRemovalCircleCenterX;
    float noisyPixelRemovalCircleCenterY;
    float noisyPixelRemovalCircleRadius;

    // The directions (in camera space) in which each pixel of the depth frames points at.
    hololens_msgs::PixelDirections::Ptr shortThrowDirections;
    hololens_msgs::PixelDirections::Ptr longThrowDirections;

    // ROS publishers.
    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher shortThrowPointCloudCamSpaceUnfilteredPublisher;
    ros::Publisher longThrowPointCloudCamSpaceUnfilteredPublisher;
    ros::Publisher shortThrowPointCloudCamSpacePublisher;
    ros::Publisher longThrowPointCloudCamSpacePublisher;
    ros::Publisher shortThrowPointCloudWorldSpacePublisher;
    ros::Publisher longThrowPointCloudWorldSpacePublisher;
    ros::Publisher shortThrowArtificialEndpointsCamSpacePublisher;
    ros::Publisher longThrowArtificialEndpointsCamSpacePublisher;
    ros::Publisher shortThrowArtificialEndpointsWorldSpacePublisher;
    ros::Publisher longThrowArtificialEndpointsWorldSpacePublisher;
    ros::Publisher hololensPositionPublisher;
    ros::Publisher shortThrowPointCloudFramePublisher;
    ros::Publisher longThrowPointCloudFramePublisher;
    tf::TransformBroadcaster hololensCamPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t shortThrowSequenceNumber;
    uint32_t longThrowSequenceNumber;
    uint32_t positionSequenceNumber;
};