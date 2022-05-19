#pragma once

#include "DepthDataReceiver.h"

class InitialDepthDataReceiver : public DepthDataReceiver
{
public:
    // Constructors.
    InitialDepthDataReceiver(
        ros::NodeHandle n, 
        const std::string shortThrowImageTopic = SHORT_THROW_IMAGE_TOPIC, 
        const std::string longThrowImageTopic = LONG_THROW_IMAGE_TOPIC,
        const std::string shortThrowPointCloudCamSpaceUnfilteredTopic = SHORT_THROW_POINT_CLOUD_CAM_SPACE_UNFILTERED_TOPIC,
        const std::string longThrowPointCloudCamSpaceUnfilteredTopic = LONG_THROW_POINT_CLOUD_CAM_SPACE_UNFILTERED_TOPIC,
        const std::string shortThrowPointCloudCamSpaceTopic = SHORT_THROW_POINT_CLOUD_CAM_SPACE_TOPIC,
        const std::string longThrowPointCloudCamSpaceTopic = LONG_THROW_POINT_CLOUD_CAM_SPACE_TOPIC,
        const std::string shortThrowPointCloudWorldSpaceTopic = SHORT_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC,
        const std::string longThrowPointCloudWorldSpaceTopic = LONG_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC,
        const std::string shortThrowArtificialEndpointsCamSpaceTopic = SHORT_THROW_ARTIFICIAL_ENDPOINTS_CAM_SPACE_TOPIC,
        const std::string longThrowArtificialEndpointsCamSpaceTopic = LONG_THROW_ARTIFICIAL_ENDPOINTS_CAM_SPACE_TOPIC,
        const std::string shortThrowArtificialEndpointsWorldSpaceTopic = SHORT_THROW_ARTIFICIAL_ENDPOINTS_WORLD_SPACE_TOPIC,
        const std::string longThrowArtificialEndpointsWorldSpaceTopic = LONG_THROW_ARTIFICIAL_ENDPOINTS_WORLD_SPACE_TOPIC,
        const std::string hololensPositionTopic = HOLOLENS_POSITION_TOPIC,
        const std::string shortThrowPointCloudFrameTopic = SHORT_THROW_POINT_CLOUD_FRAME_TOPIC,
        const std::string longThrowPointCloudFrameTopic = LONG_THROW_POINT_CLOUD_FRAME_TOPIC);

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