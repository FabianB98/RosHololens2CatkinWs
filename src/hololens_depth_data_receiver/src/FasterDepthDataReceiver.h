#pragma once

#include "DepthDataReceiver.h"

class FasterDepthDataReceiver : public DepthDataReceiver
{
public:
    // Constructors.
    FasterDepthDataReceiver(ros::NodeHandle n);

    // Callbacks for handling the incoming depth frames and pixel directions.
    void handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg);

private:
    // Initializes a pixel neighborhood based on the euclidean distance to the centermost pixel.
    std::vector<Pixel> initializeEuclideanDistanceNeighborhood(double neighborDistance);

    // Handles the arrival of a new pixel directions message.
    void handlePixelDirections(
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
        std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>* pixelDirectionsLookupTableTarget);

    // Handles the arrival of a new depth frame.
    void handleDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        const ros::Publisher& pointCloudWorldSpacePublisher,
        const ros::Publisher& artificialEndpointsWorldSpacePublisher,
        const ros::Publisher& pointCloudFramePublisher,
        uint32_t* sequenceNumber);

    // Detects clusters (in image space) in the given depth map.
    std::vector<std::vector<Pixel>> detectDepthClustersWithRegionGrowing(DepthMap& depthMap);
    std::vector<std::vector<Pixel>> detectDepthClustersWithPclClustering(DepthMap& depthMap);

    // Reconstructs a point cloud from the data stored in the given depth map for all pixels which are part of any of
    // the given clusters.
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> createPointCloudFromClusters(
        const std::vector<std::vector<Pixel>>& depthClusters,
        DepthMap& depthMap,
        DepthMap& reflectivityImage,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth);

    // Publishes a colored image displaying information about which pixels of the depth map were used to reconstruct the
    // observed environment and to which cluster they were assigned to.
    void publishDepthImage(
        const DepthMap depthMap,
        const std::vector<std::vector<Pixel>>& depthClusters,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
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

    // Hyper parameters for clustering of similar pixels in received depth maps.
    bool pixelClusteringUsePcl;
    double pixelClusteringNeighboringPixelDistance;
    int pixelClusteringAbsoluteDepthValueSimilarity;
    uint32_t pixelClusteringSquaredDepthValueSimilarity;
    int pixelClusteringMinClusterSize;
    std::vector<Pixel> pixelClusteringNeighborhood;
    std::vector<std::vector<float>> pixelClusterColors;

    // Hyper parameters used for downsampling.
    float downsamplingLeafSize;
    int downsamplingMinPointsPerVoxel;

    // The directions (in camera space) in which each pixel of the depth frames points at.
    std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>> shortThrowDirectionLookupTable;
    std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>> longThrowDirectionLookupTable;

    // ROS publishers.
    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher shortThrowPointCloudWorldSpacePublisher;
    ros::Publisher longThrowPointCloudWorldSpacePublisher;
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
