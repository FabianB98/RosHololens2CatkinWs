#pragma once

#include "DepthDataReceiver.h"

namespace std {
    template <>
    struct hash<std::pair<uint32_t, uint32_t>>
    {
        std::size_t operator()(const std::pair<uint32_t, uint32_t>& pair) const
        {
            std::size_t res = 17;
            res = res * 31 + hash<float>()(pair.first);
            res = res * 31 + hash<float>()(pair.second);
            return res;
        }
    };
}

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
    // Handles the arrival of a new pixel directions message.
    void handlePixelDirections(
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
        std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection>& pixelDirectionsTarget);

    // Handles the arrival of a new depth frame.
    void handleDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection>& pixelDirections,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        const ros::Publisher& pointCloudWorldSpacePublisher,
        const ros::Publisher& pointCloudFramePublisher,
        uint32_t* sequenceNumber);

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
    std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection> shortThrowDirections;
    std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection> longThrowDirections;

    // ROS publishers.
    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher shortThrowPointCloudWorldSpacePublisher;
    ros::Publisher longThrowPointCloudWorldSpacePublisher;
    ros::Publisher hololensPositionPublisher;
    ros::Publisher shortThrowPointCloudFramePublisher;
    ros::Publisher longThrowPointCloudFramePublisher;
    tf::TransformBroadcaster hololensCamPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t shortThrowSequenceNumber;
    uint32_t longThrowSequenceNumber;
    uint32_t positionSequenceNumber;
};
