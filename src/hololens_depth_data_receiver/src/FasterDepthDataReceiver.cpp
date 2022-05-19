#include "FasterDepthDataReceiver.h"

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

#include <pcl_conversions/pcl_conversions.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                       SWITCHES FOR CHANGING THE DEFAULT BEHAVIOUR OF THE DEPTH DATA RECEIVER                       //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches for whether short throw and/or long throw depth frames should be used for calculating the point cloud.
// Should short throw depth frames be used for calculating the point cloud?
#define USE_SHORT_THROW false
// Should long throw depth frames be used for calculating the point cloud?
#define USE_LONG_THROW true

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                           DEFAULT DEPTH RANGES (TOTAL AND RELIABLE) OF THE DEPTH SENSORS                           //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// The depth sensor intrinsics of the short throw depth sensor.
#define SHORT_THROW_MIN_DEPTH 0.2f          // The minimum depth value which the sensor can report.
#define SHORT_THROW_MIN_RELIABLE_DEPTH 0.2f // The minimum depth value to use. Smaller depth values will be discarded.
#define SHORT_THROW_MAX_RELIABLE_DEPTH 1.0f // The maximum depth value to use. Greater depth values will be discarded.
#define SHORT_THROW_MAX_DEPTH 1.0f          // The maximum depth value which the sensor can report.

// The depth sensor intrinsics of the long throw depth sensor.
#define LONG_THROW_MIN_DEPTH 0.2f           // The minimum depth value which the sensor can report.
#define LONG_THROW_MIN_RELIABLE_DEPTH 0.75f // The minimum depth value to use. Smaller depth values will be discarded.
#define LONG_THROW_MAX_RELIABLE_DEPTH 8.0f  // The maximum depth value to use. Greater depth values will be discarded.
#define LONG_THROW_MAX_DEPTH 16.0f          // The maximum depth value which the sensor can report.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                      SWITCHES AND DEFAULT HYPER PARAMETERS RELATED TO REMOVAL OF NOISY PIXELS                      //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a rectangular region of
// interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
#define DISCARD_NOISY_PIXELS_RECT false
#define NOISY_PIXEL_REMOVAL_RECT_CENTER_X 0.5f
#define NOISY_PIXEL_REMOVAL_RECT_CENTER_Y 0.5f
#define NOISY_PIXEL_REMOVAL_RECT_WIDTH 0.8f
#define NOISY_PIXEL_REMOVAL_RECT_HEIGHT 0.8f

// Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a circular region of
// interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
#define DISCARD_NOISY_PIXELS_CIRCLE false
#define NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_X 0.5f
#define NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_Y 0.5f
#define NOISY_PIXEL_REMOVAL_CIRCLE_RADIUS 0.45f

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

FasterDepthDataReceiver::FasterDepthDataReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating FasterDepthDataReceiver...");

    // Initialize all switches as specified by parameters (or their default value).
    n.param("useShortThrow", useShortThrow, USE_SHORT_THROW);
    n.param("useLongThrow", useLongThrow, USE_LONG_THROW);

    // Initialize all sensor intrinsics as specified by parameters (or their default value).
    n.param("shortThrowMinDepth", shortThrowMinDepth, SHORT_THROW_MIN_DEPTH);
    n.param("shortThrowMinReliableDepth", shortThrowMinReliableDepth, SHORT_THROW_MIN_RELIABLE_DEPTH);
    n.param("shortThrowMaxReliableDepth", shortThrowMaxReliableDepth, SHORT_THROW_MAX_RELIABLE_DEPTH);
    n.param("shortThrowMaxDepth", shortThrowMaxDepth, SHORT_THROW_MAX_DEPTH);
    n.param("longThrowMinDepth", longThrowMinDepth, LONG_THROW_MIN_DEPTH);
    n.param("longThrowMinReliableDepth", longThrowMinReliableDepth, LONG_THROW_MIN_RELIABLE_DEPTH);
    n.param("longThrowMaxReliableDepth", longThrowMaxReliableDepth, LONG_THROW_MAX_RELIABLE_DEPTH);
    n.param("longThrowMaxDepth", longThrowMaxDepth, LONG_THROW_MAX_DEPTH);

    // Initialize all switches and sensor intrinsics related to removal of noisy pixels as specified by parameters (or
    // their default value).
    n.param("discardNoisyPixelsRect", discardNoisyPixelsRect, DISCARD_NOISY_PIXELS_RECT);
    n.param("noisyPixelRemovalRectCenterX", noisyPixelRemovalRectCenterX, NOISY_PIXEL_REMOVAL_RECT_CENTER_X);
    n.param("noisyPixelRemovalRectCenterY", noisyPixelRemovalRectCenterY, NOISY_PIXEL_REMOVAL_RECT_CENTER_Y);
    n.param("noisyPixelRemovalRectWidth", noisyPixelRemovalRectWidth, NOISY_PIXEL_REMOVAL_RECT_WIDTH);
    n.param("noisyPixelRemovalRectHeight", noisyPixelRemovalRectHeight, NOISY_PIXEL_REMOVAL_RECT_HEIGHT);
    n.param("discardNoisyPixelsCircle", discardNoisyPixelsCircle, DISCARD_NOISY_PIXELS_CIRCLE);
    n.param("noisyPixelRemovalCircleCenterX", noisyPixelRemovalCircleCenterX, NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_X);
    n.param("noisyPixelRemovalCircleCenterY", noisyPixelRemovalCircleCenterY, NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_Y);
    n.param("noisyPixelRemovalCircleRadius", noisyPixelRemovalCircleRadius, NOISY_PIXEL_REMOVAL_CIRCLE_RADIUS);

    // Advertise the topics to which the results will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(SHORT_THROW_IMAGE_TOPIC, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(LONG_THROW_IMAGE_TOPIC, 10);
    shortThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(SHORT_THROW_POINT_CLOUD_CAM_SPACE_TOPIC, 10);
    longThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(LONG_THROW_POINT_CLOUD_CAM_SPACE_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);
    shortThrowPointCloudFramePublisher = n.advertise<hololens_depth_data_receiver_msgs::PointCloudFrame>(SHORT_THROW_POINT_CLOUD_FRAME_TOPIC, 10);
    longThrowPointCloudFramePublisher = n.advertise<hololens_depth_data_receiver_msgs::PointCloudFrame>(LONG_THROW_POINT_CLOUD_FRAME_TOPIC, 10);
}

void FasterDepthDataReceiver::handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    if (useShortThrow)
        handleDepthFrame(msg, shortThrowDirections, shortThrowMinDepth, shortThrowMinReliableDepth,
                shortThrowMaxReliableDepth, shortThrowMaxDepth, shortThrowImagePublisher,
                shortThrowPointCloudWorldSpacePublisher, shortThrowPointCloudFramePublisher,
                &shortThrowSequenceNumber);
}

void FasterDepthDataReceiver::handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    if (useLongThrow)
        handleDepthFrame(msg, longThrowDirections, longThrowMinDepth, longThrowMinReliableDepth, 
                longThrowMaxReliableDepth, longThrowMaxDepth, longThrowImagePublisher, 
                longThrowPointCloudWorldSpacePublisher, longThrowPointCloudFramePublisher,
                &longThrowSequenceNumber);
}

void FasterDepthDataReceiver::handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    handlePixelDirections(msg, shortThrowDirections);
}

void FasterDepthDataReceiver::handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    handlePixelDirections(msg, longThrowDirections);
}

// Handles the arrival of a new pixel directions message.
void FasterDepthDataReceiver::handlePixelDirections(
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
        std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection>& pixelDirectionsTarget)
{
    pixelDirectionsTarget.clear();

    // Note: From a software engineering perspective it would be better to actually have two separate methods for
    // applying the rectangular filter and the circular filter. However, it was chosen to apply both filters in the same
    // method due to the fact that some calculations can be shared between both filters, so there is some performance
    // benefit of applying both filters in the same method.

    // Determine the size of the region for which we got pixel directions.
    uint32_t min_u = pixelDirectionsMsg->pixelDirections.at(0).u;
    uint32_t max_u = pixelDirectionsMsg->pixelDirections.at(0).u;
    uint32_t min_v = pixelDirectionsMsg->pixelDirections.at(0).v;
    uint32_t max_v = pixelDirectionsMsg->pixelDirections.at(0).v;

    for (uint32_t i = 0; i < pixelDirectionsMsg->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirectionsMsg->pixelDirections.at(i);
        if (dir.u < min_u) min_u = dir.u;
        if (dir.u > max_u) max_u = dir.u;
        if (dir.v < min_v) min_v = dir.v;
        if (dir.v > max_v) max_v = dir.v;
    }

    float width = static_cast<float>(max_u - min_u);
    float height = static_cast<float>(max_v - min_v);

    // Calculate the coordinates of the rectangle's edges.
    float rectUMinRelative = noisyPixelRemovalRectCenterX - 0.5f * noisyPixelRemovalRectWidth;
    float rectUMaxRelative = noisyPixelRemovalRectCenterX + 0.5f * noisyPixelRemovalRectWidth;
    float rectVMinRelative = noisyPixelRemovalRectCenterY - 0.5f * noisyPixelRemovalRectHeight;
    float rectVMaxRelative = noisyPixelRemovalRectCenterY + 0.5f * noisyPixelRemovalRectHeight;

    uint32_t rectUMin = min_u + static_cast<uint32_t>(roundf(rectUMinRelative * width));
    uint32_t rectUMax = min_u + static_cast<uint32_t>(roundf(rectUMaxRelative * width));
    uint32_t rectVMin = min_v + static_cast<uint32_t>(roundf(rectVMinRelative * height));
    uint32_t rectVMax = min_v + static_cast<uint32_t>(roundf(rectVMaxRelative * height));

    // Calculate the coordinates of the circle's center point and its radius.
    uint32_t circleCenterU = min_u + static_cast<uint32_t>(roundf(noisyPixelRemovalCircleCenterX * width));
    uint32_t circleCenterV = min_v + static_cast<uint32_t>(roundf(noisyPixelRemovalCircleCenterY * height));
    float circleRadius = noisyPixelRemovalCircleRadius * std::max(width, height);
    float circleRadiusSquared = circleRadius * circleRadius;

    // Check for each pixel direction whether the corresponding pixel is inside the rectangle and the circle.
    for (uint32_t i = 0; i < pixelDirectionsMsg->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirectionsMsg->pixelDirections.at(i);

        bool insideRect = !discardNoisyPixelsRect 
            || (rectUMin <= dir.u && dir.u <= rectUMax && rectVMin <= dir.v && dir.v <= rectVMax);
        bool insideCircle = !discardNoisyPixelsCircle 
            || ((dir.u - circleCenterU) * (dir.u - circleCenterU) + (dir.v - circleCenterV) * (dir.v - circleCenterV) <= circleRadiusSquared);

        if (insideRect && insideCircle)
        {
            pixelDirectionsTarget[std::make_pair(dir.u, dir.v)] = dir;
        }
    }
    ROS_INFO("There are %zu pixel directions after filtering.", pixelDirectionsTarget.size());
}

// Handles the arrival of a new depth frame.
void FasterDepthDataReceiver::handleDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const std::unordered_map<std::pair<uint32_t, uint32_t>, hololens_msgs::PixelDirection>& pixelDirections,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        const ros::Publisher& pointCloudWorldSpacePublisher,
        const ros::Publisher& pointCloudFramePublisher,
        uint32_t* sequenceNumber)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight,
            depthFrame->depthMapPixelStride, false);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //      START OF SOMEWHAT EXPERIMENTAL CODE. EVERYTHING BELOW NEEDS TO BE TIDIED UP.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // TODO: Clustering of pixels in the depth frame (in image space).

    // TODO: Removal of clusters with too less points.

    // TODO: Downsampling of all clusters (downsample each cluster separately)

    // TODO: Combine all clusters into a single point cloud.




    // Everything below is copied (and modified) code from computePointCloudFromDepthMap in InitialDepthDataReceiver.

    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZ>());
    
    // Iterate over each pixel of the depth frame.
    for (auto it = pixelDirections.begin(); it != pixelDirections.end(); ++it)
    {
        // Get the depth at the current pixel.
        hololens_msgs::PixelDirection dir = it->second;
        uint32_t pixelValue = depthMap.valueAt(dir.u, dir.v);
        float depth = static_cast<float>(pixelValue) / 1000.0f;

        // Skip pixels whose depth values are not within the reliable depth range.
        if (depth < minReliableDepth || depth > maxReliableDepth)
            continue;

        // Calculate the point for the current pixel based on the pixels depth value.
        pcl::PointXYZ point = pcl::PointXYZ(dir.direction.x * depth, dir.direction.y * depth, dir.direction.z * depth);
        if (minReliableDepth <= depth && depth <= maxReliableDepth)
            pointCloudCamSpace->push_back(point);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //      END OF SOMEWHAT EXPERIMENTAL CODE. EVERYTHING ABOVE NEEDS TO BE TIDIED UP.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




    // Calculate the transformation from camera space to world space and transform the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromDepthFrame(depthFrame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);

    // Publish the depth map, the HoloLens's current position and the computed point cloud.
    ros::Time time = ros::Time::now();
    sensor_msgs::PointCloud2 pointCloudWorldSpaceMsg;
    sensor_msgs::PointCloud2 artificialEndpointsWorldSpaceMsg;
    pointCloudToMsg(pointCloudWorldSpace, pointCloudWorldSpaceMsg, *sequenceNumber, time, "hololens_world");
    publishHololensPosition(depthFrame, hololensPositionPublisher, positionSequenceNumber, time);
    publishHololensCamToWorldTf(depthFrame, hololensCamPublisher, time);
    // TODO: Publishing the depth image needs to be fixed.
    // publishDepthImage(depthMap, pixelDirections, imagePublisher, *sequenceNumber, time, minDepth, minReliableDepth,
    //         maxReliableDepth, maxDepth);
    pointCloudWorldSpacePublisher.publish(pointCloudWorldSpaceMsg);
    publishPointCloudFrame(pointCloudFramePublisher, pointCloudWorldSpaceMsg, artificialEndpointsWorldSpaceMsg,
            depthFrame, maxReliableDepth);
    sequenceNumber++;
    positionSequenceNumber++;
}