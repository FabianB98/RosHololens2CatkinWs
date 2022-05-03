#include "DepthDataReceiver.h"

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
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

// Switches indicating which filter algorithms should be executed.
// Should new point clouds (i.e. non-registered point clouds calculated from the depth frames) be downsampled?
#define DOWNSAMPLE_NEW_CLOUD true
// Should outliers be removed from new point clouds with a radius outlier removal filter?
#define REMOVE_OUTLIERS_RADIUS true
// Should outliers be removed from new point clouds with a statistical outlier removal filter?
#define REMOVE_OUTLIERS_STATISTICAL false

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Parameters for the voxel grid filter used for downsampling.
#define DOWNSAMPLING_LEAF_SIZE 0.01f    // At most one point allowed in a voxel within this edge length.

// Parameters for the radius outlier removal filter used for removing outliers.
#define OUTLIER_REMOVAL_RADIUS_SEARCH 0.05          // The radius in which to search for neighbors.
#define OUTLIER_REMOVAL_MIN_NEIGHBORS_IN_RADIUS 9   // The minimum amount of neighbors which have to be in the radius.

// Parameters for the statistical outlier removal filter used for removing outliers.
#define OUTLIER_REMOVAL_NEIGHBORS_TO_CHECK 10           // The amount of nearest neighbors to check.
#define OUTLIER_REMOVAL_STD_DEVIATION_MULTIPLIER 0.5    // How far off the nearest neighbors may be from the std dev.

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
#define LONG_THROW_MIN_DEPTH 0.5f           // The minimum depth value which the sensor can report.
#define LONG_THROW_MIN_RELIABLE_DEPTH 1.0f  // The minimum depth value to use. Smaller depth values will be discarded.
#define LONG_THROW_MAX_RELIABLE_DEPTH 3.3f  // The maximum depth value to use. Greater depth values will be discarded.
#define LONG_THROW_MAX_DEPTH 4.0f           // The maximum depth value which the sensor can report.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

DepthDataReceiver::DepthDataReceiver(
    ros::NodeHandle n, 
    const std::string shortThrowImageTopic, 
    const std::string longThrowImageTopic,
    const std::string shortThrowPointCloudCamSpaceTopic,
    const std::string longThrowPointCloudCamSpaceTopic,
    const std::string shortThrowPointCloudWorldSpaceTopic,
    const std::string longThrowPointCloudWorldSpaceTopic,
    const std::string hololensPositionTopic)
{
    ROS_INFO("Creating DepthDataReceiver...");

    // Initialize all switches as specified by parameters (or their default value).
    n.param("useShortThrow", useShortThrow, USE_SHORT_THROW);
    n.param("useLongThrow", useLongThrow, USE_LONG_THROW);
    n.param("doDownsampling", doDownsampling, DOWNSAMPLE_NEW_CLOUD);
    n.param("doOutlierRemovalRadius", doOutlierRemovalRadius, REMOVE_OUTLIERS_RADIUS);
    n.param("doOutlierRemovalStatistical", doOutlierRemovalStatistical, REMOVE_OUTLIERS_STATISTICAL);

    // Initialize all hyper parameters as specified by parameters (or their default value).
    n.param("downsamplingLeafSize", downsamplingLeafSize, DOWNSAMPLING_LEAF_SIZE);
    n.param("outlierRemovalRadiusSearch", outlierRemovalRadiusSearch, OUTLIER_REMOVAL_RADIUS_SEARCH);
    n.param("outlierRemovalMinNeighborsInRadius", outlierRemovalMinNeighborsInRadius, OUTLIER_REMOVAL_MIN_NEIGHBORS_IN_RADIUS);
    n.param("outlierRemovalNeighborsToCheck", outlierRemovalNeighborsToCheck, OUTLIER_REMOVAL_NEIGHBORS_TO_CHECK);
    n.param("outlierRemovalStdDeviationMultiplier", outlierRemovalStdDeviationMultiplier, OUTLIER_REMOVAL_STD_DEVIATION_MULTIPLIER);

    // Initialize all sensor intrinsics as specified by parameters (or their default value).
    n.param("shortThrowMinDepth", shortThrowMinDepth, SHORT_THROW_MIN_DEPTH);
    n.param("shortThrowMinReliableDepth", shortThrowMinReliableDepth, SHORT_THROW_MIN_RELIABLE_DEPTH);
    n.param("shortThrowMaxReliableDepth", shortThrowMaxReliableDepth, SHORT_THROW_MAX_RELIABLE_DEPTH);
    n.param("shortThrowMaxDepth", shortThrowMaxDepth, SHORT_THROW_MAX_DEPTH);
    n.param("longThrowMinDepth", longThrowMinDepth, LONG_THROW_MIN_DEPTH);
    n.param("longThrowMinReliableDepth", longThrowMinReliableDepth, LONG_THROW_MIN_RELIABLE_DEPTH);
    n.param("longThrowMaxReliableDepth", longThrowMaxReliableDepth, LONG_THROW_MAX_RELIABLE_DEPTH);
    n.param("longThrowMaxDepth", longThrowMaxDepth, LONG_THROW_MAX_DEPTH);

    // Advertise the topics to which the raw depth images (short throw & long throw) will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(shortThrowImageTopic, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(longThrowImageTopic, 10);
    shortThrowPointCloudCamSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(shortThrowPointCloudCamSpaceTopic, 10);
    longThrowPointCloudCamSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(longThrowPointCloudCamSpaceTopic, 10);
    shortThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(shortThrowPointCloudWorldSpaceTopic, 10);
    longThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(longThrowPointCloudWorldSpaceTopic, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(hololensPositionTopic, 10);
}

void DepthDataReceiver::handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    if (useShortThrow)
        handleDepthFrame(msg, shortThrowDirections, shortThrowMinDepth, shortThrowMinReliableDepth,
                shortThrowMaxReliableDepth, shortThrowMaxDepth, shortThrowImagePublisher,
                shortThrowPointCloudCamSpacePublisher, shortThrowPointCloudWorldSpacePublisher,
                &shortThrowSequenceNumber);
}

void DepthDataReceiver::handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    if (useLongThrow)
        handleDepthFrame(msg, longThrowDirections, longThrowMinDepth, longThrowMinReliableDepth, 
                longThrowMaxReliableDepth, longThrowMaxDepth, longThrowImagePublisher, 
                longThrowPointCloudCamSpacePublisher, longThrowPointCloudWorldSpacePublisher,
                &longThrowSequenceNumber);
}

void DepthDataReceiver::handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    shortThrowDirections = msg;
}

void DepthDataReceiver::handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    longThrowDirections = msg;
}

void DepthDataReceiver::handleDepthFrame(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame, 
    const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
    const float minDepth,
    const float minReliableDepth,
    const float maxReliableDepth,
    const float maxDepth,
    const ros::Publisher& imagePublisher,
    const ros::Publisher& pointCloudCamSpacePublisher,
    const ros::Publisher& pointCloudWorldSpacePublisher,
    uint32_t* sequenceNumber)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight, 
            depthFrame->depthMapPixelStride, false);

    // Calculate the point cloud (in camera space).
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace = computePointCloudFromDepthMap(depthMap, pixelDirections,
            minReliableDepth, maxReliableDepth);

    // Downsample the point cloud to ensure that the point density is not too high.
    if (doDownsampling)
        pointCloudCamSpace = downsamplePointCloud(pointCloudCamSpace, downsamplingLeafSize);

    // Remove outliers (measurement errors, extremely noisy values, ...) from the point cloud.
    if (doOutlierRemovalRadius)
        pointCloudCamSpace = removeOutliersRadius(pointCloudCamSpace, outlierRemovalRadiusSearch, 
                outlierRemovalMinNeighborsInRadius);
    if (doOutlierRemovalStatistical)
        pointCloudCamSpace = removeOutliersStatistical(pointCloudCamSpace, outlierRemovalNeighborsToCheck, 
                outlierRemovalStdDeviationMultiplier);

    // Calculate the transformation from camera space to world space and transform the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromDepthFrame(depthFrame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);

    // TODO: Remove the following line of code later on...
    ROS_INFO("Current frame consists of %zu points.", pointCloudCamSpace->size());

    // Publish the depth map, the HoloLens's current position and the computed point cloud.
    publishHololensPosition(depthFrame);
    publishHololensCamToWorldTf(depthFrame);
    publishDepthImage(depthMap, imagePublisher, *sequenceNumber, minDepth, minReliableDepth, maxReliableDepth, maxDepth);
    publishPointCloud(pointCloudCamSpace, pointCloudCamSpacePublisher, *sequenceNumber, "hololens_cam");
    publishPointCloud(pointCloudWorldSpace, pointCloudWorldSpacePublisher, *sequenceNumber, "hololens_world");
    sequenceNumber++;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataReceiver::computePointCloudFromDepthMap(
    const DepthMap depthMap, 
    const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
    const float minReliableDepth,
    const float maxReliableDepth)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZ>());
    
    // Iterate over each pixel of the depth frame.
    for (uint32_t i = 0; i < pixelDirections->pixelDirections.size(); ++i)
    {
        // Get the depth at the current pixel.
        hololens_msgs::PixelDirection dir = pixelDirections->pixelDirections.at(i);
        uint32_t pixelValue = depthMap.valueAt(dir.u, dir.v);
        float depth = static_cast<float>(pixelValue) / 1000.0f;

        // Skip pixels whose depth value are not within the reliable depth range.
        if (depth < minReliableDepth || depth > maxReliableDepth)
            continue;

        // Calculate the point for the current pixel based on the pixels depth value.
        pointCloudCamSpace->push_back(
                pcl::PointXYZ(dir.direction.x * depth, dir.direction.y * depth, dir.direction.z * depth));
    }

    // Return the calculated point cloud.
    return pointCloudCamSpace;
}

Eigen::Matrix4f DepthDataReceiver::computeCamToWorldFromDepthFrame(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame)
{
    // Create the camera to world transformation matrix which will be returned later on.
    Eigen::Matrix4f camToWorld = Eigen::Matrix4f::Identity();

    // Set the rotational part of the camera to world transformation matrix.
    camToWorld.block(0, 0, 3, 3) = Eigen::Quaternionf(
        depthFrame->camToWorldRotation.w, 
        depthFrame->camToWorldRotation.x, 
        depthFrame->camToWorldRotation.y, 
        depthFrame->camToWorldRotation.z
    ).toRotationMatrix();

    // Set the translational part of the camera to world transformation matrix.
    camToWorld(0, 3) = depthFrame->camToWorldTranslation.x;
    camToWorld(1, 3) = depthFrame->camToWorldTranslation.y;
    camToWorld(2, 3) = depthFrame->camToWorldTranslation.z;

    // Return the camera to world transformation matrix.
    return camToWorld;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataReceiver::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
    const float leafSize)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDownsampled (new pcl::PointCloud<pcl::PointXYZ>());

    // Downsample the given point cloud using a voxel grid filter.
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(pointCloudToDownsample);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*pointCloudDownsampled);

    // Return the downsampled point cloud.
    return pointCloudDownsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataReceiver::removeOutliersRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
    const float radiusSearch,
    const int minNeighborsInRadius)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>());

    // Remove outliers by using a radius outlier removal.
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval;
    radiusOutlierRemoval.setInputCloud(pointCloudToFilter);
    radiusOutlierRemoval.setRadiusSearch(radiusSearch);
    radiusOutlierRemoval.setMinNeighborsInRadius(minNeighborsInRadius);
    radiusOutlierRemoval.filter(*pointCloudFiltered);

    // Return the point cloud with the outliers removed.
    return pointCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataReceiver::removeOutliersStatistical(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
    const float numNeighborsToCheck,
    const float standardDeviationMultiplier)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>());

    // Remove outliers by using a statistical outlier removal.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(pointCloudToFilter);
    statisticalOutlierRemoval.setMeanK(numNeighborsToCheck);
    statisticalOutlierRemoval.setStddevMulThresh(standardDeviationMultiplier);
    statisticalOutlierRemoval.filter(*pointCloudFiltered);

    // Return the point cloud with the outliers removed.
    return pointCloudFiltered;
}

void DepthDataReceiver::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Publisher& publisher,
    uint32_t sequenceNumber, const std::string frameId)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    // Set the header of the message.
    pointCloudMessage.header.seq = sequenceNumber;
    pointCloudMessage.header.stamp = ros::Time::now();
    pointCloudMessage.header.frame_id = frameId;

    // Publish the message.
    publisher.publish(pointCloudMessage);
}

void DepthDataReceiver::publishHololensPosition(const hololens_msgs::DepthFrame::ConstPtr& depthFrame)
{
    // Create the ROS message for the current position of the HoloLens.
    geometry_msgs::PointStamped hololensPosition;

    // Set the header of the message.
    hololensPosition.header.seq = positionSequenceNumber;
    hololensPosition.header.stamp = ros::Time::now();
    hololensPosition.header.frame_id = "hololens_world";

    // Add the position of the HoloLens to the message.
    hololensPosition.point.x = depthFrame->camToWorldTranslation.x;
    hololensPosition.point.y = depthFrame->camToWorldTranslation.y;
    hololensPosition.point.z = depthFrame->camToWorldTranslation.z;

    // Publish the message.
    hololensPositionPublisher.publish(hololensPosition);
    positionSequenceNumber++;
}

void DepthDataReceiver::publishHololensCamToWorldTf(const hololens_msgs::DepthFrame::ConstPtr& depthFrame)
{
    // Create the transform instance which will be published.
    tf::Transform transform;

    // Set the translational part of the transform.
    transform.setOrigin(tf::Vector3(
        depthFrame->camToWorldTranslation.x,
        depthFrame->camToWorldTranslation.y,
        depthFrame->camToWorldTranslation.z));

    // Set the rotational part of the transform.
    transform.setRotation(tf::Quaternion(
        depthFrame->camToWorldRotation.x,
        depthFrame->camToWorldRotation.y,
        depthFrame->camToWorldRotation.z,
        depthFrame->camToWorldRotation.w));
    
    // Publish the transform.
    hololensCamPublisher.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "hololens_world", "hololens_cam"));
}

void DepthDataReceiver::publishDepthImage(
    const DepthMap depthMap, 
    const ros::Publisher& publisher, 
    uint32_t sequenceNumber,
    const float minDepth,
    const float minReliableDepth,
    const float maxReliableDepth,
    const float maxDepth)
{
    // Create the ROS message for the image.
    sensor_msgs::Image image;

    // Set the header of the image.
    image.header.seq = sequenceNumber;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "hololens_cam";

    // Set the meta data (width, height, encoding, ...) of the image.
    image.height = depthMap.height;
    image.width = depthMap.width;
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = 0;
    image.step = depthMap.width * 3;

    // Brighten the image, color all unreliable pixels and add the image it to the message.
    for (uint32_t v = 0; v < image.height; ++v)
    {
        for (uint32_t u = 0; u < image.width; ++u)
        {
            // Get the depth of the current pixel.
            uint32_t pixelValue = depthMap.valueAt(u, v);
            float depth = static_cast<float>(pixelValue) / 1000.0f;

            // Calculate the greyscale value which will be used for the pixel.
            uint8_t greyscaleValue = static_cast<uint8_t>(255.0f * depth / maxDepth);

            // Check if the depth is within a the (reliable) depth range.
            if (depth < minDepth || depth > maxDepth)
            {
                // Depth is outside the depth range. Color the current pixel black.
                image.data.push_back(0);
                image.data.push_back(0);
                image.data.push_back(0);
            }
            else if (depth < minReliableDepth)
            {
                // Depth too small to be considered as reliable. Color the current pixel purple.
                image.data.push_back(greyscaleValue * 3);
                image.data.push_back(0);
                image.data.push_back(greyscaleValue * 3);
            }
            else if (depth > maxReliableDepth)
            {
                // Depth is too high to be considered as reliable. Color the current pixel red.
                image.data.push_back(greyscaleValue);
                image.data.push_back(0);
                image.data.push_back(0);
            }
            else
            {
                // Depth is inside the reliable depth range. Color the current pixel grey.
                image.data.push_back(greyscaleValue);
                image.data.push_back(greyscaleValue);
                image.data.push_back(greyscaleValue);
            }
        }
    }
    
    // Publish the message.
    publisher.publish(image);
}
