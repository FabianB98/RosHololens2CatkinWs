#include "DepthDataReceiver.h"

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

// Switches indicating which filter algorithms should be executed.
// Should a median filter be applied on the raw depth map?
#define MEDIAN_FILTER_DEPTH_MAP false
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

// Parameters for the median filter used for removing outliers.
#define MEDIAN_FILTER_WINDOW_SIZE 3     // The kernel size of the filter. A value of 3 corresponds to a 3x3 kernel.

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
//                      SWITCHES AND DEFAULT HYPER PARAMETERS RELATED TO REMOVAL OF NOISY PIXELS                      //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a rectangular region of
// interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
#define DISCARD_NOISY_PIXELS_RECT false
#define NOISY_PIXEL_REMOVAL_RECT_CENTER_X 0.5f
#define NOISY_PIXEL_REMOVAL_RECT_CENTER_Y 0.5f
#define NOISY_PIXEL_REMOVAL_RECT_WIDTH 1.0f
#define NOISY_PIXEL_REMOVAL_RECT_HEIGHT 1.0f

// Switches and sensor intrinsics related to discarding noisy pixels of the depth sensors with a circular region of
// interest. Pixels outside that region won't be used in the creation of a point cloud from a sensor frame.
#define DISCARD_NOISY_PIXELS_CIRCLE false
#define NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_X 0.5f
#define NOISY_PIXEL_REMOVAL_CIRCLE_CENTER_Y 0.5f
#define NOISY_PIXEL_REMOVAL_CIRCLE_RADIUS 1.0f

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
    n.param("doMedianFiltering", doMedianFiltering, MEDIAN_FILTER_DEPTH_MAP);
    n.param("doDownsampling", doDownsampling, DOWNSAMPLE_NEW_CLOUD);
    n.param("doOutlierRemovalRadius", doOutlierRemovalRadius, REMOVE_OUTLIERS_RADIUS);
    n.param("doOutlierRemovalStatistical", doOutlierRemovalStatistical, REMOVE_OUTLIERS_STATISTICAL);

    // Initialize all hyper parameters as specified by parameters (or their default value).
    n.param("medianFilterWindowSize", medianFilterWindowSize, MEDIAN_FILTER_WINDOW_SIZE);
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

    // Advertise the topics to which the raw depth images (short throw & long throw) will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(shortThrowImageTopic, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(longThrowImageTopic, 10);
    shortThrowPointCloudCamSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(shortThrowPointCloudCamSpaceTopic, 10);
    longThrowPointCloudCamSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(longThrowPointCloudCamSpaceTopic, 10);
    shortThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(shortThrowPointCloudWorldSpaceTopic, 10);
    longThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(longThrowPointCloudWorldSpaceTopic, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(hololensPositionTopic, 10);

    // Initialize the pixel directions to an empty array each.
    shortThrowDirections = hololens_msgs::PixelDirections::Ptr(new hololens_msgs::PixelDirections());
    longThrowDirections = hololens_msgs::PixelDirections::Ptr(new hololens_msgs::PixelDirections());
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
    handlePixelDirections(msg, &shortThrowDirections);
}

void DepthDataReceiver::handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    handlePixelDirections(msg, &longThrowDirections);
}

void DepthDataReceiver::handlePixelDirections(
    const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
    hololens_msgs::PixelDirections::Ptr* pixelDirectionsTarget)
{
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
    hololens_msgs::PixelDirections::Ptr filteredPixelDirections = hololens_msgs::PixelDirections::Ptr(new hololens_msgs::PixelDirections());
    for (uint32_t i = 0; i < pixelDirectionsMsg->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirectionsMsg->pixelDirections.at(i);

        bool insideRect = !discardNoisyPixelsRect 
            || (rectUMin <= dir.u && dir.u <= rectUMax && rectVMin <= dir.v && dir.v <= rectVMax);
        bool insideCircle = !discardNoisyPixelsCircle 
            || ((dir.u - circleCenterU) * (dir.u - circleCenterU) + (dir.v - circleCenterV) * (dir.v - circleCenterV) <= circleRadiusSquared);

        if (insideRect && insideCircle)
        {
            filteredPixelDirections->pixelDirections.push_back(dir);
        }
    }
    ROS_INFO("There are %zu pixel directions after filtering.", filteredPixelDirections->pixelDirections.size());

    *pixelDirectionsTarget = filteredPixelDirections;
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

    // Apply a median filter onto the depth map to remove outliers (extremely noisy values distributed as "salt and
    // pepper noise") and to change these outliers to a reasonable value.
    if (doMedianFiltering)
        applyMedianFilter(depthMap);

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
    ros::Time time = ros::Time::now();
    publishHololensPosition(depthFrame, time);
    publishHololensCamToWorldTf(depthFrame, time);
    publishDepthImage(depthMap, pixelDirections, imagePublisher, *sequenceNumber, time, minDepth, minReliableDepth,
            maxReliableDepth, maxDepth);
    publishPointCloud(pointCloudCamSpace, pointCloudCamSpacePublisher, *sequenceNumber, time, "hololens_cam");
    publishPointCloud(pointCloudWorldSpace, pointCloudWorldSpacePublisher, *sequenceNumber, time, "hololens_world");
    sequenceNumber++;
}

void DepthDataReceiver::applyMedianFilter(DepthMap& depthMap)
{
    // Convert the depth map to an organized point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthMapCloud (new pcl::PointCloud<pcl::PointXYZ>());
    depthMapCloud->width = depthMap.width;
    depthMapCloud->height = depthMap.height;
    depthMapCloud->is_dense = false;
    depthMapCloud->points.resize(depthMap.width * depthMap.height);

    for (uint32_t u = 0; u < depthMap.width; u++)
    {
        for (uint32_t v = 0; v < depthMap.height; v++)
        {
            depthMapCloud->at(u, v).x = static_cast<float>(u);
            depthMapCloud->at(u, v).y = static_cast<float>(v);
            depthMapCloud->at(u, v).z = static_cast<float>(depthMap.valueAt(u, v));
        }
    }

    // Apply a median filter over the organized point cloud.
    pcl::PointCloud<pcl::PointXYZ> depthMapCloudFiltered;
    pcl::MedianFilter<pcl::PointXYZ> medianFilter;
    medianFilter.setWindowSize(medianFilterWindowSize);
    medianFilter.setInputCloud(depthMapCloud);
    medianFilter.applyFilter(depthMapCloudFiltered);

    // Convert the filtered organized point cloud back into a depth map.
    for (uint32_t u = 0; u < depthMap.width; u++)
    {
        for (uint32_t v = 0; v < depthMap.height; v++) 
        {
            uint32_t filteredDepth = static_cast<uint32_t>(roundf(depthMapCloudFiltered.at(u, v).z));
            depthMap.setValueAt(u, v, filteredDepth);
        }
    }
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

void DepthDataReceiver::publishPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    // Set the header of the message.
    pointCloudMessage.header.seq = sequenceNumber;
    pointCloudMessage.header.stamp = timestamp;
    pointCloudMessage.header.frame_id = frameId;

    // Publish the message.
    publisher.publish(pointCloudMessage);
}

void DepthDataReceiver::publishHololensPosition(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
    const ros::Time& timestamp)
{
    // Create the ROS message for the current position of the HoloLens.
    geometry_msgs::PointStamped hololensPosition;

    // Set the header of the message.
    hololensPosition.header.seq = positionSequenceNumber;
    hololensPosition.header.stamp = timestamp;
    hololensPosition.header.frame_id = "hololens_world";

    // Add the position of the HoloLens to the message.
    hololensPosition.point.x = depthFrame->camToWorldTranslation.x;
    hololensPosition.point.y = depthFrame->camToWorldTranslation.y;
    hololensPosition.point.z = depthFrame->camToWorldTranslation.z;

    // Publish the message.
    hololensPositionPublisher.publish(hololensPosition);
    positionSequenceNumber++;
}

void DepthDataReceiver::publishHololensCamToWorldTf(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame, 
    const ros::Time& timestamp)
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
    hololensCamPublisher.sendTransform(tf::StampedTransform(transform, timestamp, "hololens_world", "hololens_cam"));
}

void DepthDataReceiver::publishDepthImage(
    const DepthMap depthMap,
    const hololens_msgs::PixelDirections::ConstPtr& pixelDirections,
    const ros::Publisher& publisher, 
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const float minDepth,
    const float minReliableDepth,
    const float maxReliableDepth,
    const float maxDepth)
{
    // Create the ROS message for the image.
    sensor_msgs::Image image;

    // Set the header of the image.
    image.header.seq = sequenceNumber;
    image.header.stamp = timestamp;
    image.header.frame_id = "hololens_cam";

    // Set the meta data (width, height, encoding, ...) of the image.
    image.height = depthMap.height;
    image.width = depthMap.width;
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = 0;
    image.step = depthMap.width * 3;

    // Initialize the image to be completely black.
    image.data.resize(image.height * image.width * 3, 0);

    // Mark all pixels which have a corresponding pixel direction as red.
    for (uint32_t i = 0; i < pixelDirections->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirections->pixelDirections.at(i);
        uint32_t index = (dir.u * image.width + dir.v) * 3;
        image.data.at(index) = 255;
    }

    // Brighten the image and color all unreliable pixels.
    for (uint32_t v = 0; v < image.height; ++v)
    {
        for (uint32_t u = 0; u < image.width; ++u)
        {
            uint32_t index = (v * image.width + u) * 3;
            bool hasNoPixelDirection = image.data.at(index) == 0;

            // Get the depth of the current pixel.
            uint32_t pixelValue = depthMap.valueAt(u, v);
            float depth = static_cast<float>(pixelValue) / 1000.0f;

            // Calculate the greyscale value which will be used for the pixel.
            uint8_t greyscaleValue = static_cast<uint8_t>(255.0f * depth / maxDepth);

            // Check if the depth is within a the (reliable) depth range.
            if (depth < minDepth || depth > maxDepth)
            {
                // Depth is outside the depth range. Color the current pixel black.
                image.data.at(index) = 0;
                image.data.at(index + 1) = 0;
                image.data.at(index + 2) = 0;
            }
            else if (hasNoPixelDirection)
            {
                // Pixel doesn't have any pixel direction associated with it. Color it blue.
                image.data.at(index) = 0;
                image.data.at(index + 1) = 0;
                image.data.at(index + 2) = greyscaleValue;
            }
            else if (depth < minReliableDepth)
            {
                // Depth too small to be considered as reliable. Color the current pixel purple.
                image.data.at(index) = greyscaleValue * 3;
                image.data.at(index + 1) = 0;
                image.data.at(index + 2) = greyscaleValue * 3;
            }
            else if (depth > maxReliableDepth)
            {
                // Depth is too high to be considered as reliable. Color the current pixel red.
                image.data.at(index) = greyscaleValue;
                image.data.at(index + 1) = 0;
                image.data.at(index + 2) = 0;
            }
            else
            {
                // Depth is inside the reliable depth range. Color the current pixel grey.
                image.data.at(index) = greyscaleValue;
                image.data.at(index + 1) = greyscaleValue;
                image.data.at(index + 2) = greyscaleValue;
            }
        }
    }
    
    // Publish the message.
    publisher.publish(image);
}
