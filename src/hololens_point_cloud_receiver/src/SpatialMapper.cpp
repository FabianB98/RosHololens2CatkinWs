#include "SpatialMapper.h"

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
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

#include <pcl_conversions/pcl_conversions.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                         SWITCHES FOR CHANGING THE DEFAULT BEHAVIOUR OF THE SPATIAL MAPPER                          //
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
// Should the global point cloud be downsampled after the new point cloud was registered to it?
#define DOWNSAMPLE_GLOBAL_CLOUD true

// Switches regarding the registration of new point clouds to the global point cloud.
// Should ICP be used for registering the new point cloud to the global point cloud?
#define USE_ICP false

// Switches indicating which results should be published.
// Should the current position of the HoloLens be published?
#define PUBLISH_POSITION true
// Should the current depth image be published?
#define PUBLISH_DEPTH_IMAGE true
// Should the resulting point cloud be published?
#define PUBLISH_POINT_CLOUD true

// Switches regarding debugging information.
// Should the centroid of new point clouds be calculated and printed to the console?
#define PRINT_CENTROID false
// Should the results of ICP (i.e. convergence, number of iterations and fitness score) be printed to the console?
#define PRINT_ICP_RESULTS true

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Parameters for the voxel grid filter used for downsampling.
#define DOWNSAMPLING_LEAF_SIZE 0.01f

// Parameters for the radius outlier removal filter used for removing outliers.
#define OUTLIER_REMOVAL_RADIUS_SEARCH 0.05
#define OUTLIER_REMOVAL_MIN_NEIGHBORS_IN_RADIUS 9

// Parameters for the statistical outlier removal filter used for removing outliers.
#define OUTLIER_REMOVAL_NEIGHBORS_TO_CHECK 10
#define OUTLIER_REMOVAL_STD_DEVIATION_MULTIPLIER 0.5

// Parameters for ICP used for the registration of new point clouds to the global cloud.
#define ICP_MAX_ITERATIONS 20
#define ICP_TRANSFORMATION_EPSILON 1e-12
#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.1
#define ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD 0.001
#define ICP_EUCLIDEAN_FITNESS_EPSILON 1.0

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
#define LONG_THROW_MIN_RELIABLE_DEPTH 0.85f // The minimum depth value to use. Smaller depth values will be discarded.
#define LONG_THROW_MAX_RELIABLE_DEPTH 3.3f  // The maximum depth value to use. Greater depth values will be discarded.
#define LONG_THROW_MAX_DEPTH 4.0f           // The maximum depth value which the sensor can report.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

SpatialMapper::SpatialMapper(
    ros::NodeHandle n, 
    const std::string shortThrowImageTopic, 
    const std::string longThrowImageTopic,
    const std::string pointCloudTopic,
    const std::string hololensPositionTopic)
{
    ROS_INFO("Creating SpatialMapper...");

    // Initialize all switches to their default value.
    useShortThrow = USE_SHORT_THROW;
    useLongThrow = USE_LONG_THROW;
    downsampleNewCloud = DOWNSAMPLE_NEW_CLOUD;
    removeOutliersRadiusNewCloud = REMOVE_OUTLIERS_RADIUS;
    removeOutliersStatisticalNewCloud = REMOVE_OUTLIERS_STATISTICAL;
    downsampleGlobalCloud = DOWNSAMPLE_GLOBAL_CLOUD;
    useICP = USE_ICP;
    publishCurrentPosition = PUBLISH_POSITION;
    publishCurrentDepthImage = PUBLISH_DEPTH_IMAGE;
    publishGlobalPointCloud = PUBLISH_POINT_CLOUD;
    printCentroid = PRINT_CENTROID;
    printICPResults = PRINT_ICP_RESULTS;

    // Initialize all hyper parameters to their default value.
    downsamplingLeafSize = DOWNSAMPLING_LEAF_SIZE;
    outlierRemovalRadiusSearch = OUTLIER_REMOVAL_RADIUS_SEARCH;
    outlierRemovalMinNeighborsInRadius = OUTLIER_REMOVAL_MIN_NEIGHBORS_IN_RADIUS;
    outlierRemovalNeighborsToCheck = OUTLIER_REMOVAL_NEIGHBORS_TO_CHECK;
    outlierRemovalStdDeviationMultiplier = OUTLIER_REMOVAL_STD_DEVIATION_MULTIPLIER;
    icpMaxIterations = ICP_MAX_ITERATIONS;
    icpTransformationEpsilon = ICP_TRANSFORMATION_EPSILON;
    icpMaxCorrespondenceDistance = ICP_MAX_CORRESPONDENCE_DISTANCE;
    icpRansacOutlierRejectionThreshold = ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD;
    icpEuclideanFitnessEpsilon = ICP_EUCLIDEAN_FITNESS_EPSILON;

    // Initialize all sensor intrinsics to their default value.
    shortThrowMinDepth = SHORT_THROW_MIN_DEPTH;
    shortThrowMinReliableDepth = SHORT_THROW_MIN_RELIABLE_DEPTH;
    shortThrowMaxReliableDepth = SHORT_THROW_MAX_RELIABLE_DEPTH;
    shortThrowMaxDepth = SHORT_THROW_MAX_DEPTH;
    longThrowMinDepth = LONG_THROW_MIN_DEPTH;
    longThrowMinReliableDepth = LONG_THROW_MIN_RELIABLE_DEPTH;
    longThrowMaxReliableDepth = LONG_THROW_MAX_RELIABLE_DEPTH;
    longThrowMaxDepth = LONG_THROW_MAX_DEPTH;

    // Initialize the point cloud.
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Advertise the topics to which the raw depth images (short throw & long throw) will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(shortThrowImageTopic, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(longThrowImageTopic, 10);
    pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(hololensPositionTopic, 10);

    // Advertise additional topics for debugging information. As these publishers are intended for debugging purposes,
    // it is very likely that they'll eventually be removed again.
    additionalPublishers.push_back(n.advertise<sensor_msgs::PointCloud2>("/pointCloudPlanes", 10));
    additionalPublishers.push_back(n.advertise<sensor_msgs::PointCloud2>("/pointCloudRemainder", 10));
}

void SpatialMapper::handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    if (useShortThrow)
        handleDepthFrame(msg, shortThrowDirections, shortThrowMinDepth, shortThrowMinReliableDepth,
                shortThrowMaxReliableDepth, shortThrowMaxDepth, shortThrowImagePublisher, &shortThrowSequenceNumber);
}

void SpatialMapper::handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    if (useLongThrow)
        handleDepthFrame(msg, longThrowDirections, longThrowMinDepth, longThrowMinReliableDepth, 
                longThrowMaxReliableDepth, longThrowMaxDepth, longThrowImagePublisher, &longThrowSequenceNumber);
}

void SpatialMapper::handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    shortThrowDirections = msg;
}

void SpatialMapper::handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    longThrowDirections = msg;
}

void SpatialMapper::handleDepthFrame(
    const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame, 
    const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
    const float minDepth,
    const float minReliableDepth,
    const float maxReliableDepth,
    const float maxDepth,
    const ros::Publisher& imagePublisher,
    uint32_t* sequenceNumber)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight, 
            depthFrame->depthMapPixelStride, false);

    // Publish the depth map as well as the current position of the HoloLens.
    if (publishCurrentPosition)
    {
        publishHololensPosition(depthFrame);
        publishHololensCamToWorldTf(depthFrame);
    }
    if (publishCurrentDepthImage)
        publishDepthImage(depthMap, imagePublisher, sequenceNumber, minDepth, minReliableDepth, maxReliableDepth, 
                maxDepth);

    // Calculate the point cloud (in camera space).
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace = computePointCloudFromDepthMap(depthMap, pixelDirections,
            minReliableDepth, maxReliableDepth);
    
    // Assert that there is at least one point in the calculated point cloud.
    if (pointCloudCamSpace->size() == 0)
        return;

    // Downsample the point cloud to ensure that the point density is not too high.
    if (downsampleNewCloud)
        pointCloudCamSpace = downsamplePointCloud(pointCloudCamSpace, downsamplingLeafSize);

    // Remove outliers (measurement errors, extremely noisy values, ...) from the point cloud.
    if (removeOutliersRadiusNewCloud)
        pointCloudCamSpace = removeOutliersRadius(pointCloudCamSpace, outlierRemovalRadiusSearch, 
                outlierRemovalMinNeighborsInRadius);
    if (removeOutliersStatisticalNewCloud)
        pointCloudCamSpace = removeOutliersStatistical(pointCloudCamSpace, outlierRemovalNeighborsToCheck, 
                outlierRemovalStdDeviationMultiplier);

    // Assert that there is at least one point in the filtered point cloud.
    if (pointCloudCamSpace->size() == 0)
        return;

    // Calculate the transformation from camera space to world space and register the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromDepthFrame(depthFrame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr combinedPointCloud = registerPointCloud(pointCloudCamSpace, camToWorld);

    // TODO: Remove the following line of code later on...
    ROS_INFO("Total point cloud consists of %zu points.", combinedPointCloud->size());

    // Publish the point cloud.
    if (publishGlobalPointCloud)
        publishPointCloud(combinedPointCloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::computePointCloudFromDepthMap(
    const DepthMap depthMap, 
    const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
    const float minReliableDepth,
    const float maxReliableDepth)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZ>());
    
    // Iterate over each pixel of the depth frame.
    for (uint32_t i = 0; i < pixelDirections->pixelDirections.size(); ++i)
    {
        // Get the depth at the current pixel.
        hololens_point_cloud_msgs::PixelDirection dir = pixelDirections->pixelDirections.at(i);
        uint32_t pixelValue = depthMap.valueAt(dir.u, dir.v);
        float depth = static_cast<float>(pixelValue) / 1000.0f;

        // Skip pixels whose depth value are not within the reliable depth range.
        if (depth < minReliableDepth || depth > maxReliableDepth)
            continue;

        // Calculate the point for the current pixel based on the pixels depth value.
        pointCloudCamSpace->push_back(
                pcl::PointXYZ(-dir.direction.x * depth, -dir.direction.y * depth, -dir.direction.z * depth));
    }

    // Return the calculated point cloud.
    return pointCloudCamSpace;
}

Eigen::Matrix4f SpatialMapper::computeCamToWorldFromDepthFrame(
    const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::downsamplePointCloud(
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::removeOutliersRadius(
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::removeOutliersStatistical(
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::registerPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace,
    Eigen::Matrix4f camToWorld)
{
    // Lock the point cloud mutex as we're about to start the registration of the new point cloud.
    pointCloudMutex.lock();

    // Transform the point cloud from camera space to world space.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    if (pointCloud->size() != 0 && useICP) 
    {
        // There exists some part of the global point cloud, so we need to align the given point cloud. Use ICP to
        // align the given point cloud to the global point cloud.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Set the parameters for ICP.
        icp.setMaximumIterations(1);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
        icp.setRANSACOutlierRejectionThreshold(icpRansacOutlierRejectionThreshold);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        // Set source (i.e. the given point cloud) and target (i.e. the global point cloud) point clouds for ICP.
        icp.setInputSource(pointCloudCamSpace);
        icp.setInputTarget(pointCloud);

        // Align the new point cloud using the camera to world transformation as an initial guess.
        icp.align(*pointCloudWorldSpace, camToWorld);

        unsigned int iteration = 1;
        while (!icp.hasConverged() && iteration < icpMaxIterations)
        {
            icp.align(*pointCloudWorldSpace, icp.getFinalTransformation());
            iteration++;
        }

        // Print information about the results of ICP.
        if (printICPResults)
        {
            ROS_INFO("Has converged? %s", icp.hasConverged() ? "true" : "false");
            ROS_INFO("Num iterations: %u", iteration);
            ROS_INFO("Fitness score: %f", icp.getFitnessScore());
        }
    }
    else 
    {
        // There is no global point cloud yet, so we don't need to align the given point cloud. Simply transform the
        // point cloud from camera space to world space.
        pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);
    }

    // Calculate the centroid of the point cloud from the new frame.
    if (printCentroid)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pointCloudWorldSpace, centroid);
        ROS_INFO("Centroid is located at (%f, %f, %f)", centroid(0), centroid(1), centroid(2));
    }

    // Concatenate the point clouds (i.e. add the new point cloud calculated from the new depth frame to the point cloud
    // calculated from the previous frames).
    *pointCloud += *pointCloudWorldSpace;

    // Downsample the concatenated point cloud to avoid a point density which is higher than what is needed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pointCloud;
    if (downsampleGlobalCloud) 
    {
        cloud = downsamplePointCloud(pointCloud, downsamplingLeafSize);
        pointCloud = cloud;
    }

    // Unlock the point cloud mutex as we're done with the registration of the new point cloud.
    pointCloudMutex.unlock();

    // Return the downsampled concatenated point cloud.
    return cloud;
}

void SpatialMapper::detectPlanes()
{
    ROS_INFO("Detecting planes in the point cloud...");

    // Lock the point cloud mutex as we're about to detect planes in the global point cloud.
    pointCloudMutex.lock();

    // Create the objects for saving the determined plane coefficients and the inliers.
    pcl::ModelCoefficients::Ptr coefficientsHorizontal (new pcl::ModelCoefficients());
    pcl::ModelCoefficients::Ptr coefficientsVertical (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersHorizontal (new pcl::PointIndices());
    pcl::PointIndices::Ptr inliersVertical (new pcl::PointIndices());

    // Set up the parameters for RANSAC.
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setAxis(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    seg.setEpsAngle(pcl::deg2rad(1.0));
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.04);
    seg.setOptimizeCoefficients(true);

    // Create an indices extractor for removing the detect planes from the global point cloud.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pointCloud;

    // Create a point cloud for the detect planes.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planes (new pcl::PointCloud<pcl::PointXYZ>());

    // Iteratively use RANSAC to find the next plane until a stop condition is met.
    int i = 0;
    std::size_t numPoints = cloud->points.size();
    while (cloud->points.size() >= 0.2 * numPoints)
    {
        // Use the current remainder of the global point cloud as input cloud for RANSAC.
        seg.setInputCloud(cloud);

        // Try to find a horizontal plane candidate.
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.segment(*inliersHorizontal, *coefficientsHorizontal);

        // Try to find a vertical plane candidate.
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.segment(*inliersVertical, *coefficientsVertical);

        // Determine whether the plane candidates both contain too less points. If that's the case, stop the loop.
        std::size_t numHorizontal = inliersHorizontal->indices.size();
        std::size_t numVertical = inliersVertical->indices.size();
        std::size_t maxInliers = std::max(numHorizontal, numVertical);
        if (maxInliers <= 0.02 * numPoints || maxInliers <= 25000)
            break;
        
        // Determine which of both plane candidates has the most points and use that plane for point extraction.
        extract.setInputCloud(cloud);
        extract.setIndices(numHorizontal >= numVertical ? inliersHorizontal : inliersVertical);

        // Extract the detected plane from the point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>());
        extract.setNegative(false);
        extract.filter(*plane);

        // Extract the remainder (everything except the detected plane) from the point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr remainder (new pcl::PointCloud<pcl::PointXYZ>());
        extract.setNegative(true);
        extract.filter(*remainder);

        ROS_INFO("Detected a plane consisting of %zu points!", plane->points.size());

        // Extract the clusters of the plane. This is done in order to avoid having points inside the plane which do
        // not belong to the object which should be represented by the plane.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(plane);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterExtraction;
        clusterExtraction.setClusterTolerance(0.03);
        clusterExtraction.setMinClusterSize(1000);
        clusterExtraction.setSearchMethod(tree);
        clusterExtraction.setInputCloud(plane);
        clusterExtraction.extract(clusters);

        ROS_INFO("Detected %zu clusters in the detected plane!", clusters.size());

        // Iterate over all found clusters.
        for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
        {
            ROS_INFO("Cluster contains of %zu points!", iter->indices.size());

            pcl::PointIndices::Ptr indices (new pcl::PointIndices());
            std::copy(iter->indices.begin(), iter->indices.end(), std::back_inserter(indices->indices));

            // Create a point cloud representing the current cluster.
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>());
            extract.setInputCloud(plane);
            extract.setIndices(indices);
            extract.setNegative(false);
            extract.filter(*cluster);

            // TODO: Do something with the cluster.
        }

        // Add the current plane to the detected planes.
        *planes += *plane;
        publishPointCloud(planes, &additionalPublishers[0]);
        publishPointCloud(remainder, &additionalPublishers[1]);
        
        // Use the current remainder as the input for the next iteration.
        cloud = remainder;
        ++i;
    }

    // Can't find any more planes. Therefore, we can unlock the point cloud mutex.
    pointCloudMutex.unlock();

    ROS_INFO("Detected %i planes!", i);
}

void SpatialMapper::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher* publisher)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    // Set the header of the message.
    pointCloudMessage.header.seq = pointCloudSequenceNumber++;
    pointCloudMessage.header.stamp = ros::Time::now();
    pointCloudMessage.header.frame_id = "hololens_world";

    // Publish the message.
    if (publisher != NULL)
        publisher->publish(pointCloudMessage);
    else
        pointCloudPublisher.publish(pointCloudMessage);
}

void SpatialMapper::publishHololensPosition(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame)
{
    // Create the ROS message for the current position of the HoloLens.
    geometry_msgs::PointStamped hololensPosition;

    // Set the header of the message.
    hololensPosition.header.seq = pointCloudSequenceNumber;
    hololensPosition.header.stamp = ros::Time::now();
    hololensPosition.header.frame_id = "hololens_world";

    // Add the position of the HoloLens to the message.
    hololensPosition.point.x = depthFrame->camToWorldTranslation.x;
    hololensPosition.point.y = depthFrame->camToWorldTranslation.y;
    hololensPosition.point.z = depthFrame->camToWorldTranslation.z;

    // Publish the message.
    hololensPositionPublisher.publish(hololensPosition);
}

void SpatialMapper::publishHololensCamToWorldTf(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame)
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

void SpatialMapper::publishDepthImage(
    const DepthMap depthMap, 
    const ros::Publisher& publisher, 
    uint32_t* sequenceNumber,
    const float minDepth,
    const float minReliableDepth,
    const float maxReliableDepth,
    const float maxDepth)
{
    // Create the ROS message for the image.
    sensor_msgs::Image image;

    // Set the header of the image.
    image.header.seq = (*sequenceNumber)++;
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

void SpatialMapper::clearPointCloud()
{
    ROS_INFO("Clearing point cloud...");

    // Remove all points from the point cloud and publish the cleared point cloud.
    pointCloudMutex.lock();
    pointCloud->clear();
    publishPointCloud(pointCloud);
    pointCloudMutex.unlock();
}

void SpatialMapper::savePointCloud()
{
    ROS_INFO("Saving point cloud...");

    // Create the prefix (file path and file name) of the file.
    std::string home = std::string(getenv("HOME"));
    std::string directory = home + "/spatial_maps/";
    std::string time = boost::lexical_cast<std::string>(ros::Time::now().toNSec());
    std::string prefix = directory + time;

    // Create the directory (if not already done) in which the point cloud will be saved.
    boost::filesystem::create_directories(directory);

    // Save the point cloud in both PCD and PLY file format.
    pointCloudMutex.lock();
    bool notEmpty = pointCloud->size() > 0;
    if (notEmpty)
    {
        pcl::io::savePCDFileASCII(prefix + ".pcd", *pointCloud);
        pcl::io::savePLYFileASCII(prefix + ".ply", *pointCloud);
    }
    pointCloudMutex.unlock();

    if (notEmpty)
        ROS_INFO("Saved point cloud!");
    else
        ROS_INFO("Couldn't save point cloud as it doesn't contain any points!");
}
