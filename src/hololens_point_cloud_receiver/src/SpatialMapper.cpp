#include "SpatialMapper.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

#include <pcl_conversions/pcl_conversions.h>

SpatialMapper::SpatialMapper(ros::NodeHandle n)
{
    ROS_INFO("Creating SpatialMapper...");

    // Initialize the point cloud.
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Advertise the topics to which the raw depth images (short throw & long throw) will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(SHORT_THROW_IMAGE_TOPIC, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(LONG_THROW_IMAGE_TOPIC, 10);
    pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);
}

void SpatialMapper::handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    // handleDepthFrame(msg, shortThrowDirections, SHORT_THROW_MIN_RELIABLE_DEPTH, SHORT_THROW_MAX_RELIABLE_DEPTH,
    //         shortThrowImagePublisher, &shortThrowSequenceNumber);
}

void SpatialMapper::handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    handleDepthFrame(msg, longThrowDirections, LONG_THROW_MIN_RELIABLE_DEPTH, LONG_THROW_MAX_RELIABLE_DEPTH,
            longThrowImagePublisher, &longThrowSequenceNumber);
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
    const float minReliableDepth,
    const float maxReliableDepth,
    const ros::Publisher& imagePublisher,
    uint32_t* sequenceNumber)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight, depthFrame->depthMapPixelStride, false);

    // Publish the depth map as well as the current position of the HoloLens.
    publishHololensPosition(depthFrame);
    publishHololensCamToWorldTf(depthFrame);
    publishDepthImage(depthMap, imagePublisher, sequenceNumber, maxReliableDepth);

    // Calculate the point cloud (in camera space).
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace = 
        computePointCloudFromDepthMap(depthMap, pixelDirections, minReliableDepth, maxReliableDepth);
    
    // Assert that there is at least one point in the calculated point cloud.
    if (pointCloudCamSpace->size() == 0)
        return;

    // Downsample the point cloud and remove outliers.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpaceDownsampled = downsamplePointCloud(pointCloudCamSpace, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpaceFiltered = removeOutliers(pointCloudCamSpaceDownsampled, 10, 1.0);

    // Assert that there is at least one point in the filtered point cloud.
    if (pointCloudCamSpaceFiltered->size() == 0)
        return;

    // Calculate the transformation from camera space to world space and register the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromDepthFrame(depthFrame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr combinedPointCloud = registerPointCloud(pointCloudCamSpaceFiltered, camToWorld);

    // TODO: Remove the following line of code later on...
    ROS_INFO("Total point cloud consists of %zu points.", combinedPointCloud->size());

    // Publish the point cloud.
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
        pointCloudCamSpace->push_back(pcl::PointXYZ(-dir.direction.x * depth, -dir.direction.y * depth, -dir.direction.z * depth));
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
    const float numNeighborsToCheck,
    const float standardDeviationMultiplier)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>());

    // Remove outliers by using a statistical outlier removal.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(pointCloudToFilter);
    statisticalOutlierRemoval.setMeanK(50);
    statisticalOutlierRemoval.setStddevMulThresh(0.2);
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
    if (pointCloud->size() != 0) 
    {
        // There exists some part of the global point cloud, so we need to align the given point cloud. Use ICP to
        // align the given point cloud to the global point cloud.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Set the parameters for ICP.
        icp.setMaximumIterations(1);
        icp.setTransformationEpsilon(1e-12);
        icp.setMaxCorrespondenceDistance(0.1);
        icp.setRANSACOutlierRejectionThreshold(0.001);
        icp.setEuclideanFitnessEpsilon(1);

        // Set source (i.e. the given point cloud) and target (i.e. the global point cloud) point clouds for ICP.
        icp.setInputSource(pointCloudCamSpace);
        icp.setInputTarget(pointCloud);

        // Align the new point cloud using the camera to world transformation as an initial guess.
        icp.align(*pointCloudWorldSpace, camToWorld);

        unsigned int iteration = 1;
        while (!icp.hasConverged() && iteration < 20)
        {
            icp.align(*pointCloudWorldSpace, icp.getFinalTransformation());
            iteration++;
        }

        ROS_INFO("Has converged? %s", icp.hasConverged() ? "true" : "false");
        ROS_INFO("Num iterations: %u", iteration);
        ROS_INFO("Fitness score: %f", icp.getFitnessScore());
    }
    else 
    {
        // There is no global point cloud yet, so we don't need to align the given point cloud. Simply transform the
        // point cloud from camera space to world space.
        pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);
    }

    // Calculate the centroid of the point cloud from the new frame.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pointCloudWorldSpace, centroid);
    ROS_INFO("Centroid is located at (%f, %f, %f)", centroid(0), centroid(1), centroid(2));

    // Concatenate the point clouds (i.e. add the new point cloud calculated from the new depth frame to the point cloud
    // calculated from the previous frames).
    *pointCloud += *pointCloudWorldSpace;

    // Downsample the concatenated point cloud to avoid a point density which is higher than what is needed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledPointCloud = downsamplePointCloud(pointCloud, 0.01f);
    pointCloud = downsampledPointCloud;

    // Unlock the point cloud mutex as we're done with the registration of the new point cloud.
    pointCloudMutex.unlock();

    // Return the downsampled concatenated point cloud.
    return downsampledPointCloud;
}

void SpatialMapper::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    // Set the header of the message.
    pointCloudMessage.header.seq = pointCloudSequenceNumber++;
    pointCloudMessage.header.stamp = ros::Time::now();
    pointCloudMessage.header.frame_id = "hololens_world";

    // Publish the message.
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
    const float maxReliableDepth)
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
    image.encoding = sensor_msgs::image_encodings::MONO8;
    image.is_bigendian = 0;
    image.step = depthMap.width;

    // Brighten the image and add it to the message.
    for (uint32_t v = 0; v < image.height; ++v)
        for (uint32_t u = 0; u < image.width; ++u)
            image.data.push_back(static_cast<uint8_t>(255.0f * depthMap.valueAt(u, v) / (maxReliableDepth * 1000.0f)));
    
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
    std::string time = boost::lexical_cast<std::string>(ros::Time::now().toNSec());
    std::string prefix = home + "/spatial_maps/" + time;

    // Save the point cloud in both PCD and PLY file format.
    pointCloudMutex.lock();
    pcl::io::savePCDFileASCII(prefix + ".pcd", *pointCloud);
    pcl::io::savePLYFileASCII(prefix + ".ply", *pointCloud);
    pointCloudMutex.unlock();

    ROS_INFO("Saved point cloud!");
}
