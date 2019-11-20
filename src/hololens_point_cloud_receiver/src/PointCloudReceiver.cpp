#include "PointCloudReceiver.h"
#include "DepthMap.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

PointCloudReceiver::PointCloudReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating PointCloudReceiver...");

    // Initialize the point cloud.
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Advertise the topics to which the raw depth images (short throw & long throw) will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(SHORT_THROW_IMAGE_TOPIC, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(LONG_THROW_IMAGE_TOPIC, 10);
    pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 10);
}

void PointCloudReceiver::handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    handleDepthFrame(msg, shortThrowDirections, SHORT_THROW_MIN_RELIABLE_DEPTH, SHORT_THROW_MAX_RELIABLE_DEPTH,
            shortThrowImagePublisher, &shortThrowSequenceNumber);
}

void PointCloudReceiver::handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    handleDepthFrame(msg, longThrowDirections, LONG_THROW_MIN_RELIABLE_DEPTH, LONG_THROW_MAX_RELIABLE_DEPTH,
            longThrowImagePublisher, &longThrowSequenceNumber);
}

void PointCloudReceiver::handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    shortThrowDirections = msg;
}

void PointCloudReceiver::handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    longThrowDirections = msg;
}

void PointCloudReceiver::handleDepthFrame(
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

    // Publish a brightened version of the depth image for visualization.
    sensor_msgs::Image image;
    image.header.seq = (*sequenceNumber)++;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "hololens";
    image.height = depthFrame->depthMapHeight;
    image.width = depthFrame->depthMapWidth;
    image.encoding = sensor_msgs::image_encodings::MONO8;
    image.is_bigendian = 0;
    image.step = depthFrame->depthMapWidth;
    for (uint32_t v = 0; v < image.height; ++v)
        for (uint32_t u = 0; u < image.width; ++u)
            image.data.push_back(static_cast<uint8_t>(255.0f * depthMap.valueAt(u, v) / (maxReliableDepth * 1000.0f)));
    imagePublisher.publish(image);

    // Calculate the point cloud (in camera space).
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZ>());
    for (uint32_t i = 0; i < pixelDirections->pixelDirections.size(); ++i)
    {
        // Get the depth at the current pixel.
        hololens_point_cloud_msgs::PixelDirection dir = pixelDirections->pixelDirections.at(i);
        uint32_t pixelValue = depthMap.valueAt(dir.u, dir.v);
        float depth = static_cast<float>(pixelValue) / 1000.0f;
        if (depth < minReliableDepth || depth > maxReliableDepth)
            continue;

        // Calculate the point for the current pixel based on the pixels depth value.
        pointCloudCamSpace->push_back(pcl::PointXYZ(-dir.direction.x * depth, -dir.direction.y * depth, -dir.direction.z * depth));
    }

    // Downsample the point cloud to reduce the amount of processing time needed for removing outliers.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpaceDownsampled (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(pointCloudCamSpace);
    voxelGrid.setLeafSize(DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE);
    voxelGrid.filter(*pointCloudCamSpaceDownsampled);

    // TODO: Remove the following line of code later on.
    ROS_INFO("Downsampled %zu points to %zu points.", pointCloudCamSpace->size(), pointCloudCamSpaceDownsampled->size());

    // Remove outliers from the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpaceFiltered (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(pointCloudCamSpaceDownsampled);
    statisticalOutlierRemoval.setMeanK(10);
    statisticalOutlierRemoval.setStddevMulThresh(1.0);
    statisticalOutlierRemoval.filter(*pointCloudCamSpaceFiltered);

    // TODO: Remove the following line of code later on.
    ROS_INFO("Found %zu outliers.", pointCloudCamSpaceDownsampled->size() - pointCloudCamSpaceFiltered->size());

    // Transform the point cloud from camera space to world space.
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0, 0, 3, 3) = Eigen::Quaternionf(
        depthFrame->camToWorldRotation.w, 
        depthFrame->camToWorldRotation.x, 
        depthFrame->camToWorldRotation.y, 
        depthFrame->camToWorldRotation.z
    ).toRotationMatrix().transpose();
    transform(0, 3) = depthFrame->camToWorldTranslation.x;
    transform(1, 3) = depthFrame->camToWorldTranslation.y;
    transform(2, 3) = depthFrame->camToWorldTranslation.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*pointCloudCamSpaceFiltered, *pointCloudWorldSpace, transform);

    // Concatenate the point clouds (i.e. add the new point cloud calculated from the new depth frame to the point cloud
    // calculated from the previous frames).
    pointCloudMutex.lock();
    *pointCloud += *pointCloudWorldSpace;

    // Downsample the concatenated point cloud to avoid a point density which is higher than what is needed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2 (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid2;
    voxelGrid2.setInputCloud(pointCloud);
    voxelGrid2.setLeafSize(DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE, DOWNSAMPLING_LEAF_SIZE);
    voxelGrid2.filter(*pointCloud2);
    pointCloud = pointCloud2;
    pointCloudMutex.unlock();

    ROS_INFO("Total point cloud consists of %zu points.", pointCloud2->size());

    // Publish the point cloud.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*pointCloud2, pointCloudMessage);
    pointCloudMessage.header = image.header;
    pointCloudPublisher.publish(pointCloudMessage);
}

void PointCloudReceiver::clearPointCloud()
{
    ROS_INFO("Clearing point cloud...");

    pointCloudMutex.lock();
    pointCloud->clear();
    pointCloudMutex.unlock();
}

void PointCloudReceiver::savePointCloud()
{
    ROS_INFO("Saving point cloud...");
}
