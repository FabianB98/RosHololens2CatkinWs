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

// I don't understand why, but as soon as I try to use any PCL filter or algorithm (such as the VoxelGrid used for
// downsampling for example) inside a templated function I get a ton of compiler errors stating that the filter either
// is no member of pcl or is not templated (even though the PCL documentation literally states VoxelGrid to be templated
// as VoxelGrid<PointT>...). These errors don't make any sense at all and I don't know how to resolve them, so I'll just
// just have to resort to duplicating these functions...
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

pcl::PointCloud<pcl::PointXYZI>::Ptr DepthDataReceiver::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToDownsample,
    const float leafSize)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudDownsampled (new pcl::PointCloud<pcl::PointXYZI>());

    // Downsample the given point cloud using a voxel grid filter.
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataReceiver::removeOutliersClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
    const double clusterTolerance,
    const int minClusterSize)
{
    // Set up a KD tree for searching inside the cloud.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(pointCloudToFilter);
    
    // Create a vector for storing the detected point indices of each cluster.
    std::vector<pcl::PointIndices> clusters;

    // Extract the clusters of the point cloud.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance);
    clusterExtraction.setSearchMethod(tree);
    clusterExtraction.setInputCloud(pointCloudToFilter);
    clusterExtraction.extract(clusters);

    // Create an indices extractor for removing each cluster from the cloud.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointCloudToFilter);

    // Iterate over all found clusters and add all big enough clusters to the filtered point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud (new pcl::PointCloud<pcl::PointXYZ>());
    for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
    {
        if (iter->indices.size() >= minClusterSize)
        {
            for (const auto& idx : iter->indices)
            {
                filteredPointCloud->push_back((*pointCloudToFilter)[idx]);
            }
        }
    }

    return filteredPointCloud;
}

void DepthDataReceiver::pointCloudToMsg(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    sensor_msgs::PointCloud2& message,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    pcl::toROSMsg(*cloud, message);

    // Set the header of the message.
    message.header.seq = sequenceNumber;
    message.header.stamp = timestamp;
    message.header.frame_id = frameId;
}

void DepthDataReceiver::pointCloudToMsg(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    sensor_msgs::PointCloud2& message,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    pcl::toROSMsg(*cloud, message);

    // Set the header of the message.
    message.header.seq = sequenceNumber;
    message.header.stamp = timestamp;
    message.header.frame_id = frameId;
}

void DepthDataReceiver::publishPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    sensor_msgs::PointCloud2 pointCloudMessage;
    pointCloudToMsg(cloud, pointCloudMessage, sequenceNumber, timestamp, frameId);

    publisher.publish(pointCloudMessage);
}

void DepthDataReceiver::publishPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    sensor_msgs::PointCloud2 pointCloudMessage;
    pointCloudToMsg(cloud, pointCloudMessage, sequenceNumber, timestamp, frameId);

    publisher.publish(pointCloudMessage);
}

void DepthDataReceiver::publishPointCloudFrame(
    const ros::Publisher& publisher,
    sensor_msgs::PointCloud2& pointCloudWorldSpaceMsg,
    sensor_msgs::PointCloud2& artificialEndpointsWorldSpaceMsg,
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
    const float maxReliableDepth)
{
    hololens_depth_data_receiver_msgs::PointCloudFrame pointCloudFrame;

    pointCloudFrame.hololensPosition.header = pointCloudWorldSpaceMsg.header;
    pointCloudFrame.hololensPosition.point.x = depthFrame->camToWorldTranslation.x;
    pointCloudFrame.hololensPosition.point.y = depthFrame->camToWorldTranslation.y;
    pointCloudFrame.hololensPosition.point.z = depthFrame->camToWorldTranslation.z;

    pointCloudFrame.pointCloudWorldSpace = pointCloudWorldSpaceMsg;
    pointCloudFrame.artificialEndpointsWorldSpace = artificialEndpointsWorldSpaceMsg;

    pointCloudFrame.maxReliableDepth = maxReliableDepth;

    publisher.publish(pointCloudFrame);
}

void DepthDataReceiver::publishHololensPosition(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
    const ros::Publisher& publisher, 
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    // Create the ROS message for the current position of the HoloLens.
    geometry_msgs::PointStamped hololensPosition;

    // Set the header of the message.
    hololensPosition.header.seq = sequenceNumber;
    hololensPosition.header.stamp = timestamp;
    hololensPosition.header.frame_id = "hololens_world";

    // Add the position of the HoloLens to the message.
    hololensPosition.point.x = depthFrame->camToWorldTranslation.x;
    hololensPosition.point.y = depthFrame->camToWorldTranslation.y;
    hololensPosition.point.z = depthFrame->camToWorldTranslation.z;

    // Publish the message.
    publisher.publish(hololensPosition);
}

void DepthDataReceiver::publishHololensCamToWorldTf(
    const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
    tf::TransformBroadcaster& publisher,
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
    publisher.sendTransform(tf::StampedTransform(transform, timestamp, "hololens_world", "hololens_cam"));
}
