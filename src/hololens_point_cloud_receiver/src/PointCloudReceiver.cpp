#include "PointCloudReceiver.h"
#include "DepthMap.h"

pcl::visualization::CloudViewer cloudViewer("Point cloud");

PointCloudReceiver::PointCloudReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating PointCloudReceiver...");

    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(SHORT_THROW_IMAGE_TOPIC, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(LONG_THROW_IMAGE_TOPIC, 10);
}

void PointCloudReceiver::handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    handleDepthFrame(msg, shortThrowDirections, SHORT_THROW_MIN_RELIABLE_DEPTH, SHORT_THROW_MAX_RELIABLE_DEPTH,
            shortThrowImagePublisher, &shortThrowSequenceNumber, "short_throw_point_cloud");
}

void PointCloudReceiver::handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    handleDepthFrame(msg, longThrowDirections, LONG_THROW_MIN_RELIABLE_DEPTH, LONG_THROW_MAX_RELIABLE_DEPTH,
            longThrowImagePublisher, &longThrowSequenceNumber, "long_throw_point_cloud");
}

void PointCloudReceiver::handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received short throw pixel directions!");
    shortThrowDirections = msg;
}

void PointCloudReceiver::handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received long throw pixel directions!");
    longThrowDirections = msg;
}

void PointCloudReceiver::handleDepthFrame(
    const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame, 
    const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
    const float minReliableDepth,
    const float maxReliableDepth,
    const ros::Publisher& imagePublisher,
    uint32_t* sequenceNumber,
    const std::string pointCloudName)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight, depthFrame->depthMapPixelStride, false);

    // Publish a brightened version of the depth image for visualization.
    sensor_msgs::Image image;
    image.header.seq = (*sequenceNumber)++;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "";
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

    // Transform the point cloud from camera space to world space.
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = depthFrame->camToWorld.m11;
    transform(1, 0) = depthFrame->camToWorld.m12;
    transform(2, 0) = depthFrame->camToWorld.m13;
    transform(3, 0) = depthFrame->camToWorld.m14;

    transform(0, 1) = depthFrame->camToWorld.m21;
    transform(1, 1) = depthFrame->camToWorld.m22;
    transform(2, 1) = depthFrame->camToWorld.m23;
    transform(3, 1) = depthFrame->camToWorld.m24;

    transform(0, 2) = depthFrame->camToWorld.m31;
    transform(1, 2) = depthFrame->camToWorld.m32;
    transform(2, 2) = depthFrame->camToWorld.m33;
    transform(3, 2) = depthFrame->camToWorld.m34;

    transform(0, 3) = depthFrame->camToWorld.m41;
    transform(1, 3) = depthFrame->camToWorld.m42;
    transform(2, 3) = depthFrame->camToWorld.m43;
    transform(3, 3) = depthFrame->camToWorld.m44;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, transform);

    // Visualize the point cloud.
    if (pointCloudName.size() > 0)
        cloudViewer.showCloud(pointCloudWorldSpace, pointCloudName);
}
