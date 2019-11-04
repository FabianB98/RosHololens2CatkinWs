
#include "ros/ros.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/PixelDirection.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

#include "Base64.h"
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#define SHORT_THROW_MIN_RELIABLE_DEPTH 0.2f
#define SHORT_THROW_MAX_RELIABLE_DEPTH 1.0f

#define LONG_THROW_MIN_RELIABLE_DEPTH 0.5f
#define LONG_THROW_MAX_RELIABLE_DEPTH 4.0f

#define SHORT_THROW_PIXEL_DIRECTIONS_TOPIC "/hololensShortThrowPixelDirections"
#define LONG_THROW_PIXEL_DIRECTIONS_TOPIC "/hololensLongThrowPixelDirections"

#define SHORT_THROW_DEPTH_TOPIC "/hololensShortThrowDepth"
#define LONG_THROW_DEPTH_TOPIC "/hololensLongThrowDepth"

pcl::visualization::CloudViewer viewer("Point cloud");

hololens_point_cloud_msgs::PixelDirections::ConstPtr shortThrowDirections;
hololens_point_cloud_msgs::PixelDirections::ConstPtr longThrowDirections;

void pointCloudCallback(const hololens_point_cloud_msgs::PointCloud::ConstPtr& msg)
{
    ROS_INFO("Received a point cloud!");

    pcl::PointCloud<pcl::PointXYZ>::Ptr shortThrowPointCloud (new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < msg->shortThrowPoints.size(); ++i)
    {
        shortThrowPointCloud->points.push_back(pcl::PointXYZ(msg->shortThrowPoints[i].x, msg->shortThrowPoints[i].y, msg->shortThrowPoints[i].z));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr longThrowPointCloud (new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < msg->longThrowPoints.size(); ++i)
    {
        longThrowPointCloud->points.push_back(pcl::PointXYZ(msg->longThrowPoints[i].x, msg->longThrowPoints[i].y, msg->longThrowPoints[i].z));
    }

    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> shortThrowColor (shortThrowPointCloud, 230, 20, 20);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> longThrowColor (longThrowPointCloud, 255, 255, 255);
    //viewer.removeAllPointClouds();
    //viewer.addPointCloud(shortThrowPointCloud, shortThrowColor, "short_throw_point_cloud");
    //viewer.addPointCloud(longThrowPointCloud, longThrowColor, "long_throw_point_cloud");
    //viewer.addCoordinateSystem(1.0, "cloud", 0);
    //viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "short_throw_point_cloud");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "long_throw_point_cloud");
    //viewer.spinOnce(1000);

    viewer.showCloud(shortThrowPointCloud, "short_throw_point_cloud");
    viewer.showCloud(longThrowPointCloud, "long_throw_point_cloud");
}

void shortThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
}

void longThrowDepthFrameCallback(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    std::string decoded = base64_decode(msg->base64encodedDepthMap);
}

void shortThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received short throw pixel directions!");
    shortThrowDirections = msg;
}

void longThrowPixelDirectionsCallback(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received long throw pixel directions!");
    longThrowDirections = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointCloudListener");

    ROS_INFO("Initializing HoloLens Point Cloud Listener...");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("hololensPointCloud", 1000, pointCloudCallback);

    ros::Subscriber shortThrowDepthSubscriber = n.subscribe(SHORT_THROW_DEPTH_TOPIC, 1000, shortThrowDepthFrameCallback);
    ros::Subscriber longThrowDepthSubscriber = n.subscribe(LONG_THROW_DEPTH_TOPIC, 1000, longThrowDepthFrameCallback);
    ros::Subscriber shortThrowDirectionsSubscriber = n.subscribe(SHORT_THROW_PIXEL_DIRECTIONS_TOPIC, 1000, shortThrowPixelDirectionsCallback);
    ros::Subscriber longThrowDirectionsSubscriber = n.subscribe(LONG_THROW_PIXEL_DIRECTIONS_TOPIC, 1000, longThrowPixelDirectionsCallback);

    ros::spin();

    return 0;
}
