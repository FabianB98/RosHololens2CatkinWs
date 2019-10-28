 
#include "ros/ros.h"

#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::visualization::CloudViewer viewer("Point cloud");

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointCloudListener");
    
    ROS_INFO("Initializing HoloLens Point Cloud Listener...");
    
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("hololensPointCloud", 1000, pointCloudCallback);
    
    ros::spin();
    
    return 0;
}
