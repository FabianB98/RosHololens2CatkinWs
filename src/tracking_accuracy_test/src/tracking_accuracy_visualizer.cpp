#include <cmath>
#include <iostream>
#include <math.h>
#include <sstream>
#include <vector>

#include "boost/filesystem/fstream.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include <ros/ros.h>

#include "geometry_msgs/PointStamped.h"
#include "object3d_detector/DetectionResults.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl_conversions/pcl_conversions.h>

std::string resultsPathPrefixRelativeToHome;

ros::Publisher allTrackingSystemPublisher;
ros::Publisher allObjectDetectionsPublisher;
ros::Publisher registrationTrackingSystemPublisher;
ros::Publisher registrationObjectDetectionsPublisher;
ros::Publisher stationaryTrackingSystemPublisher;
ros::Publisher stationaryObjectDetectionsPublisher;
ros::Publisher movingTrackingSystemPublisher;
ros::Publisher movingObjectDetectionsPublisher;

int main(int argc, char** argv) { 
    ros::init(argc, argv, "tracking_accuracy_visualizer");

    ros::NodeHandle publicNodeHandle = ros::NodeHandle();
    ros::NodeHandle privateNodeHandle = ros::NodeHandle("~");

    if (!privateNodeHandle.getParam("resultsPathPrefixRelativeToHome", resultsPathPrefixRelativeToHome)) {
        ROS_ERROR("No value for parameter resultsPathPrefixRelativeToHome found!");
        return 1;
    }
    
    allTrackingSystemPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("allTrackingSystem", 100);
    allObjectDetectionsPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("allObjectDetections", 100);
    registrationTrackingSystemPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("registrationTrackingSystem", 100);
    registrationObjectDetectionsPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("registrationObjectDetections", 100);
    stationaryTrackingSystemPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("stationaryTrackingSystem", 100);
    stationaryObjectDetectionsPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("stationaryObjectDetections", 100);
    movingTrackingSystemPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("movingTrackingSystem", 100);
    movingObjectDetectionsPublisher = publicNodeHandle.advertise<sensor_msgs::PointCloud2>("movingObjectDetections", 100);

    std::string home = std::string(getenv("HOME"));
    std::string directory = home + resultsPathPrefixRelativeToHome;

    // Beware: Ugly code ahead

    pcl::PointCloud<pcl::PointXYZ> allTrackingSystemCloud;
    pcl::PointCloud<pcl::PointXYZ> allObjectDetectionsCloud;
    pcl::PointCloud<pcl::PointXYZ> registrationTrackingSystemCloud;
    pcl::PointCloud<pcl::PointXYZ> registrationObjectDetectionsCloud;
    pcl::PointCloud<pcl::PointXYZ> stationaryTrackingSystemCloud;
    pcl::PointCloud<pcl::PointXYZ> stationaryObjectDetectionsCloud;
    pcl::PointCloud<pcl::PointXYZ> movingTrackingSystemCloud;
    pcl::PointCloud<pcl::PointXYZ> movingObjectDetectionsCloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "all_externalTrackingSystem.pcd", allTrackingSystemCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "all_objectDetection.pcd", allObjectDetectionsCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "registration_externalTrackingSystem.pcd", registrationTrackingSystemCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "registration_objectDetection.pcd", registrationObjectDetectionsCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "stationary_externalTrackingSystem.pcd", stationaryTrackingSystemCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "stationary_objectDetection.pcd", stationaryObjectDetectionsCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "moving_externalTrackingSystem.pcd", movingTrackingSystemCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory + "moving_objectDetection.pcd", movingObjectDetectionsCloud) == -1) {
        ROS_ERROR("Loading point cloud data failed! Is there tracking accuracy information stored in \"%s\"?", directory.c_str());
        return 1;
    }

    sensor_msgs::PointCloud2 allTrackingSystemMsg;
    sensor_msgs::PointCloud2 allObjectDetectionsMsg;
    sensor_msgs::PointCloud2 registrationTrackingSystemMsg;
    sensor_msgs::PointCloud2 registrationObjectDetectionsMsg;
    sensor_msgs::PointCloud2 stationaryTrackingSystemMsg;
    sensor_msgs::PointCloud2 stationaryObjectDetectionsMsg;
    sensor_msgs::PointCloud2 movingTrackingSystemMsg;
    sensor_msgs::PointCloud2 movingObjectDetectionsMsg;

    pcl::toROSMsg(allTrackingSystemCloud, allTrackingSystemMsg);
    pcl::toROSMsg(allObjectDetectionsCloud, allObjectDetectionsMsg);
    pcl::toROSMsg(registrationTrackingSystemCloud, registrationTrackingSystemMsg);
    pcl::toROSMsg(registrationObjectDetectionsCloud, registrationObjectDetectionsMsg);
    pcl::toROSMsg(stationaryTrackingSystemCloud, stationaryTrackingSystemMsg);
    pcl::toROSMsg(stationaryObjectDetectionsCloud, stationaryObjectDetectionsMsg);
    pcl::toROSMsg(movingTrackingSystemCloud, movingTrackingSystemMsg);
    pcl::toROSMsg(movingObjectDetectionsCloud, movingObjectDetectionsMsg);

    allTrackingSystemMsg.header.seq = 0;
    allTrackingSystemMsg.header.stamp = ros::Time::now();
    allTrackingSystemMsg.header.frame_id = "map";

    allObjectDetectionsMsg.header.seq = 0;
    allObjectDetectionsMsg.header.stamp = ros::Time::now();
    allObjectDetectionsMsg.header.frame_id = "map";

    registrationTrackingSystemMsg.header.seq = 0;
    registrationTrackingSystemMsg.header.stamp = ros::Time::now();
    registrationTrackingSystemMsg.header.frame_id = "map";

    registrationObjectDetectionsMsg.header.seq = 0;
    registrationObjectDetectionsMsg.header.stamp = ros::Time::now();
    registrationObjectDetectionsMsg.header.frame_id = "map";

    stationaryTrackingSystemMsg.header.seq = 0;
    stationaryTrackingSystemMsg.header.stamp = ros::Time::now();
    stationaryTrackingSystemMsg.header.frame_id = "map";

    stationaryObjectDetectionsMsg.header.seq = 0;
    stationaryObjectDetectionsMsg.header.stamp = ros::Time::now();
    stationaryObjectDetectionsMsg.header.frame_id = "map";

    movingTrackingSystemMsg.header.seq = 0;
    movingTrackingSystemMsg.header.stamp = ros::Time::now();
    movingTrackingSystemMsg.header.frame_id = "map";

    movingObjectDetectionsMsg.header.seq = 0;
    movingObjectDetectionsMsg.header.stamp = ros::Time::now();
    movingObjectDetectionsMsg.header.frame_id = "map";

    ros::Rate rate(10);
    while (ros::ok()) {
        allTrackingSystemPublisher.publish(allTrackingSystemMsg);
        allObjectDetectionsPublisher.publish(allObjectDetectionsMsg);
        registrationTrackingSystemPublisher.publish(registrationTrackingSystemMsg);
        registrationObjectDetectionsPublisher.publish(registrationObjectDetectionsMsg);
        stationaryTrackingSystemPublisher.publish(stationaryTrackingSystemMsg);
        stationaryObjectDetectionsPublisher.publish(stationaryObjectDetectionsMsg);
        movingTrackingSystemPublisher.publish(movingTrackingSystemMsg);
        movingObjectDetectionsPublisher.publish(movingObjectDetectionsMsg);

        rate.sleep();
    }
      
    return 0; 
}