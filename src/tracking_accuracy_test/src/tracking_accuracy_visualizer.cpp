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
#include <pcl/common/distances.h>
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
ros::Publisher trajectoryTrackingSystemPublisher;
ros::Publisher trajectoryObjectDetectionsPublisher;
ros::Publisher trajectoryDifferencesPublisher;

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
    trajectoryTrackingSystemPublisher = publicNodeHandle.advertise<visualization_msgs::MarkerArray>("trajectoryTrackingSystem", 100);
    trajectoryObjectDetectionsPublisher = publicNodeHandle.advertise<visualization_msgs::MarkerArray>("trajectoryObjectDetections", 100);
    trajectoryDifferencesPublisher = publicNodeHandle.advertise<visualization_msgs::MarkerArray>("trajectoryDifferences", 100);

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

    visualization_msgs::MarkerArray trajectoryTrackingSystemMarkerArray;
    visualization_msgs::Marker trajectoryTrackingSystemMarker;
    trajectoryTrackingSystemMarker.header.stamp = ros::Time::now();
    trajectoryTrackingSystemMarker.header.frame_id = "map";
    trajectoryTrackingSystemMarker.ns = "trajectory";
    trajectoryTrackingSystemMarker.id = 0;
    trajectoryTrackingSystemMarker.type = visualization_msgs::Marker::LINE_STRIP;
    for (size_t i = 0; i < allTrackingSystemCloud.points.size(); i++) {
        geometry_msgs::Point point;
        point.x = allTrackingSystemCloud.points[i].x;
        point.y = allTrackingSystemCloud.points[i].y;
        point.z = allTrackingSystemCloud.points[i].z;
        trajectoryTrackingSystemMarker.points.push_back(point);
    }
    trajectoryTrackingSystemMarker.scale.x = 0.005;
    trajectoryTrackingSystemMarker.color.r = 1.0;
    trajectoryTrackingSystemMarker.color.g = 0.5;
    trajectoryTrackingSystemMarker.color.b = 0.0;
    trajectoryTrackingSystemMarker.color.a = 1.0;
    trajectoryTrackingSystemMarker.lifetime = ros::Duration(0.25);
    trajectoryTrackingSystemMarkerArray.markers.push_back(trajectoryTrackingSystemMarker);

    visualization_msgs::MarkerArray trajectoryObjectDetectionsMarkerArray;
    visualization_msgs::Marker trajectoryObjectDetectionsMarker;
    trajectoryObjectDetectionsMarker.header.stamp = ros::Time::now();
    trajectoryObjectDetectionsMarker.header.frame_id = "map";
    trajectoryObjectDetectionsMarker.ns = "trajectory";
    trajectoryObjectDetectionsMarker.id = 0;
    trajectoryObjectDetectionsMarker.type = visualization_msgs::Marker::LINE_STRIP;
    for (size_t i = 0; i < allObjectDetectionsCloud.points.size(); i++) {
        geometry_msgs::Point point;
        point.x = allObjectDetectionsCloud.points[i].x;
        point.y = allObjectDetectionsCloud.points[i].y;
        point.z = allObjectDetectionsCloud.points[i].z;
        trajectoryObjectDetectionsMarker.points.push_back(point);
    }
    trajectoryObjectDetectionsMarker.scale.x = 0.005;
    trajectoryObjectDetectionsMarker.color.r = 0.0;
    trajectoryObjectDetectionsMarker.color.g = 1.0;
    trajectoryObjectDetectionsMarker.color.b = 1.0;
    trajectoryObjectDetectionsMarker.color.a = 1.0;
    trajectoryObjectDetectionsMarker.lifetime = ros::Duration(0.25);
    trajectoryObjectDetectionsMarkerArray.markers.push_back(trajectoryObjectDetectionsMarker);

    visualization_msgs::MarkerArray trajectoryDifferencesMarkerArray;
    visualization_msgs::Marker trajectoryDifferencesMarker;
    trajectoryDifferencesMarker.header.stamp = ros::Time::now();
    trajectoryDifferencesMarker.header.frame_id = "map";
    trajectoryDifferencesMarker.ns = "trajectoryDifference";
    trajectoryDifferencesMarker.id = 0;
    trajectoryDifferencesMarker.type = visualization_msgs::Marker::LINE_LIST;
    float maxDistance = 0.0;
    for (size_t i = 0; i < allTrackingSystemCloud.points.size(); i++) {
        float distance = pcl::euclideanDistance(allTrackingSystemCloud.points[i], allObjectDetectionsCloud.points[i]);
        maxDistance = std::max(maxDistance, distance);
    }
    for (size_t i = 0; i < allTrackingSystemCloud.points.size(); i++) {
        geometry_msgs::Point trackingSystemPoint;
        trackingSystemPoint.x = allTrackingSystemCloud.points[i].x;
        trackingSystemPoint.y = allTrackingSystemCloud.points[i].y;
        trackingSystemPoint.z = allTrackingSystemCloud.points[i].z;
        trajectoryDifferencesMarker.points.push_back(trackingSystemPoint);

        geometry_msgs::Point objectDetectionsPoint;
        objectDetectionsPoint.x = allObjectDetectionsCloud.points[i].x;
        objectDetectionsPoint.y = allObjectDetectionsCloud.points[i].y;
        objectDetectionsPoint.z = allObjectDetectionsCloud.points[i].z;
        trajectoryDifferencesMarker.points.push_back(objectDetectionsPoint);

        float distance = pcl::euclideanDistance(allTrackingSystemCloud.points[i], allObjectDetectionsCloud.points[i]);
        float distanceRelativeToMaxDistance = distance / maxDistance;
        std_msgs::ColorRGBA lineColor;
        lineColor.r = distanceRelativeToMaxDistance;
        lineColor.g = 1.0 - distanceRelativeToMaxDistance;
        lineColor.b = 0.0;
        lineColor.a = 1.0;
        trajectoryDifferencesMarker.colors.push_back(lineColor);
        trajectoryDifferencesMarker.colors.push_back(lineColor);
    }
    trajectoryDifferencesMarker.scale.x = 0.0025;
    trajectoryDifferencesMarker.lifetime = ros::Duration(0.25);
    trajectoryDifferencesMarkerArray.markers.push_back(trajectoryDifferencesMarker);

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
        trajectoryTrackingSystemPublisher.publish(trajectoryTrackingSystemMarkerArray);
        trajectoryObjectDetectionsPublisher.publish(trajectoryObjectDetectionsMarkerArray);
        trajectoryDifferencesPublisher.publish(trajectoryDifferencesMarkerArray);

        rate.sleep();
    }
      
    return 0; 
}