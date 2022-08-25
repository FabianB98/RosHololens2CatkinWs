#include <cmath>
#include <math.h>
#include <vector>

#include <ros/ros.h>

#include "geometry_msgs/PointStamped.h"
#include "object3d_detector/DetectionResults.h"
#include "visualization_msgs/MarkerArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_2D.h>

#define TRACKING_SYSTEM_MARKER_POSITION_TOPIC "trackingSystemMarkerPosition"
#define OBJECT_DETECTIONS_TOPIC "object3d_detector/detections"

// Set to true if the y axis is the upwards facing axis (i.e. for octomap_based_spatial_mapper).
// Set to false if the z axis is the upwards facing axis (i.e. for object3d_detector).
#define OBJECT_DETECTIONS_COORD_SYSTEM_Y_AXIS_UP false

#define TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES 100
#define TRACKING_SYSTEM_MARKER_STATIONARY_TEST_AVG_DISTANCE_THRESHOLD 0.01 // Unit: meters
#define TRACKING_SYSTEM_MARKER_STATIONARY_TEST_MAX_DISTANCE_THRESHOLD 0.05 // Unit: meters

#define TRACKING_SYSTEM_MARKER_POSITION_MAX_AGE 1 // Unit: seconds

#define TRANSFORMATION_ESTIMATION_NUM_CORRESPONDENCES 4

#define TRACKING_SYSTEM_MARKER_FRAME_ID "odom"

pcl::PointXYZ lastTrackingPosition;
ros::Time lastTrackingTime;
bool trackingSystemMarkerStationary = false;
bool trackingSystemMarkerMovedSinceLastDetection = false;
pcl::PointCloud<pcl::PointXYZ> lastTrackingPositionsCloud;
size_t lastTrackingPositionsCloudInsertionIndex = 0;

bool transformationEstimated = false;
Eigen::Matrix4f transformation;

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ> > associatedPointsTransformationEstimation;
std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ> > associatedPointsStationary;
std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ> > associatedPointsMoving;

ros::Publisher trackingSystemMarkerPositionPublisher;

void trackingSystemMarkerPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Ideally, the marker used for the object tracking system should be located exactly at the center of mass of the
    // tracked human. However, this would be somewhere around the stomach area, where the marker obviously can't be
    // placed at. To overcome this issue, the marker is located above the test subject's head, which means that we can't
    // calculate the tracking error for the height as this would result in wrong numbers. We're therefore only able to
    // determine the tracking error for the plane perpendicular to the upwards facing axis (which is the XY plane in the
    // tracking system's coordinate system).
    lastTrackingPosition = pcl::PointXYZ(msg->point.x, msg->point.y, 0.0);
    lastTrackingTime = msg->header.stamp;

    // Determine whether the marker is somewhat stationary (used to determine the steady-state error).
    pcl::PointXYZ trackingPosition = pcl::PointXYZ(msg->point.x, msg->point.y, msg->point.z);
    if (lastTrackingPositionsCloud.points.size() < TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES) {
        // Not enough measurements to determine whether marker is stationary... Assuming marker to be moving.
        lastTrackingPositionsCloud.points.push_back(trackingPosition);
        trackingSystemMarkerStationary = false;
    } else {
        lastTrackingPositionsCloud.points[lastTrackingPositionsCloudInsertionIndex] = trackingPosition;
        lastTrackingPositionsCloudInsertionIndex
                = (lastTrackingPositionsCloudInsertionIndex + 1) % TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(lastTrackingPositionsCloud, centroid);

        double summedDistanceToCentroid = 0.0;
        double maxDistanceToCentroid = 0.0;
        for (size_t i = 0; i < TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES; i++) {
            const pcl::PointXYZ& point = lastTrackingPositionsCloud.points[i];

            double diffX = point.x - centroid[0];
            double diffY = point.y - centroid[1];
            double diffZ = point.z - centroid[2];
            double distanceToCentroid = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

            summedDistanceToCentroid += distanceToCentroid;
            maxDistanceToCentroid = std::max(maxDistanceToCentroid, distanceToCentroid);
        }

        bool markerStationaryNew =
                summedDistanceToCentroid < TRACKING_SYSTEM_MARKER_STATIONARY_TEST_AVG_DISTANCE_THRESHOLD * TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES
                && maxDistanceToCentroid < TRACKING_SYSTEM_MARKER_STATIONARY_TEST_MAX_DISTANCE_THRESHOLD;

        if (markerStationaryNew && !trackingSystemMarkerStationary) {
            ROS_INFO("Tracking system marker is now stationary.");
        } else if (!markerStationaryNew && trackingSystemMarkerStationary) {
            ROS_INFO("Tracking system marker is no longer stationary.");
        }

        trackingSystemMarkerStationary = markerStationaryNew;

        if (transformationEstimated) {
            Eigen::Affine3f affineTransformation;
            affineTransformation.matrix() = transformation;
            pcl::PointXYZ trackingPositionTransformed = transformPoint(trackingPosition, affineTransformation);

            // We don't have any transformation information regarding the height (see comment further above in this
            // method for more information), so we don't know at which height in the HoloLens’s world coordinate
            // system the marker is located at. We will therefore simply draw a vertical line to indicate the possible
            // locations at which the marker might be at. 
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = TRACKING_SYSTEM_MARKER_FRAME_ID;
            marker.ns = "externalTrackingSystem";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_LIST;

            geometry_msgs::Point upperEnd;
            upperEnd.x = trackingPositionTransformed.x;
            upperEnd.y = trackingPositionTransformed.y;
            upperEnd.z = 2.0;
            marker.points.push_back(upperEnd);

            geometry_msgs::Point lowerEnd;
            lowerEnd.x = trackingPositionTransformed.x;
            lowerEnd.y = trackingPositionTransformed.y;
            lowerEnd.z = -2.0;
            marker.points.push_back(lowerEnd);

            marker.scale.x = 0.02;

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(0.25);
            markerArray.markers.push_back(marker);

            trackingSystemMarkerPositionPublisher.publish(markerArray);
        }
    }
}

void objectDetectionsCallback(const object3d_detector::DetectionResults::ConstPtr& msg) {
    if (lastTrackingPositionsCloud.points.size() < TRACKING_SYSTEM_MARKER_STATIONARY_TEST_NUM_FRAMES) {
        // Not enough measurements to determine whether the tracking system's marker is stationary...
        return;
    }

    const ros::Time& detectionsTime = msg->header.stamp;
    ros::Duration detectionsAndTrackingTimeDifference = detectionsTime - lastTrackingTime;
    if (detectionsAndTrackingTimeDifference.toSec() > TRACKING_SYSTEM_MARKER_POSITION_MAX_AGE) {
        ROS_WARN("Tracking system may have lost sight of the marker! Aborting association of current detections...");
        return;
    }

    for (size_t i = 0; i < msg->detections.size(); i++) {
        const object3d_detector::DetectionResult& detectionResult = msg->detections[i];

        if (detectionResult.classificationResult.objectClass == 1 /* Human */) {
            pcl::PointXYZ detectedCentroid;
            if (OBJECT_DETECTIONS_COORD_SYSTEM_Y_AXIS_UP) {
                detectedCentroid = pcl::PointXYZ(detectionResult.centroid.x, -detectionResult.centroid.z, 0.0);
            } else {
                detectedCentroid = pcl::PointXYZ(detectionResult.centroid.x, detectionResult.centroid.y, 0.0);
            }

            bool correspondenceUsedForTransformationDetection = trackingSystemMarkerStationary
                    && trackingSystemMarkerMovedSinceLastDetection
                    && associatedPointsTransformationEstimation.size() < TRANSFORMATION_ESTIMATION_NUM_CORRESPONDENCES;
            if (correspondenceUsedForTransformationDetection) {
                ROS_INFO("Using current detection as correspondence for transformation estimation.");
                associatedPointsTransformationEstimation.push_back(
                        std::make_pair(detectedCentroid, lastTrackingPosition));
                trackingSystemMarkerMovedSinceLastDetection = false;
                
                if (associatedPointsTransformationEstimation.size() == TRANSFORMATION_ESTIMATION_NUM_CORRESPONDENCES) {
                    ROS_INFO("Estimating transformation between coordinate systems...");

                    pcl::PointCloud<pcl::PointXYZ> detectionPoints;
                    pcl::PointCloud<pcl::PointXYZ> trackingSystemPoints;
                    pcl::Correspondences correspondences;
                    for (size_t i = 0; i < associatedPointsTransformationEstimation.size(); i++) {
                        detectionPoints.push_back(associatedPointsTransformationEstimation[i].first);
                        trackingSystemPoints.push_back(associatedPointsTransformationEstimation[i].second);
                        correspondences.push_back(pcl::Correspondence(i, i, 1.0));
                    }

                    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, float> estimation;
                    estimation.estimateRigidTransformation(
                            trackingSystemPoints, detectionPoints, correspondences, transformation);
                    transformationEstimated = true;

                    ROS_INFO("Transformation estimated! Marker positions will be published from now on.");
                }
            } else {
                if (trackingSystemMarkerStationary) {
                    associatedPointsStationary.push_back(std::make_pair(detectedCentroid, lastTrackingPosition));
                } else {
                    associatedPointsMoving.push_back(std::make_pair(detectedCentroid, lastTrackingPosition));
                }
            }
        }
    }

    if (!trackingSystemMarkerStationary && !trackingSystemMarkerMovedSinceLastDetection) {
        ROS_INFO("Marker moved since last detection.");
        trackingSystemMarkerMovedSinceLastDetection = true;
    }
}

int main(int argc, char** argv) { 
    ros::init(argc, argv, "tracking_accuracy_test");

    ros::NodeHandle nodeHandle = ros::NodeHandle();

    ros::Subscriber trackingSystemMarkerPositionSubscriber
            = nodeHandle.subscribe(TRACKING_SYSTEM_MARKER_POSITION_TOPIC, 10, trackingSystemMarkerPositionCallback);
    ros::Subscriber objectDetectionsSubscriber
            = nodeHandle.subscribe(OBJECT_DETECTIONS_TOPIC, 10, objectDetectionsCallback);

    trackingSystemMarkerPositionPublisher
            = nodeHandle.advertise<visualization_msgs::MarkerArray>("trackingSystemMarkerPositionRegistered", 100);

    ros::spin();

    // TODO: Calculate average tracking error and average steady-state error.
      
    return 0; 
}