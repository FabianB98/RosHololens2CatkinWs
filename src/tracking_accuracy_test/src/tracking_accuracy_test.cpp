#include <ros/ros.h>

#include "geometry_msgs/PointStamped.h"
#include "object3d_detector/DetectionResults.h"

#define TRACKING_SYSTEM_MARKER_POSITION_TOPIC "trackingSystemMarkerPosition"
#define OBJECT_DETECTIONS_TOPIC "object3d_detector/detections"

geometry_msgs::PointStamped lastTrackingPositionPoint;

void trackingSystemMarkerPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    lastTrackingPositionPoint = *msg;

    // TODO: Determine whether the marker is somewhat stationary (used to determine the steady-state error).
}

void objectDetectionsCallback(const object3d_detector::DetectionResults::ConstPtr& msg) {
    for (size_t i = 0; i < msg->detections.size(); i++) {
        const object3d_detector::DetectionResult& detectionResult = msg->detections[i];

        if (detectionResult.classificationResult.objectClass == 1 // Human
                && detectionResult.classificationResult.trackingId >= 0) {
            // TODO: Associate the current object's centroid with the last known point of the external tracking system.

            // TODO: Determine whether the object is somewhat stationary (used to determine steady-state error).
        }
    }
}

int main(int argc, char** argv) { 
    ros::init(argc, argv, "tracking_accuracy_test");

    ros::NodeHandle nodeHandle = ros::NodeHandle();
    ros::Subscriber trackingSystemMarkerPositionSubscriber
            = nodeHandle.subscribe(TRACKING_SYSTEM_MARKER_POSITION_TOPIC, 10, trackingSystemMarkerPositionCallback);
    ros::Subscriber objectDetectionsSubscriber
            = nodeHandle.subscribe(OBJECT_DETECTIONS_TOPIC, 10, objectDetectionsCallback);

    ros::spin();

    // TODO: Find transformation between the two coordinate systems.

    // TODO: Calculate average tracking error and average steady-state error.
      
    return 0; 
}