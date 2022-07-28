// This file was not part of the repository published at https://github.com/yzrobot/online_learning. It was added as
// part of my work to be able to use the object tracker from other ROS nodes without the need of having to subscribe to
// the different ROS topics to which people_tracker publishes to.
// Please note that my original intent was to avoid duplicating any code already written in people_tracker.h and
// people_tracker.cpp, but this would have required to make some substantial refactorings to people_tracker for which I
// sadly don't have the time right now. This file will therefore contain some duplicated code...

#include "people_tracker/tracking_service.h"

TrackingService::TrackingService() {
    ros::NodeHandle n;
  
    listener = new tf::TransformListener();

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("base_link", base_link, std::string("base_link"));
    private_node_handle.param("target_frame", target_frame, std::string("base_link"));

    parseParams(private_node_handle);

    tracking_service = private_node_handle.advertiseService("trackClusters", &TrackingService::trackClustersCallback, this);

    ros::spin();
}

void TrackingService::parseParams(ros::NodeHandle n) {
    std::string filter;
    n.getParam("filter_type", filter);
    ROS_INFO_STREAM("Found filter type: " << filter);
    if (filter == "EKF") {
        if (n.hasParam("std_limit")) {
            double stdLimit;
            n.getParam("std_limit", stdLimit);
            ROS_INFO("std_limit: %f ",stdLimit);
            ekf = new SimpleTracking<EKFilter>(stdLimit);
        } else {
            ekf = new SimpleTracking<EKFilter>();
        }
    } else if (filter == "UKF") {
        if (n.hasParam("std_limit")) {
            double stdLimit;
            n.getParam("std_limit", stdLimit);
            ROS_INFO("std_limit: %f ",stdLimit);
            ukf = new SimpleTracking<UKFilter>(stdLimit);
        } else {
            ukf = new SimpleTracking<UKFilter>();
        }
    } else if (filter == "PF") {
        if (n.hasParam("std_limit")) {
            double stdLimit;
            n.getParam("std_limit", stdLimit);
            ROS_INFO("std_limit: %f ",stdLimit);
            pf = new SimpleTracking<PFilter>(stdLimit);
        } else {
            pf = new SimpleTracking<PFilter>();
        }
    } else {
        ROS_FATAL_STREAM("Filter type " << filter << " is not specified. Unable to create the tracker. Please use either EKF, UKF or PF.");
        return;
    }
    
    XmlRpc::XmlRpcValue cv_noise;
    n.getParam("cv_noise_params", cv_noise);
    ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
    if (ekf == NULL) {
        if (ukf == NULL) {
            pf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
        } else {
            ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
        }
    } else {
        ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    }
    ROS_INFO_STREAM("Created " << filter << " based tracker using constant velocity prediction model.");
    
    XmlRpc::XmlRpcValue detectors;
    n.getParam("detectors", detectors);
    ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin(); it != detectors.end(); ++it) {
        ROS_INFO_STREAM("Found detector: " << (std::string)(it->first) << " ==> " << detectors[it->first]);
        try {
            if (ekf == NULL) {
                if (ukf == NULL) {
                    if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
                        int seq_size = detectors[it->first]["seq_size"];
                        pf->addDetectorModel(it->first,
                                detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                                detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                                detectors[it->first]["noise_params"]["x"],
                                detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size, detectors[it->first]["seq_time"]);
                    } else {
                        pf->addDetectorModel(it->first,
                                detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                                detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                                detectors[it->first]["noise_params"]["x"],
                                detectors[it->first]["noise_params"]["y"]);
                    }
                } else {
                    if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
                        int seq_size = detectors[it->first]["seq_size"];
                        ukf->addDetectorModel(it->first,
                                detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                                detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                                detectors[it->first]["noise_params"]["x"],
                                detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size,detectors[it->first]["seq_time"]);
                    } else {
                        ukf->addDetectorModel(it->first,
                                detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                                detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                                detectors[it->first]["noise_params"]["x"],
                                detectors[it->first]["noise_params"]["y"]);
                    }
                }
            } else {
                if (detectors[it->first].hasMember("seq_size") && detectors[it->first].hasMember("seq_time")) {
                    int seq_size = detectors[it->first]["seq_size"];
                    ekf->addDetectorModel(it->first,
                            detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                            detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                            detectors[it->first]["noise_params"]["x"],
                            detectors[it->first]["noise_params"]["y"],(unsigned int) seq_size,detectors[it->first]["seq_time"]);
                } else {
                    ekf->addDetectorModel(it->first,
                            detectors[it->first]["matching_algorithm"] == "NN" ? NN : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA : throw(asso_exception()),
                            detectors[it->first]["observation_model"] == "CARTESIAN" ? CARTESIAN : detectors[it->first]["observation_model"] == "POLAR" ? POLAR : throw(observ_exception()),
                            detectors[it->first]["noise_params"]["x"],
                            detectors[it->first]["noise_params"]["y"]);
                }
            }
        } catch (asso_exception& e) {
            ROS_FATAL_STREAM(""
                    << e.what()
                    << " "
                    << detectors[it->first]["matching_algorithm"]
                    << " is not specified. Unable to add "
                    << (std::string)(it->first)
                    << " to the tracker. Please use either NN or NNJPDA as association algorithms."
                    );
            return;
        } catch (observ_exception& e) {
            ROS_FATAL_STREAM(""
                    << e.what()
                    << " "
                    << detectors[it->first]["observation_model"]
                    << " is not specified. Unable to add "
                    << (std::string)(it->first)
                    << " to the tracker. Please use either CARTESIAN or POLAR as observation models."
                    );
            return;
        }
    }
}

bool TrackingService::trackClustersCallback(bayes_people_tracker::TrackClusters::Request& req, bayes_people_tracker::TrackClusters::Response& res) {
    try {
        std::vector<std::pair<long, geometry_msgs::Pose> > trackedPoints = trackClusters(req.clusterCenterPoints, req.detectorName);

        for (size_t i = 0; i < trackedPoints.size(); i++) {
            const std::pair<long, geometry_msgs::Pose>& trackedPoint = trackedPoints[i];

            res.trackIds.push_back(trackedPoint.first);
            res.trackedPoints.push_back(trackedPoint.second);
        }

        return true;
    } catch (tracking_exception ex) {
        ROS_WARN("Tracking failed: %s", ex.what());
        return false;
    }
}

std::vector<std::pair<long, geometry_msgs::Pose> > TrackingService::trackClusters(const geometry_msgs::PoseArray& clusterCenterPoints, std::string& detector) {
    // Transform everything from the given coordinate systems to the tracking coordinate system.
    geometry_msgs::Pose robotPoseInTargetCoords;
    try {
        // TODO: This may not be needed.
        // // Transform into given target frame. Default /map
        // ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
        // listener->waitForTransform(clusterCenterPoints.header.frame_id, target_frame, clusterCenterPoints.header.stamp, ros::Duration(1.0));

        tf::StampedTransform transform;
        listener->lookupTransform(target_frame, base_link, ros::Time(0), transform);
        robotPoseInTargetCoords.position.x = transform.getOrigin().getX();
        robotPoseInTargetCoords.position.y = transform.getOrigin().getY();
        robotPoseInTargetCoords.position.z = transform.getOrigin().getZ();
        robotPoseInTargetCoords.orientation.x = transform.getRotation().getX();
        robotPoseInTargetCoords.orientation.y = transform.getRotation().getY();
        robotPoseInTargetCoords.orientation.z = transform.getRotation().getZ();
        robotPoseInTargetCoords.orientation.w = transform.getRotation().getW();
    } catch(tf::TransformException ex) {
        ROS_WARN("Failed transform: %s", ex.what());
        throw tracking_exception(ex.what());
    }

    std::vector<geometry_msgs::Point> observationsInTrackingCoordinateSystem;
    for (int i = 0; i < clusterCenterPoints.poses.size(); i++) {
        geometry_msgs::Pose clusterCenterPoint = clusterCenterPoints.poses[i];

        // Create stamped pose for tf
        geometry_msgs::PoseStamped poseInCamCoords;
        geometry_msgs::PoseStamped poseInTargetCoords;
        poseInCamCoords.header = clusterCenterPoints.header;
        poseInCamCoords.pose = clusterCenterPoint;
        
        // Transform
        try {
            listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, clusterCenterPoints.header.frame_id, poseInTargetCoords);
        } catch(tf::TransformException ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            throw tracking_exception(ex.what());
        }
        
        observationsInTrackingCoordinateSystem.push_back(poseInTargetCoords.pose.position);
    }

    // Add observations to the tracker and perform tracking.
    std::map<long, std::vector<geometry_msgs::Pose> > trackedPointsInTrackingCoordinateSystem;
    if(ekf == NULL) {
        if(ukf == NULL) {
            pf->addObservation(detector, observationsInTrackingCoordinateSystem, clusterCenterPoints.header.stamp.toSec(), robotPoseInTargetCoords);
            trackedPointsInTrackingCoordinateSystem = pf->track();
        } else {
            ukf->addObservation(detector, observationsInTrackingCoordinateSystem, clusterCenterPoints.header.stamp.toSec(), robotPoseInTargetCoords);
            trackedPointsInTrackingCoordinateSystem = ukf->track();
        }
    } else {
        ekf->addObservation(detector, observationsInTrackingCoordinateSystem, clusterCenterPoints.header.stamp.toSec(), robotPoseInTargetCoords);
        trackedPointsInTrackingCoordinateSystem = ekf->track();
    }

    // Transform everything back from the tracking coordinate system to the given coordinate system.
    std::vector<std::pair<long, geometry_msgs::Pose> > trackedPoints;
    for(std::map<long, std::vector<geometry_msgs::Pose> >::const_iterator it = trackedPointsInTrackingCoordinateSystem.begin(); it != trackedPointsInTrackingCoordinateSystem.end(); ++it) {
        long trackingId = it->first;
        const geometry_msgs::Pose& trackedPointInTrackingCoordinateSystem = it->second[0];

        // Create stamped pose for tf
        geometry_msgs::PoseStamped poseInCamCoords;
        geometry_msgs::PoseStamped poseInTargetCoords;
        poseInTargetCoords.header = clusterCenterPoints.header;
        poseInTargetCoords.header.frame_id = target_frame;
        poseInTargetCoords.pose = trackedPointInTrackingCoordinateSystem;

        // Transform
        try {
            listener->transformPose(clusterCenterPoints.header.frame_id, ros::Time(0), poseInTargetCoords, target_frame, poseInCamCoords);
        } catch(tf::TransformException ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            throw tracking_exception(ex.what());
        }

        trackedPoints.push_back(std::make_pair(trackingId, poseInCamCoords.pose));
    }

    return trackedPoints;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bayes_people_tracker");
    TrackingService* t = new TrackingService();
    return 0;
}
