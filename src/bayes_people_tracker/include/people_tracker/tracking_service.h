// This file was not part of the repository published at https://github.com/yzrobot/online_learning. It was added as
// part of my work to be able to use the object tracker from other ROS nodes without the need of having to subscribe to
// the different ROS topics to which people_tracker publishes to.
// Please note that my original intent was to avoid duplicating any code already written in people_tracker.h and
// people_tracker.cpp, but this would have required to make some substantial refactorings to people_tracker for which I
// sadly don't have the time right now. This file will therefore contain some duplicated code...

#ifndef TRACKING_SERVICE_H
#define TRACKING_SERVICE_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bayes_tracking/BayesFilter/bayesFlt.hpp>

#include <XmlRpcValue.h>

#include <sstream>
#include <string.h>
#include <vector>
#include <math.h>

#include "bayes_people_tracker/PeopleTracker.h"
#include "bayes_people_tracker/TrackClusters.h"

#include "people_tracker/flobot_tracking.h"
#include "people_tracker/asso_exception.h"

#define INVALID_ID -1

class TrackingService
{
public:
    TrackingService();

    bool trackClustersCallback(bayes_people_tracker::TrackClusters::Request& req, bayes_people_tracker::TrackClusters::Response& res);
    std::vector<std::pair<long, geometry_msgs::Pose> > trackClusters(const geometry_msgs::PoseArray& clusterCenterPoints, std::string& detector);
    
private:
    void parseParams(ros::NodeHandle n);

    ros::ServiceServer tracking_service;
    tf::TransformListener* listener;
    std::string target_frame;
    std::string base_link;
    
    SimpleTracking<EKFilter> *ekf = NULL;
    SimpleTracking<UKFilter> *ukf = NULL;
    SimpleTracking<PFilter> *pf = NULL;
};

struct tracking_exception: public exception
{
public:
    tracking_exception(const char* cause) throw()
    {
        std::stringstream ss;
        ss << "Exception during tracking: " << cause;
        text = ss.str();
    }

    ~tracking_exception() throw() {}

    const char* what() const throw()
    {
        return text.c_str();
    }

private:
    std::string text;
};

#endif // TRACKING_SERVICE_H
