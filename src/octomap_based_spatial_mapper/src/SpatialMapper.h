#include "Topics.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv);

class SpatialMapper
{
public:
    // Constructors.
    SpatialMapper(ros::NodeHandle n);

    // Callbacks for handling the incoming point cloud frames.
    void handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    // A transform listener used for accessing the position.
    tf::TransformListener tfListener;

    // ROS publishers.
    ros::Publisher octomapPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t octomapSequenceNumber;
};
