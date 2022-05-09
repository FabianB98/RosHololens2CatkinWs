#include <unordered_map>

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

namespace std {
  template <>
  struct hash<octomap::point3d>
  {
    std::size_t operator()(const octomap::point3d& point) const
    {
        std::size_t res = 17;
        res = res * 31 + hash<float>()(point.x());
        res = res * 31 + hash<float>()(point.y());
        res = res * 31 + hash<float>()(point.z());
        return res;
    }
  };
}

class SpatialMapper
{
public:
    // Constructors.
    SpatialMapper(ros::NodeHandle n);

    // Destructors.
    ~SpatialMapper();

    // Callbacks for handling the incoming point cloud frames.
    void handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    // Converts a PCL point cloud to an Octomap OcTree.
    octomap::OcTree* pointCloudToOctree(const sensor_msgs::PointCloud2ConstPtr& msg);

    // Detects dynamic voxels based on inconsistencies between the octrees of the previous frame and the current frame.
    octomap::OcTree* detectDynamicVoxels(octomap::OcTree* previousFrameOctree, octomap::OcTree* currentFrameOctree);

    // Updates the octree containing the global spatial map based on the information contained in the given octree with
    // information obtained in the current frame.
    void updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree);

    // Publishes an Octomap OcTree using a given publisher.
    void publishOctree(const octomap::OcTree* octree, ros::Publisher& publisher, const ros::Time& timestamp);

    // Hyperparameters for insertion of point clouds into the octree data structure.
    double leafSize;
    double maxRange;

    // Octrees storing information about the occupancy in the previous frame and the global spatial map.
    octomap::OcTree* previousFrameOctree;
    octomap::OcTree* staticObjectsOctree;

    std::unordered_map<octomap::point3d, uint32_t> possibleDynamicVoxels;

    // A transform listener used for accessing the position.
    tf::TransformListener tfListener;

    // ROS publishers.
    ros::Publisher octomapCurrentFramePublisher;
    ros::Publisher octomapStaticObjectsPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t octomapSequenceNumber;
};
