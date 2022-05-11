#include <unordered_map>
#include <vector>

#include "Topics.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

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

enum VoxelDiffType {
    FREE_FREE,
    FREE_OCCUPIED,
    OCCUPIED_FREE,
    OCCUPIED_OCCUPIED,
    UNKNOWN_FREE,
    UNKNOWN_OCCUPIED
};

struct VoxelDiffInfo {
    octomap::point3d coordinates;
    VoxelDiffType type;
    octomap::OcTreeNode* nodeOldOctree;
    octomap::OcTreeNode* nodeNewOctree;

    VoxelDiffInfo() {}

    VoxelDiffInfo(
            octomap::point3d _coordinates, 
            VoxelDiffType _type, 
            octomap::OcTreeNode* _nodeOldOctree, 
            octomap::OcTreeNode* _nodeNewOctree)
    {
        coordinates = _coordinates;
        type = _type;
        nodeOldOctree = _nodeOldOctree;
        nodeNewOctree = _nodeNewOctree;
    }
};

struct PossibleDynamicVoxelInfo {
    uint32_t freeCounter;
    uint32_t lastUpdate;

    PossibleDynamicVoxelInfo(uint32_t _lastUpdate = 0)
    {
        freeCounter = 0;
        lastUpdate = _lastUpdate;
    }
};

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

    // Filters an octree by removing all free voxels neighboring at least one unknown voxel.
    octomap::OcTree* filterOctree(octomap::OcTree* octreeToFilter);

    // Compares each voxel of the new octree with the old octree.
    std::vector<VoxelDiffInfo> calculateOctreeVoxelDiff(
            octomap::OcTree* oldOctree,
            octomap::OcTree* newOctree,
            bool returnOnlyChanges = false);

    // Updates the octree containing the global spatial map based on the information contained in the given octree with
    // information obtained in the current frame.
    void updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree);

    // Publishes an Octomap OcTree using a given publisher.
    void publishOctree(const octomap::OcTree* octree, ros::Publisher& publisher, const ros::Time& timestamp);

    // Hyper parameters for insertion of point clouds into the octree data structure.
    double leafSize;
    double maxRange;

    // Switches and hyper parameters for octree filtering.
    bool doOctreeFiltering;
    int octreeFilteringNeighborhoodSize;
    std::vector<octomap::point3d> octreeFilteringNeighborhood;

    // Hyper parameters for incorporating new octrees to the global spatial map.
    int numFreeObservationsBeforeVoxelRemoval;
    int numFramesBeforePossibleDynamicVoxelRemoval;

    // The octree storing information about the global spatial map.
    octomap::OcTree* staticObjectsOctree;

    // A mutex used for mutual exclusion when accessing the global spatial map.
    boost::mutex spatialMapMutex;

    // A list of voxels that may contain some movements.
    std::unordered_map<octomap::point3d, PossibleDynamicVoxelInfo> possibleDynamicVoxels;
    uint32_t dynamicVoxelsUpdateSequenceNumber;

    // A transform listener used for accessing the position.
    tf::TransformListener tfListener;

    // ROS publishers.
    ros::Publisher octomapCurrentFramePublisher;
    ros::Publisher octomapStaticObjectsPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t octomapSequenceNumber;
};
