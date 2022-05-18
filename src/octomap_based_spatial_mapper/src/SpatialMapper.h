#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Topics.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "hololens_depth_data_receiver_msgs/PointCloudFrame.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
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

struct StaticObjectsOctreeUpdateResult
{
    // The center points of all occupied voxels (in the octomap of the new sensor data frame) which were classified as
    // static voxels.
    std::vector<octomap::point3d> staticVoxelCenterPoints;

    // The center points of all occupied voxels (in the octomap of the new sensor data frame) which were classified as
    // dynamic voxels.
    std::vector<octomap::point3d> dynamicVoxelCenterPoints;

    // The center points of all occupied voxels (in the octomap of the new frame) which were previously assumed to be
    // static, but were now determined to be dynamic. All points in the spatial map point cloud which lie in any of
    // these voxels should be removed from the spatial map as we no longer consider them part of the static environment.
    std::vector<octomap::point3d> clearedVoxelCenterPoints;
};

class SpatialMapper
{
public:
    // Constructors.
    SpatialMapper(ros::NodeHandle n);

    // Destructors.
    ~SpatialMapper();

    // Callbacks for handling the incoming point cloud frames.
    void handlePointCloudFrame(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg);

private:
    // Initializes a voxel neighborhood based on the euclidean distance to the centermost voxel.
    std::vector<octomap::point3d> initializeEuclideanDistanceNeighborhood(double relativeNeighborDistance);

    // Converts a PCL point cloud to an Octomap OcTree.
    octomap::OcTree* pointCloudFrameToOctree(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg);

    // Filters an octree (must be expanded) by removing all free voxels neighboring at least one unknown voxel.
    octomap::OcTree* filterOctreeFreespace(octomap::OcTree* octreeToFilter);

    // Compares each voxel of the new octree (must be expanded) with the old octree.
    std::vector<VoxelDiffInfo> calculateOctreeVoxelDiff(
            octomap::OcTree* oldOctree,
            octomap::OcTree* newOctree,
            bool returnOnlyChanges = false);

    // Updates the octree containing the global spatial map based on the information contained in the given octree (must
    // be expanded) with information obtained in the current frame.
    StaticObjectsOctreeUpdateResult updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree);

    // Updates the floor height according to the given center points of occupied voxels. The floor is assumed to be at
    // the height of the lowest voxels ever observed.
    void updateFloorHeight(std::vector<octomap::point3d> voxelCenterPoints);

    // Removes all voxels corresponding to the floor from the given voxel center points.
    std::vector<octomap::point3d> removeFloorVoxels(std::vector<octomap::point3d> voxelCenterPointsToFilter);

    // Creates an octree where all voxels denoted by the given voxel center points are marked as occupied.
    octomap::OcTree* voxelCenterPointsToOctree(std::vector<octomap::point3d> voxelCenterPoints);

    // Detects clusters in the given octree (must be expanded and may only contain occupied and unknown voxels).
    std::vector<std::vector<octomap::point3d>> detectVoxelClusters(octomap::OcTree* octreeToCluster);

    // Creates a colorized octree where a voxels belonging to the same cluster will have the same color.
    octomap::ColorOcTree* createVoxelClusterOctree(std::vector<std::vector<octomap::point3d>> clusters);

    // Extracts all the given point cloud's points which lie inside the voxels denoted by the given voxel center points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPointsCorrespondingToVoxels(
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
            std::vector<octomap::point3d> voxelCenterPoints);

    // Publishes an Octomap OcTree using a given publisher.
    template<class OctomapT>
    void publishOctree(const OctomapT* octree, ros::Publisher& publisher, const ros::Time& timestamp)
    {
        octomap_msgs::Octomap octomapMsg;
        octomap_msgs::fullMapToMsg(*octree, octomapMsg);

        octomapMsg.header.seq = octomapSequenceNumber;
        octomapMsg.header.stamp = timestamp;
        octomapMsg.header.frame_id = "hololens_world";

        publisher.publish(octomapMsg);
    }

    // Hyper parameters for insertion of point clouds into the octree data structure.
    double leafSize;
    bool octreeInsertionLazyEval;
    bool octreeInsertionDiscretize;

    // Switches and hyper parameters for octree freespace filtering.
    bool doOctreeFreespaceFiltering;
    double octreeFilteringRelativeNeighborDistance;
    std::vector<octomap::point3d> octreeFilteringNeighborhood;

    // Hyper parameters for incorporating new octrees to the global spatial map.
    int numFreeObservationsBeforeVoxelRemoval;
    int numFramesBeforePossibleDynamicVoxelRemoval;

    // Hyper parameters for floor removal in voxels of dynamic objects.
    bool doFloorRemovalInDynamicObjects;
    int floorRemovalMinFloorVoxels;
    int floorRemovalRelativeNoiseHeight;

    // Hyper parameters for clustering dynamic voxels.
    double voxelClusteringRelativeClusterDistance;
    int voxelClusteringMinClusterSize;
    std::vector<octomap::point3d> voxelClusteringNeighborhood;
    std::vector<octomap::ColorOcTreeNode::Color> voxelClusterColors;

    // The octree storing information about the global spatial map.
    octomap::OcTree* staticObjectsOctree;

    // A mutex used for mutual exclusion when accessing the global spatial map.
    boost::mutex spatialMapMutex;

    // A list of voxels that may contain some movements.
    std::unordered_map<octomap::point3d, PossibleDynamicVoxelInfo> possibleDynamicVoxels;
    uint32_t dynamicVoxelsUpdateSequenceNumber;

    // The currently estimated floor height.
    float floorHeight = 10000.0;

    // A transform listener used for accessing the position.
    tf::TransformListener tfListener;

    // ROS publishers.
    ros::Publisher octomapCurrentFramePublisher;
    ros::Publisher octomapStaticObjectsPublisher;
    ros::Publisher octomapDynamicObjectsPublisher;
    ros::Publisher octomapDynamicObjectClustersPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t octomapSequenceNumber;
};
