#include "SpatialMapper.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Hyper parameters for insertion of point clouds into the octree data structure.
#define LEAF_SIZE 0.05  // The size of a voxel (in meters).
#define MAX_RANGE -1.0  // The maximum range for how long individual beams are inserted into the octree data structure.

// Switches and hyper parameters for octree filtering.
#define DO_OCTREE_FILTERING true                // Whether free voxels at the borders of new octrees should be filtered.
#define OCTREE_FILTERING_NEIGHBORHOOD_SIZE 3    // The size of the neighborhood to check when filtering octrees.

// Hyper parameters for incorporating new octrees to the global spatial map.
#define NUM_FREE_OBSERVATIONS_BEFORE_VOXEL_REMOVAL 5        // Mark occupied voxel as free after this many free observations.
#define NUM_FRAMES_BEFORE_POSSIBLE_DYNAMIC_VOXEL_REMOVAL 10 // Remove dynamic status after at most this many frames.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

SpatialMapper::SpatialMapper(ros::NodeHandle n)
{
    ROS_INFO("Creating SpatialMapper...");

    // Initialize all switches and hyper parameters as specified by parameters (or their default value).
    n.param("octomapLeafSize", leafSize, LEAF_SIZE);
    n.param("octomapMaxRange", maxRange, MAX_RANGE); // TODO: maxRange could be fetched from the hololens_depth_data_receiver package.
    n.param("doOctreeFiltering", doOctreeFiltering, DO_OCTREE_FILTERING);
    n.param("octreeFilteringNeighborhoodSize", octreeFilteringNeighborhoodSize, OCTREE_FILTERING_NEIGHBORHOOD_SIZE);
    n.param("numFreeObservationsBeforeVoxelRemoval", numFreeObservationsBeforeVoxelRemoval, NUM_FREE_OBSERVATIONS_BEFORE_VOXEL_REMOVAL);
    n.param("numFramesBeforePossibleDynamicVoxelRemoval", numFramesBeforePossibleDynamicVoxelRemoval, NUM_FRAMES_BEFORE_POSSIBLE_DYNAMIC_VOXEL_REMOVAL);

    // Initialize the array of the neighborhood to check when filtering voxels.
    int halfNeighborhoodSize = octreeFilteringNeighborhoodSize / 2;
    for (int x = -halfNeighborhoodSize; x <= halfNeighborhoodSize; x++)
    {
        for (int y = -halfNeighborhoodSize; y <= halfNeighborhoodSize; y++)
        {
            for (int z = -halfNeighborhoodSize; z <= halfNeighborhoodSize; z++)
            {
                // Don't add the centermost voxel (i.e. (0, 0, 0)) to its neighborhood.
                if (x != 0 || y != 0 || z != 0)
                {
                    octreeFilteringNeighborhood.push_back(octomap::point3d(x * leafSize, y * leafSize, z * leafSize));
                }
            }
        }
    }

    // Initialize the global spatial map.
    staticObjectsOctree = new octomap::OcTree(leafSize);

    // Advertise the topics to which the results will be published.
    octomapCurrentFramePublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_CURRENT_FRAME_TOPIC, 10);
    octomapStaticObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_STATIC_OBJECTS_TOPIC, 10);
    octomapDynamicObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_DYNAMIC_OBJECTS_TOPIC, 10);
    pointCloudDynamicObjectsPublisher = n.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_DYNAMIC_OBJECTS_TOPIC, 10);
}

SpatialMapper::~SpatialMapper()
{
    delete staticObjectsOctree;
}

void SpatialMapper::handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentFramePointCloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *currentFramePointCloud);

    octomap::OcTree* currentFrameOctree = pointCloudToOctree(msg);
    currentFrameOctree->expand();

    // Filter all free voxels at the edges of the new octree. This will ensure that we don't accidentally mark voxels
    // as free even though they are actually occupied by some object which is ever so slightly outside the sensor's
    // viewing frustum or sensing range. While such objects usually shouldn't be a problem in almost all cases, there is
    // an edge case when looking at large flat objects (such as walls or the floor) at shallow angles.
    // For these edge cases, the conversion of the point cloud to an octree can cause a free voxel to be inserted in a
    // location which would be an occupied voxel if the sensor's field of view was ever so slightly larger. This is due
    // to the fact that the rays casted for the last pixels along the border of the sensor's field of view may ever so
    // slightly pass these false free voxels. As there is no further information about a point lying inside these voxels
    // caused by the sensor's finite viewing angle, there is no evidence for these voxels to be classified as occupied,
    // so they will be wrongly classified as free.
    // As the HoloLens's depth sensor has no 360° field of view, filtering the octree is therefore highly recommended.
    if (doOctreeFiltering)
    {
        octomap::OcTree* filteredOctree = filterOctree(currentFrameOctree);
        filteredOctree->expand();

        delete currentFrameOctree;
        currentFrameOctree = filteredOctree;
    }

    // Register the new octree to the global spatial map and detect all dynamic changes in the scene.
    StaticObjectsOctreeUpdateResult updateResult = updateStaticObjectsOctree(currentFrameOctree);
    octomap::OcTree* dynamicObjectsOctree = possibleDynamicVoxelsToOctree();

    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamicObjectsPointCloud = 
            extractPointsCorrespondingToVoxels(currentFramePointCloud, updateResult.dynamicVoxelCenterPoints);

    // Prune all previously expanded octrees in order to save space when publishing these octrees.
    currentFrameOctree->prune();

    // Publish the results.
    ros::Time time = ros::Time::now();
    octomapSequenceNumber++;
    pointCloudSequenceNumber++;
    publishOctree(currentFrameOctree, octomapCurrentFramePublisher, time);
    publishOctree(staticObjectsOctree, octomapStaticObjectsPublisher, time);
    publishOctree(dynamicObjectsOctree, octomapDynamicObjectsPublisher, time);
    publishPointCloud(dynamicObjectsPointCloud, pointCloudDynamicObjectsPublisher, time);

    // Delete all octrees which we only used during this frame to ensure that we don't use more and more RAM over time.
    delete currentFrameOctree;
    delete dynamicObjectsOctree;
}

octomap::OcTree* SpatialMapper::pointCloudToOctree(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    octomap::Pointcloud pointCloudOctomap = octomap::Pointcloud();
    octomap::pointCloud2ToOctomap(*msg, pointCloudOctomap);

    // TODO: Sensor origin needs to be fetched from the hololens_depth_data_receiver package.
    ros::Time pointCloudTimestamp = msg->header.stamp;
    tf::StampedTransform transform;
    try
    {
        tfListener.waitForTransform("hololens_world", "hololens_cam", pointCloudTimestamp, ros::Duration(3.0));
        tfListener.lookupTransform("hololens_world", "hololens_cam", pointCloudTimestamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Couldn't fetch the HoloLens's position!");
        ROS_ERROR("%s", ex.what());
    }
    octomap::point3d sensorOrigin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

    octomap::OcTree* octree = new octomap::OcTree(leafSize);
    octree->insertPointCloud(pointCloudOctomap, sensorOrigin, maxRange);

    return octree;
}

octomap::OcTree* SpatialMapper::filterOctree(octomap::OcTree* octreeToFilter)
{
    // The unfiltered octree needs to be expanded so that we can traverse all its leaves. This method assumes that the
    // given octree is already expanded (as we can therefore save some time by not expanding and pruning the same octree
    // multiple times). However, this method will therefore produce unwanted results when given an octree which is not
    // already expanded.
    
    octomap::OcTree* filteredOctree = new octomap::OcTree(leafSize);
    for (auto it = octreeToFilter->begin_leafs(); it != octreeToFilter->end_leafs(); ++it)
    {
        octomap::point3d coordinates = it.getCoordinate();

        if (octreeToFilter->isNodeOccupied(*it))
        {
            // Current voxel is occupied. Add it to the filtered octree without any further checks.
            octomap::OcTreeNode* insertedNode = filteredOctree->updateNode(coordinates, true);
            insertedNode->setLogOdds(it->getLogOdds());
        }
        else
        {
            // Current voxel is free. Check that none of its neighbors is an unknown voxel.
            for (size_t i = 0; i < octreeFilteringNeighborhood.size(); i++)
            {
                if (octreeToFilter->search(coordinates + octreeFilteringNeighborhood[i]) == NULL)
                {
                    // I know, I know, goto is considered bad. However, we need to continue with the next iteration of
                    // the outer loop (i.e. the one iterating over all leaves of the octree) and according to
                    // https://stackoverflow.com/a/41179682 using a goto statement seems to be the easiest solution in
                    // this case.
                    goto next_loop_iteration;
                }
            }

            // Current voxel is free and does not neighbor any unknown voxel. We can add it to the filtered octree.
            filteredOctree->updateNode(coordinates, false)->setLogOdds(it->getLogOdds());
        }

        // Label for jumping to the next loop iteration (or for leaving the loop if we're in the last iteration). As
        // already mentioned, using goto is generally considered bad, but we need this in this case for continuing the
        // outer loop (i.e. the current loop iterating over all leaves of the octree) from an inner loop.
        next_loop_iteration:;
    }

    return filteredOctree;
}

std::vector<VoxelDiffInfo> SpatialMapper::calculateOctreeVoxelDiff(
        octomap::OcTree* oldOctree,
        octomap::OcTree* newOctree,
        bool returnOnlyChanges)
{
    // Some parts of the following code on how to traverse an octree whilst comparing it with another octree (e.g. for
    // merging two octrees) were adopted from https://github.com/OctoMap/octomap/issues/227#issuecomment-571336801

    // The new octree needs to be expanded so that we can traverse all its leaves. This method assumes that the given
    // octree is already expanded (as we can therefore save some time by not expanding and pruning the same octree
    // multiple times). However, this method will therefore produce unwanted results when given an octree which is not
    // already expanded.

    // Iterate over all voxels for which we have acquired some information in the current frame.
    std::vector<VoxelDiffInfo> result;
    for (octomap::OcTree::leaf_iterator it = newOctree->begin_leafs(); it != newOctree->end_leafs(); ++it)
    {
        // Check whether the old octree already contains some information about the node/voxel of the new octree which
        // we're currently iterating over.
        octomap::point3d coordinates = it.getCoordinate();
        octomap::OcTreeNode* nodeOldOctree = oldOctree->search(coordinates);
        if (nodeOldOctree != NULL) {
            // There already is a node/voxel in the old octree. We therefore have occupancy information from some
            // previous frame to which we can compare the new occupancy information against.
            bool voxelOccupiedOld = oldOctree->isNodeOccupied(nodeOldOctree);
            bool voxelOccupiedNew = newOctree->isNodeOccupied(*it);
            bool stateChanged = voxelOccupiedOld != voxelOccupiedNew;

            if (stateChanged)
            {
                if (voxelOccupiedNew)
                {
                    // Occupancy state transition: free --> occupied.
                    result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::FREE_OCCUPIED, nodeOldOctree, &(*it)));
                }
                else
                {
                    // Occupancy state transition: occupied --> free.
                    result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::OCCUPIED_FREE, nodeOldOctree, &(*it)));
                }
            }
            else if (!returnOnlyChanges)
            {
                if (voxelOccupiedNew)
                {
                    // Occupancy state transition: occupied --> occupied.
                    result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::OCCUPIED_OCCUPIED, nodeOldOctree, &(*it)));
                }
                else
                {
                    // Occupancy state transition: free --> free.
                    result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::FREE_FREE, nodeOldOctree, &(*it)));
                }
            }
        } 
        else 
        {
            // We don't have any information about the current voxel from any previous frame, so we're unable to compare
            // the current voxel's occupancy information against something.
            bool voxelOccupiedNew = newOctree->isNodeOccupied(*it);

            if (voxelOccupiedNew)
            {
                // Occupancy state transition: unknown --> occupied.
                result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::UNKNOWN_OCCUPIED, nullptr, &(*it)));
            }
            else
            {
                // Occupancy state transition: unknown --> free.
                result.push_back(VoxelDiffInfo(coordinates, VoxelDiffType::UNKNOWN_FREE, nullptr, &(*it)));
            }
        }
    }

    return result;
}

StaticObjectsOctreeUpdateResult SpatialMapper::updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree)
{
    StaticObjectsOctreeUpdateResult result;

    spatialMapMutex.lock();

    std::vector<VoxelDiffInfo> diff = calculateOctreeVoxelDiff(staticObjectsOctree, currentFrameOctree);
    for (auto it = diff.begin(); it != diff.end(); it++)
    {
        if (it->type == VoxelDiffType::FREE_OCCUPIED)
        {
            // Azim 2012 paper states: "If the transition between these two states for a specific voxel of the grid is
            // such that S_{t-1} = free and S_t = occupied then this is the case when an object is detected on a
            // location previously seen as free space and it is possibly a moving object. We add it to the list of
            // possible dynamic voxels.
            possibleDynamicVoxels[it->coordinates] = PossibleDynamicVoxelInfo(dynamicVoxelsUpdateSequenceNumber);

            // What if the initial observation about the voxel being free was a measurement error? This is not mentioned
            // in the paper...

            result.dynamicVoxelCenterPoints.push_back(it->coordinates);
        }
        else if (it->type == VoxelDiffType::OCCUPIED_FREE)
        {
            // Azim 2012 paper states: "In contrary, if S_{t-1} = occupied and S_t = free, it means that the location
            // which was previously observed as occupied is free now. This can possibly be caused by a missed detection
            // by the sensor or it was a voxel occupied by a dynamic object which may have displaced now. We search this
            // voxel in our list of dynamic voxels maintained from previous scans. If it is found, we wait for the next
            // few scans instead of removing it from the dynamic voxels list immediately. If it is observed as free in
            // the next scans was well, then we delete it from the list."
            auto search = possibleDynamicVoxels.find(it->coordinates);
            if (search == possibleDynamicVoxels.end())
            {
                // I'm deviating here from the paper by also ensuring that a voxel is added to the list of possible
                // dynamic voxels if it was originally observed as occupied, but is now seen as free.
                possibleDynamicVoxels[it->coordinates] = PossibleDynamicVoxelInfo(dynamicVoxelsUpdateSequenceNumber);
            }

            PossibleDynamicVoxelInfo& dynamicVoxelInfo = possibleDynamicVoxels[it->coordinates];
            dynamicVoxelInfo.freeCounter++;
            dynamicVoxelInfo.lastUpdate = dynamicVoxelsUpdateSequenceNumber;

            if (dynamicVoxelInfo.freeCounter >= numFreeObservationsBeforeVoxelRemoval)
            {
                possibleDynamicVoxels.erase(it->coordinates);

                it->nodeOldOctree->setLogOdds(-1.0);
                result.clearedVoxelCenterPoints.push_back(it->coordinates);
            }

            // What if we only observed the voxel in the first frame as being occupied and after that always as free
            // (e.g. due to a measurement error)? In this case it will never be added to the list of possible dynamic
            // voxels meaning that it will never be marked as free even though it is free... This case is not mentioned
            // in the paper...
        }
        else if (it->type == VoxelDiffType::OCCUPIED_OCCUPIED)
        {
            // Azim 2012 paper states: "If S_{t-1} = occupied and S_t = occupied, it means that an object is observed on
            // a location previously occupied then it probably is static."
            auto search = possibleDynamicVoxels.find(it->coordinates);
            if (search != possibleDynamicVoxels.end())
            {
                possibleDynamicVoxels.erase(it->coordinates);
            }

            result.staticVoxelCenterPoints.push_back(it->coordinates);
        }
        else if (it->type == VoxelDiffType::UNKNOWN_OCCUPIED)
        {
            // Azim 2012 paper states: "If an object appears at a location which was previously unobserved, then we can
            // say nothing about that object. For such measurements, a priori we will suppose that they are static until
            // later evidences come."
            octomap::OcTreeNode* insertedNode = staticObjectsOctree->updateNode(it->coordinates, true);
            insertedNode->setLogOdds(1.0);

            result.staticVoxelCenterPoints.push_back(it->coordinates);
        }
        else if (it->type == VoxelDiffType::UNKNOWN_FREE)
        {
            // This case is not mentioned in the paper, but I assume that its similar to the unknown -> occupied case.
            octomap::OcTreeNode* insertedNode = staticObjectsOctree->updateNode(it->coordinates, false);
            insertedNode->setLogOdds(-1.0);
        }
    }

    // The list of possible dynamic voxels might grow over time if there are no further observations for voxels in that
    // list. Maybe remove voxels that are too long in that list again? This is not mentioned in the paper...
    uint32_t removeIfLastUpdateBefore = dynamicVoxelsUpdateSequenceNumber - numFramesBeforePossibleDynamicVoxelRemoval;
    for (auto it = possibleDynamicVoxels.begin(); it != possibleDynamicVoxels.end();)
    {
        if (it->second.lastUpdate < removeIfLastUpdateBefore)
        {
            it = possibleDynamicVoxels.erase(it);
        }
        else
        {
            it++;
        }
    }

    dynamicVoxelsUpdateSequenceNumber++;

    spatialMapMutex.unlock();

    return result;
}

octomap::OcTree* SpatialMapper::possibleDynamicVoxelsToOctree()
{
    octomap::OcTree* dynamicVoxelsOctree = new octomap::OcTree(leafSize);

    spatialMapMutex.lock();

    for (auto it = possibleDynamicVoxels.begin(); it != possibleDynamicVoxels.end(); it++)
    {
        octomap::OcTreeNode* insertedNode = dynamicVoxelsOctree->updateNode(it->first, true);
        insertedNode->setLogOdds(1.0);
    }

    spatialMapMutex.unlock();

    return dynamicVoxelsOctree;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::extractPointsCorrespondingToVoxels(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        std::vector<octomap::point3d> voxelCenterPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInVoxels (new pcl::PointCloud<pcl::PointXYZ>());

    // Iterating over all voxels and filtering the points of each voxel separately is definitely NOT performant. This
    // needs to be changed in the future to incorporate some better algorithm which clusters neighboring voxels into a
    // single, but larger box.
    // float halfLeafSize = leafSize / 2.0;
    // for (auto it = voxelCenterPoints.begin(); it != voxelCenterPoints.end(); it++)
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInCurrentVoxel (new pcl::PointCloud<pcl::PointXYZ>());

    //     pcl::CropBox<pcl::PointXYZ> boxFilter;
    //     boxFilter.setMin(Eigen::Vector4f(it->x() - halfLeafSize, it->y() - halfLeafSize, it->z() - halfLeafSize, 1.0));
    //     boxFilter.setMax(Eigen::Vector4f(it->x() + halfLeafSize, it->y() + halfLeafSize, it->z() + halfLeafSize, 1.0));
    //     boxFilter.setInputCloud(pointCloudToFilter);
    //     boxFilter.filter(*pointsInCurrentVoxel);

    //     *pointsInVoxels += *pointsInCurrentVoxel;
    // }

    return pointsInVoxels;
}

void SpatialMapper::publishOctree(const octomap::OcTree* octree, ros::Publisher& publisher, const ros::Time& timestamp)
{
    octomap_msgs::Octomap octomapMsg;
    octomap_msgs::fullMapToMsg(*octree, octomapMsg);

    octomapMsg.header.seq = octomapSequenceNumber;
    octomapMsg.header.stamp = timestamp;
    octomapMsg.header.frame_id = "hololens_world";

    publisher.publish(octomapMsg);
}

void SpatialMapper::publishPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        ros::Publisher& publisher,
        const ros::Time& timestamp)
{
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    pointCloudMessage.header.seq = pointCloudSequenceNumber;
    pointCloudMessage.header.stamp = timestamp;
    pointCloudMessage.header.frame_id = "hololens_world";

    publisher.publish(pointCloudMessage);
}