#include "SpatialMapper.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Hyper parameters for insertion of point clouds into the octree data structure.
#define LEAF_SIZE 0.05  // The size of a voxel (in meters).

// Switches and hyper parameters for octree freespace filtering.
#define DO_OCTREE_FREESPACE_FILTERING true      // Whether free voxels at the borders of new octrees should be filtered.
#define OCTREE_FILTERING_NEIGHBORHOOD_SIZE 3    // The size of the neighborhood to check when filtering octrees.

// Hyper parameters for incorporating new octrees to the global spatial map.
#define NUM_FREE_OBSERVATIONS_BEFORE_VOXEL_REMOVAL 5        // Mark occupied voxel as free after this many free observations.
#define NUM_FRAMES_BEFORE_POSSIBLE_DYNAMIC_VOXEL_REMOVAL 10 // Remove dynamic status after at most this many frames.

// Hyper parameters for floor removal in voxels of dynamic objects.
#define DO_FLOOR_REMOVAL_IN_DYNAMIC_VOXELS true // Whether potential floor voxels should be removed in dynamic voxels.
#define FLOOR_REMOVAL_MIN_FLOOR_VOXELS 25       // The minimum amount of potential floor voxels that need to be found.
#define FLOOR_REMOVAL_RELATIVE_NOISE_HEIGHT 1   // The relative height (in voxels) in which potential floor voxels can be.

// Hyper parameters for clustering dynamic voxels.
#define VOXEL_CLUSTERING_RELATIVE_CLUSTER_DISTANCE 1.9  // The mininum distance between two clusters relative to leaf size.
#define VOXEL_CLUSTERING_MIN_CLUSTER_SIZE 10            // Discard clusters with less than this amount of voxels.

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
    n.param("doOctreeFreespaceFiltering", doOctreeFreespaceFiltering, DO_OCTREE_FREESPACE_FILTERING);
    n.param("octreeFilteringNeighborhoodSize", octreeFilteringNeighborhoodSize, OCTREE_FILTERING_NEIGHBORHOOD_SIZE);
    n.param("numFreeObservationsBeforeVoxelRemoval", numFreeObservationsBeforeVoxelRemoval, NUM_FREE_OBSERVATIONS_BEFORE_VOXEL_REMOVAL);
    n.param("numFramesBeforePossibleDynamicVoxelRemoval", numFramesBeforePossibleDynamicVoxelRemoval, NUM_FRAMES_BEFORE_POSSIBLE_DYNAMIC_VOXEL_REMOVAL);
    n.param("doFloorRemovalInDynamicObjects", doFloorRemovalInDynamicObjects, DO_FLOOR_REMOVAL_IN_DYNAMIC_VOXELS);
    n.param("floorRemovalMinFloorVoxels", floorRemovalMinFloorVoxels, FLOOR_REMOVAL_MIN_FLOOR_VOXELS);
    n.param("floorRemovalRelativeNoiseHeight", floorRemovalRelativeNoiseHeight, FLOOR_REMOVAL_RELATIVE_NOISE_HEIGHT);
    n.param("voxelClusteringRelativeClusterDistance", voxelClusteringRelativeClusterDistance, VOXEL_CLUSTERING_RELATIVE_CLUSTER_DISTANCE);
    n.param("voxelClusteringMinClusterSize", voxelClusteringMinClusterSize, VOXEL_CLUSTERING_MIN_CLUSTER_SIZE);

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

    // Initialize the array of the neighborhood to check when clustering voxels.
    double clusterDistance = voxelClusteringRelativeClusterDistance * leafSize;
    double squaredClusterDistance = clusterDistance * clusterDistance;
    halfNeighborhoodSize = static_cast<int>(ceil(clusterDistance / leafSize));
    for (int x = -halfNeighborhoodSize; x <= halfNeighborhoodSize; x++)
    {
        for (int y = -halfNeighborhoodSize; y <= halfNeighborhoodSize; y++)
        {
            for (int z = -halfNeighborhoodSize; z <= halfNeighborhoodSize; z++)
            {
                // Don't add the centermost voxel (i.e. (0, 0, 0)) to its neighborhood.
                bool isCentermostVoxel = x == 0 && y == 0 && z == 0;

                double xOffset = x * leafSize;
                double yOffset = y * leafSize;
                double zOffset = z * leafSize;
                double squaredDistanceToCenter = xOffset * xOffset + yOffset * yOffset + zOffset * zOffset;
                bool satisfiesEuclideanDistance = squaredDistanceToCenter <= squaredClusterDistance;
                
                if (satisfiesEuclideanDistance && !isCentermostVoxel)
                {
                    voxelClusteringNeighborhood.push_back(octomap::point3d(xOffset, yOffset, zOffset));
                }
            }
        }
    }

    // Define the colors to use for clustering voxel clusters. I'm just going to assume that there will never be more
    // than 16 dynamic objects visible at once. If there will ever be more clusters than that, the assigned colors will
    // wrap around (i.e. the 17th cluster will have the same color as the first cluster, the 18th cluster will have the
    // same color as the second cluster, and so on). If that will ever be an issue, it is sufficient to add more colors
    // to this list.
    voxelClusterColors =
            {
                octomap::ColorOcTreeNode::Color(255, 0, 0),
                octomap::ColorOcTreeNode::Color(255, 128, 0),
                octomap::ColorOcTreeNode::Color(255, 255, 0),
                octomap::ColorOcTreeNode::Color(0, 255, 0),
                octomap::ColorOcTreeNode::Color(0, 255, 255),
                octomap::ColorOcTreeNode::Color(0, 0, 255),
                octomap::ColorOcTreeNode::Color(255, 0, 255),
                octomap::ColorOcTreeNode::Color(255, 255, 255),
                
                octomap::ColorOcTreeNode::Color(128, 0, 0),
                octomap::ColorOcTreeNode::Color(128, 64, 0),
                octomap::ColorOcTreeNode::Color(128, 128, 0),
                octomap::ColorOcTreeNode::Color(0, 128, 0),
                octomap::ColorOcTreeNode::Color(0, 128, 128),
                octomap::ColorOcTreeNode::Color(0, 0, 128),
                octomap::ColorOcTreeNode::Color(128, 0, 128),
                octomap::ColorOcTreeNode::Color(128, 128, 128)
            };

    // Initialize the global spatial map.
    staticObjectsOctree = new octomap::OcTree(leafSize);

    // Advertise the topics to which the results will be published.
    octomapCurrentFramePublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_CURRENT_FRAME_TOPIC, 10);
    octomapStaticObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_STATIC_OBJECTS_TOPIC, 10);
    octomapDynamicObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_DYNAMIC_OBJECTS_TOPIC, 10);
    octomapDynamicObjectClustersPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_DYNAMIC_OBJECTS_CLUSTERS_TOPIC, 10);
}

SpatialMapper::~SpatialMapper()
{
    delete staticObjectsOctree;
}

void SpatialMapper::handlePointCloudFrame(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg)
{
    octomap::OcTree* currentFrameOctree = pointCloudFrameToOctree(msg);
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
    if (doOctreeFreespaceFiltering)
    {
        octomap::OcTree* filteredOctree = filterOctreeFreespace(currentFrameOctree);
        filteredOctree->expand();

        delete currentFrameOctree;
        currentFrameOctree = filteredOctree;
    }

    // Register the new octree to the global spatial map and detect all dynamic changes in the scene.
    StaticObjectsOctreeUpdateResult updateResult = updateStaticObjectsOctree(currentFrameOctree);
    octomap::OcTree* dynamicObjectsOctree;
    if (doFloorRemovalInDynamicObjects)
    {
        // Potential floor voxels should be removed. Update the floor height and remove all dynamic voxels which might
        // be part of the floor or some noise above the floor.
        updateFloorHeight(updateResult.staticVoxelCenterPoints);
        updateFloorHeight(updateResult.dynamicVoxelCenterPoints);

        dynamicObjectsOctree = voxelCenterPointsToOctree(removeFloorVoxels(updateResult.dynamicVoxelCenterPoints));
    }
    else
    {
        // Potential floor voxels should not be removed. The octree containing all dynamic voxels can directly be
        // created from the dynamic voxel center points without any additional removal of certain voxels.
        dynamicObjectsOctree = voxelCenterPointsToOctree(updateResult.dynamicVoxelCenterPoints);
    }

    // Cluster all dynamic voxels.
    dynamicObjectsOctree->expand();
    std::vector<std::vector<octomap::point3d>> dynamicObjectClusters = detectVoxelClusters(dynamicObjectsOctree);
    octomap::ColorOcTree* dynamicObjectClustersOctree = createVoxelClusterOctree(dynamicObjectClusters);

    // Prune all previously expanded octrees in order to save space when publishing these octrees.
    currentFrameOctree->prune();
    dynamicObjectsOctree->prune();

    // Publish the results.
    ros::Time time = ros::Time::now();
    octomapSequenceNumber++;
    publishOctree(currentFrameOctree, octomapCurrentFramePublisher, time);
    publishOctree(staticObjectsOctree, octomapStaticObjectsPublisher, time);
    publishOctree(dynamicObjectsOctree, octomapDynamicObjectsPublisher, time);
    publishOctree(dynamicObjectClustersOctree, octomapDynamicObjectClustersPublisher, time);

    // Delete all octrees which we only used during this frame to ensure that we don't use more and more RAM over time.
    delete currentFrameOctree;
    delete dynamicObjectsOctree;
    delete dynamicObjectClustersOctree;
}

octomap::OcTree* SpatialMapper::pointCloudFrameToOctree(
        const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg)
{
    // Create an Octomap point cloud containing all points (including all artificial end points).
    octomap::Pointcloud pointcloudOctomap = octomap::Pointcloud();
    octomap::pointCloud2ToOctomap(msg->pointCloudWorldSpace, pointcloudOctomap);

    octomap::Pointcloud artificialEndpointsOctomap = octomap::Pointcloud();
    octomap::pointCloud2ToOctomap(msg->artificialEndpointsWorldSpace, artificialEndpointsOctomap);

    pointcloudOctomap.push_back(artificialEndpointsOctomap);

    // Convert the point cloud to an octree.
    octomap::point3d sensorOrigin
        = octomap::point3d(msg->hololensPosition.point.x, msg->hololensPosition.point.y, msg->hololensPosition.point.z);
    octomap::OcTree* octree = new octomap::OcTree(leafSize);
    octree->insertPointCloud(pointcloudOctomap, sensorOrigin, msg->maxReliableDepth);

    return octree;
}

octomap::OcTree* SpatialMapper::filterOctreeFreespace(octomap::OcTree* octreeToFilter)
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

void SpatialMapper::updateFloorHeight(std::vector<octomap::point3d> voxelCenterPoints)
{
    // Check if there is a voxel which is lower than the currently assumed floor height.
    float potentialNewFloorHeight = floorHeight;
    for (auto it = voxelCenterPoints.begin(); it != voxelCenterPoints.end(); it++)
    {
        float& voxelHeight = it->y();
        if (voxelHeight < floorHeight)
        {
            potentialNewFloorHeight = voxelHeight;
        }
    }

    if (potentialNewFloorHeight != floorHeight)
    {
        // There is at least one voxel which is lower than the currently assumed floor height. Verify that this is not
        // an outlier caused by some sensor noise.
        float rejectionHeightLower = potentialNewFloorHeight + leafSize - 1e-6;
        float rejectionHeightUpper = potentialNewFloorHeight + leafSize * floorRemovalRelativeNoiseHeight + 1e-6;

        int numVoxelsInPotentialNewFloorHeight = 0;
        int numVoxelsInRejectionHeight = 0;

        for (auto it = voxelCenterPoints.begin(); it != voxelCenterPoints.end(); it++)
        {
            float& voxelHeight = it->y();
            if (voxelHeight < rejectionHeightLower)
            {
                numVoxelsInPotentialNewFloorHeight++;
            }
            else if (voxelHeight < rejectionHeightUpper)
            {
                numVoxelsInRejectionHeight++;
            }
        }

        if (numVoxelsInPotentialNewFloorHeight > std::max(floorRemovalMinFloorVoxels, numVoxelsInRejectionHeight))
        {
            // There are more voxels on the new assumption of the floor height than there are voxels slightly above that
            // newly assumed floor height. It is therefore safe to say that the new assumption was not caused by an
            // outlier and that the old assumption must have been wrong. As the old assumption was wrong, we'll keep the
            // new assumption until we're proven wrong (again) at some point in the future.
            floorHeight = potentialNewFloorHeight;
            ROS_INFO("Changed floor height assumption to %f", floorHeight);
        }
    }
}

std::vector<octomap::point3d> SpatialMapper::removeFloorVoxels(std::vector<octomap::point3d> voxelCenterPointsToFilter)
{
    std::vector<octomap::point3d> nonFloorVoxels;
    float floorVoxelHeight = floorHeight + leafSize * floorRemovalRelativeNoiseHeight + 1e-6;
    for (auto it = voxelCenterPointsToFilter.begin(); it != voxelCenterPointsToFilter.end(); it++)
    {
        float& voxelHeight = it->y();
        if (voxelHeight > floorVoxelHeight)
        {
            nonFloorVoxels.push_back(*it);
        }
    }

    int numFloorVoxels = voxelCenterPointsToFilter.size() - nonFloorVoxels.size();
    if (numFloorVoxels >= floorRemovalMinFloorVoxels)
    {
        return nonFloorVoxels;
    }
    else
    {
        return voxelCenterPointsToFilter;
    }
}

octomap::OcTree* SpatialMapper::voxelCenterPointsToOctree(std::vector<octomap::point3d> voxelCenterPoints)
{
    octomap::OcTree* resultingOctree = new octomap::OcTree(leafSize);

    for (auto it = voxelCenterPoints.begin(); it != voxelCenterPoints.end(); it++)
    {
        octomap::OcTreeNode* insertedNode = resultingOctree->updateNode(*it, true);
        insertedNode->setLogOdds(1.0);
    }

    return resultingOctree;
}

std::vector<std::vector<octomap::point3d>> SpatialMapper::detectVoxelClusters(octomap::OcTree* octreeToCluster)
{
    // As described in the Azim 2012 paper, clustering of voxels can be done "using an approach similar to a region-
    // growing algorithm".
    // Initially, none of the voxels is checked, so we need to create a list (or a set in this implementation) of all
    // voxels that need to be checked (which obviously starts of containing all voxels of the octree to cluster).
    std::unordered_set<octomap::point3d> voxelsToCheck;
    for (octomap::OcTree::leaf_iterator it = octreeToCluster->begin_leafs(); it != octreeToCluster->end_leafs(); ++it)
    {
        voxelsToCheck.insert(it.getCoordinate());
    }
    
    std::vector<std::vector<octomap::point3d>> result;

    // The following loop takes care of the region-growing part. While it's not an exact implementation of what is
    // described in the Azim 2012 paper, it essentially still produces the same results: "[...] all possible dynamic
    // voxels are stored in a data list. Our clustering algorithm starts with stepping through this list. [...] If the
    // current voxel in the list is not yet assigned to any cluster, a new cluster is initialized. We find the set
    // Neighbor(v) of its neighboring voxels from the list. As criterion for adding a voxel to the cluster, we use the
    // Euclidean distance between the center of the current voxel and the voxel in consideration. If this criterion is
    // satisfied by the current voxel then it is added to the cluster. Now, we use this newly added voxel further and
    // continue the search within its neighborhood in a recursive manner."
    while (!voxelsToCheck.empty())
    {
        // As long as there are still voxels left to check (i.e. there are voxels which were not assigned to any cluster
        // yet), there is at least one cluster left. Therefore, we initialize a new cluster.
        std::vector<octomap::point3d> clusterVoxels;

        // All free voxels in the octree of voxels to check are not assigned to any cluster, so we can arbitrarily
        // choose one and use it as the seed point of the next cluster.
        octomap::point3d seedPoint = *voxelsToCheck.begin();
        
        // Cluster candidate voxels are all voxels in Neighbor(v) which were not checked yet, where v is any voxel which
        // is part of the current cluster.
        // Starting from the cluster's seed point, we'll recursively check search the cluster's neighborhood for any
        // voxels which must also be part of the current cluster.
        std::vector<octomap::point3d> clusterCandidateVoxels;
        clusterCandidateVoxels.push_back(seedPoint);
        while (!clusterCandidateVoxels.empty())
        {
            // All of the cluster candidates must be checked eventually, so we can just arbitrarily choose any to check
            // next. In this implementation, I'm using the one which was added most recently to the list of candidates
            // as this is the most efficient way (at least as long as the cluster candidates are stored in a vector).
            octomap::point3d voxelCenterPoint = clusterCandidateVoxels.back();
            voxelCenterPoint = octreeToCluster->keyToCoord(octreeToCluster->coordToKey(voxelCenterPoint));
            clusterCandidateVoxels.pop_back();
            
            // Ensure that we didn't already check this voxel. Otherwise we would run into an endless loop caused by
            // traversing loops in the neighborhood graph of the voxels.
            if (voxelsToCheck.find(voxelCenterPoint) != voxelsToCheck.end())
            {
                // Current voxel was not checked yet. Mark it as checked.
                voxelsToCheck.erase(voxelCenterPoint);

                // Before adding the voxel center point to the current cluster, ensure that voxelCenterPoint actually
                // represents the center of the current voxel. Due to floating point precision issues the calculated
                // center point may not be exactly in the voxel's center and might need to be ever so slightly adjusted.
                clusterVoxels.push_back(voxelCenterPoint);

                // Add all neighbors of the current voxel to the candidate list. This is the part which would cause an
                // infinite loop if we didn't check whether the current voxel was already checked.
                for (size_t i = 0; i < voxelClusteringNeighborhood.size(); i++)
                {
                    // The conversion from coord to key back to coord is needed to ensure that the calculated voxel
                    // center point is actually in the center of the voxel. Due to floating point imprecisions the
                    // calculated center point may not be exactly in the voxel's center and might need to be ever so
                    // slightly adjusted. If this wasn't done, it may happen that some of the neighbors may not be found
                    // in the voxelsToCheck set during lookup even though they are in that set.
                    octomap::point3d neighbor = voxelCenterPoint + voxelClusteringNeighborhood[i];
                    neighbor = octreeToCluster->keyToCoord(octreeToCluster->coordToKey(neighbor));
                    clusterCandidateVoxels.push_back(voxelCenterPoint + voxelClusteringNeighborhood[i]);
                }
            }
        }

        // Only keep clusters which contain at least a certain amount of voxels.
        if (clusterVoxels.size() >= voxelClusteringMinClusterSize)
        {
            result.push_back(clusterVoxels);
        }
    }

    return result;
}

octomap::ColorOcTree* SpatialMapper::createVoxelClusterOctree(std::vector<std::vector<octomap::point3d>> clusters)
{
    octomap::ColorOcTree* colorizedClusterOctree = new octomap::ColorOcTree(leafSize);
    
    for (int i = 0; i < clusters.size(); i++)
    {
        std::vector<octomap::point3d>& cluster = clusters[i];

        int colorIndex = i % voxelClusterColors.size();
        octomap::ColorOcTreeNode::Color& clusterColor = voxelClusterColors[colorIndex];

        for (auto voxelIt = cluster.begin(); voxelIt != cluster.end(); voxelIt++)
        {
            octomap::ColorOcTreeNode* insertedNode = colorizedClusterOctree->updateNode(*voxelIt, true);
            insertedNode->setLogOdds(1.0);
            insertedNode->setColor(clusterColor);
        }
    }

    return colorizedClusterOctree;
}