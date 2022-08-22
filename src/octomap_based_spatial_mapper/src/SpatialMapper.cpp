#include "SpatialMapper.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Hyper parameters for insertion of point clouds into the octree data structure.
#define LEAF_SIZE 0.05                                  // The size of a voxel (in meters).
#define OCTREE_INSERTION_LAZY_EVAL false                // Whether inner nodes are only updated after the point cloud was inserted.
#define OCTREE_INSERTION_DISCRETIZE false               // Whether the point cloud is discretized into octree key cells.
#define OCTREE_INSERTION_USE_ARTIFICIAL_ENDPOINTS false // Whether artificial endpoints should be used as information about free space.

// Switches and hyper parameters for octree freespace filtering.
#define DO_OCTREE_FREESPACE_FILTERING true              // Whether free voxels at the borders of octrees should be filtered.
#define OCTREE_FILTERING_RELATIVE_NEIGHBOR_DISTANCE 1.5 // The distance to neighboring voxels relative to leaf size.

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

// Switches and hyper parameters for removal of dynamic voxel clusters containing only sensor noise.
#define DO_NOISE_CLUSTER_REMOVAL true                               // Whether clusters of sensor noise should be removed.
#define NOISE_CLUSTER_REMOVAL_RELATIVE_NEIGHBOR_DISTANCE 1.0        // The distance to neighboring voxels relative to leaf size.
#define NOISE_CLUSTER_REMOVAL_STATIC_NEIGHBOR_PERCENTAGE 0.75       // How many voxels of the cluster must have a static neighbor.
#define NOISE_CLUSTER_REMOVAL_ADD_NOISE_CLUSTERS_TO_STATIC_MAP true // Should noise clusters be added to the spatial map of static objects?

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
    n.param("octreeInsertionLazyEval", octreeInsertionLazyEval, OCTREE_INSERTION_LAZY_EVAL);
    n.param("octreeInsertionDiscretize", octreeInsertionDiscretize, OCTREE_INSERTION_DISCRETIZE);
    n.param("octreeInsertionUseArtificialEndpoints", octreeInsertionUseArtificialEndpoints, OCTREE_INSERTION_USE_ARTIFICIAL_ENDPOINTS);
    n.param("doOctreeFreespaceFiltering", doOctreeFreespaceFiltering, DO_OCTREE_FREESPACE_FILTERING);
    n.param("octreeFilteringRelativeNeighborDistance", octreeFilteringRelativeNeighborDistance, OCTREE_FILTERING_RELATIVE_NEIGHBOR_DISTANCE);
    n.param("numFreeObservationsBeforeVoxelRemoval", numFreeObservationsBeforeVoxelRemoval, NUM_FREE_OBSERVATIONS_BEFORE_VOXEL_REMOVAL);
    n.param("numFramesBeforePossibleDynamicVoxelRemoval", numFramesBeforePossibleDynamicVoxelRemoval, NUM_FRAMES_BEFORE_POSSIBLE_DYNAMIC_VOXEL_REMOVAL);
    n.param("doFloorRemovalInDynamicObjects", doFloorRemovalInDynamicObjects, DO_FLOOR_REMOVAL_IN_DYNAMIC_VOXELS);
    n.param("floorRemovalMinFloorVoxels", floorRemovalMinFloorVoxels, FLOOR_REMOVAL_MIN_FLOOR_VOXELS);
    n.param("floorRemovalRelativeNoiseHeight", floorRemovalRelativeNoiseHeight, FLOOR_REMOVAL_RELATIVE_NOISE_HEIGHT);
    n.param("voxelClusteringRelativeClusterDistance", voxelClusteringRelativeClusterDistance, VOXEL_CLUSTERING_RELATIVE_CLUSTER_DISTANCE);
    n.param("voxelClusteringMinClusterSize", voxelClusteringMinClusterSize, VOXEL_CLUSTERING_MIN_CLUSTER_SIZE);
    n.param("doNoiseClusterRemoval", doNoiseClusterRemoval, DO_NOISE_CLUSTER_REMOVAL);
    n.param("noiseClusterRemovalRelativeNeighborDistance", noiseClusterRemovalRelativeNeighborDistance, NOISE_CLUSTER_REMOVAL_RELATIVE_NEIGHBOR_DISTANCE);
    n.param("noiseClusterRemovalStaticNeighborPercentage", noiseClusterRemovalStaticNeighborPercentage, NOISE_CLUSTER_REMOVAL_STATIC_NEIGHBOR_PERCENTAGE);
    noiseClusterRemovalNoStaticNeighborPercentage = 1.0 - noiseClusterRemovalStaticNeighborPercentage;
    n.param("noiseClusterRemovalAddNoiseClustersToStaticMap", noiseClusterRemovalAddNoiseClustersToStaticMap, NOISE_CLUSTER_REMOVAL_ADD_NOISE_CLUSTERS_TO_STATIC_MAP);

    // Initialize the array of the neighborhoods to check when filtering voxels and when clustering voxels.
    octreeFilteringNeighborhood = initializeEuclideanDistanceNeighborhood(octreeFilteringRelativeNeighborDistance);
    voxelClusteringNeighborhood = initializeEuclideanDistanceNeighborhood(voxelClusteringRelativeClusterDistance);
    noiseClusterRemovalNeighborhood = initializeEuclideanDistanceNeighborhood(noiseClusterRemovalRelativeNeighborDistance);

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
    
    // Define the colors to use for visualizing the detected object class.
    std_msgs::ColorRGBA humanColor;
    humanColor.r = 0.0;
    humanColor.g = 1.0;
    humanColor.b = 0.5;
    humanColor.a = 1.0;
    objectClassColors[ObjectClass::HUMAN] = humanColor;

    std_msgs::ColorRGBA robotColor;
    robotColor.r = 1.0;
    robotColor.g = 0.5;
    robotColor.b = 0.0;
    robotColor.a = 1.0;
    objectClassColors[ObjectClass::ROBOT] = robotColor;

    std_msgs::ColorRGBA unknownColor;
    unknownColor.r = 0.5;
    unknownColor.g = 0.5;
    unknownColor.b = 0.5;
    unknownColor.a = 1.0;
    objectClassColors[ObjectClass::UNKNOWN] = unknownColor;

    std_msgs::ColorRGBA classificationFailedColor;
    classificationFailedColor.r = 0.0;
    classificationFailedColor.g = 0.0;
    classificationFailedColor.b = 0.0;
    classificationFailedColor.a = 1.0;
    objectClassColors[ObjectClass::CLASSIFICATION_FAILED] = classificationFailedColor;

    // Initialize the global spatial map.
    staticObjectsOctree = new octomap::OcTree(leafSize);

    // Register the service clients for all services which will be called.
    clusterClassifierService = n.serviceClient<object3d_detector::ClassifyClusters>(CLUSTER_CLASSIFICATION_SERVICE_NAME);

    // Advertise the topics to which the results will be published.
    octomapCurrentFramePublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_CURRENT_FRAME_TOPIC, 10);
    octomapStaticObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_STATIC_OBJECTS_TOPIC, 10);
    octomapDynamicObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_DYNAMIC_OBJECTS_TOPIC, 10);
    octomapDynamicObjectClustersPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_DYNAMIC_OBJECTS_CLUSTERS_TOPIC, 10);
    boundingBoxDynamicObjectClustersPublisher = n.advertise<visualization_msgs::MarkerArray>(BOUNDING_BOX_DYNAMIC_OBJECT_CLUSTERS_TOPIC, 10);
    dynamicClusterCentroidsPublisher = n.advertise<geometry_msgs::PoseArray>(DYNAMIC_CLUSTER_CENTROIDS_CLUSTERS_OF_INTEREST_TOPIC, 10);
}

SpatialMapper::~SpatialMapper()
{
    delete staticObjectsOctree;
}

std::vector<octomap::point3d> SpatialMapper::initializeEuclideanDistanceNeighborhood(double relativeNeighborDistance)
{
    std::vector<octomap::point3d> neighborhood;

    double neighborDistance = relativeNeighborDistance * leafSize;
    double squaredNeighborDistance = neighborDistance * neighborDistance;
    int halfNeighborhoodSize = static_cast<int>(ceil(relativeNeighborDistance));
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
                bool satisfiesEuclideanDistance = squaredDistanceToCenter <= squaredNeighborDistance;

                if (satisfiesEuclideanDistance && !isCentermostVoxel)
                {
                    neighborhood.push_back(octomap::point3d(x * leafSize, y * leafSize, z * leafSize));
                }
            }
        }
    }

    ROS_INFO("Calculated neighborhood with relative neighbor distance of %f contains %lu voxels.",
            relativeNeighborDistance, neighborhood.size());

    return neighborhood;
}

void SpatialMapper::handlePointCloudFrame(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg->pointCloudWorldSpace, *pointCloud);

    octomap::OcTree* currentFrameOctree = pointCloudFrameToOctree(msg);
    currentFrameOctree->expand();

    // As this ROS node uses multiple threads to process received messages, updateSpatialMap might be changed at any
    // point in time. In order to ensure that this won't cause any unintended side effects when it is changed while
    // any thread is inside this function, we'll create a local copy of the current state of updateSpatialMap to ensure
    // that the current execution consistently does or doesn't update the spatial map (instead of changing behavior at
    // some point in time during processing of this frame).
    bool updateSpatialMapThisFrame = updateSpatialMap;

    // Filter all free voxels at the edges of the new octree. This will ensure that we don't accidentally mark voxels
    // as free even though they are actually occupied by some object which is ever so slightly outside the sensor's
    // view frustum or sensing range. While such objects usually shouldn't be a problem in almost all cases, there is
    // an edge case when looking at large flat objects (such as walls or the floor) at shallow angles.
    // For these edge cases, the conversion of the point cloud to an octree can cause a free voxel to be inserted in a
    // location which would be an occupied voxel if the sensor's field of view was ever so slightly larger. This is due
    // to the fact that the rays casted for the last pixels along the border of the sensor's field of view may ever so
    // slightly pass these false free voxels. As there is no further information about a point lying inside these voxels
    // caused by the sensor's finite viewing angle, there is no evidence for these voxels to be classified as occupied,
    // so they will be wrongly classified as free.
    // As the HoloLens's depth sensor has no 360° field of view, filtering the octree is therefore highly recommended.
    if (doOctreeFreespaceFiltering && updateSpatialMapThisFrame)
    {
        octomap::OcTree* filteredOctree = filterOctreeFreespace(currentFrameOctree);
        filteredOctree->expand();

        delete currentFrameOctree;
        currentFrameOctree = filteredOctree;
    }

    // Register the new octree to the global spatial map and detect all dynamic changes in the scene.
    VoxelClassificationResult classificationResult = updateSpatialMapThisFrame 
            ? classifyVoxelsWithSpatialMapUpdate(currentFrameOctree)
            : classifyVoxelsWithoutSpatialMapUpdate(currentFrameOctree);
    octomap::OcTree* dynamicObjectsOctree;
    if (doFloorRemovalInDynamicObjects)
    {
        // Potential floor voxels should be removed. Update the floor height and remove all dynamic voxels which might
        // be part of the floor or some noise above the floor.
        if (updateSpatialMapThisFrame)
        {
            updateFloorHeight(classificationResult.staticVoxelCenterPoints);
            updateFloorHeight(classificationResult.dynamicVoxelCenterPoints);
        }

        std::vector<octomap::point3d> dynamicVoxels = removeFloorVoxels(classificationResult.dynamicVoxelCenterPoints);
        dynamicObjectsOctree = voxelCenterPointsToOctree(dynamicVoxels);
    }
    else
    {
        // Potential floor voxels should not be removed. The octree containing all dynamic voxels can directly be
        // created from the dynamic voxel center points without any additional removal of certain voxels.
        dynamicObjectsOctree = voxelCenterPointsToOctree(classificationResult.dynamicVoxelCenterPoints);
    }

    // Cluster all dynamic voxels.
    dynamicObjectsOctree->expand();
    std::vector<std::vector<octomap::point3d>> dynamicObjectClusters = detectVoxelClusters(dynamicObjectsOctree);
    if (doNoiseClusterRemoval)
    {
        std::pair<std::vector<std::vector<octomap::point3d>>, std::vector<std::vector<octomap::point3d>>>
                noiseClusterDetectionResult = removeNoiseVoxelClusters(dynamicObjectClusters);
        dynamicObjectClusters = noiseClusterDetectionResult.first;

        if (updateSpatialMapThisFrame && noiseClusterRemovalAddNoiseClustersToStaticMap)
        {
            std::vector<std::vector<octomap::point3d>> noiseClusters = noiseClusterDetectionResult.second;
            addClustersToStaticObjectsMap(noiseClusters);
        }
    }
    dynamicObjectClusters = removeSmallVoxelClusters(dynamicObjectClusters);
    octomap::ColorOcTree* dynamicObjectClustersOctree = createVoxelClusterOctree(dynamicObjectClusters);
    
    // Determine the bounding boxes of each cluster, their contained points and their centroids. The centroids will be
    // used as the observations for tracking. Actual tracking will be performed by another ROS node based on the
    // published centroids.
    std::vector<BoundingBox> boundingBoxes = calculateBoundingBoxes(dynamicObjectClusters);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterPointClouds = extractPointsCorrespondingToBoundingBoxes(
            boundingBoxes, pointCloud);
    std::vector<pcl::PointXYZ> clusterCentroids = calculateCentroids(clusterPointClouds);

    // Classify each cluster.
    std::vector<ClassificationResult> classificationResults = classifyVoxelClusters(clusterPointClouds,
            clusterCentroids, msg->hololensPosition.point);
    std::vector<size_t> clusterIndicesClustersOfInterest = selectClustersByClass(classificationResults, 
            {ObjectClass::HUMAN, ObjectClass::ROBOT});
    std::vector<size_t> clusterIndicesUnknown = selectClustersByClass(classificationResults,
            {ObjectClass::UNKNOWN, ObjectClass::CLASSIFICATION_FAILED});

    // TODO: Clusters which were classified as corresponding to the unknown/background object class could be checked
    // if they are somewhat stationary. In case they are determined to be not moving (or only slightly jittering around
    // some point in space), these clusters could correspond to static objects which were falsely removed from the
    // spatial map of static objects, so that it may be useful to add them back to the spatial map of static objects. It
    // may be useful to test the benefits and drawbacks of doing so.

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
    publishBoundingBoxes(boundingBoxes, classificationResults, clusterIndicesClustersOfInterest,
            boundingBoxDynamicObjectClustersPublisher, time);
    publishCentroids(clusterCentroids, clusterIndicesClustersOfInterest, dynamicClusterCentroidsPublisher, time);

    // Delete all octrees which we only used during this frame to ensure that we don't use more and more RAM over time.
    delete currentFrameOctree;
    delete dynamicObjectsOctree;
    delete dynamicObjectClustersOctree;
}

octomap::OcTree* SpatialMapper::pointCloudFrameToOctree(
        const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg)
{
    // Create an Octomap point cloud containing all points.
    octomap::Pointcloud pointcloudOctomap = octomap::Pointcloud();
    octomap::pointCloud2ToOctomap(msg->pointCloudWorldSpace, pointcloudOctomap);

    // Convert the point cloud to an octree.
    octomap::point3d sensorOrigin
        = octomap::point3d(msg->hololensPosition.point.x, msg->hololensPosition.point.y, msg->hololensPosition.point.z);
    octomap::OcTree* octree = new octomap::OcTree(leafSize);
    octree->insertPointCloud(pointcloudOctomap, sensorOrigin, msg->maxReliableDepth, octreeInsertionLazyEval,
            octreeInsertionDiscretize);

    // Use the artificial endpoints as additional information about free space, in case this is wanted.
    // Please note that using artificial endpoints is not recommended for the HoloLens 2, as this will bring up more
    // issues than it helps solve. While the depth sensor embedded into the HoloLens 2 distinguishes between a multiple
    // reasons why a depth value is invalid (one of them being the measured object being too far away), it may sometimes
    // fail to determine the correct invalidation reason, such that depth values may be declared as being too far away
    // even though this is not the case in reality. Specifically, this may be the case when looking at a reflective
    // surface (as for example a PC monitor or some metal object with a glossy surface) at a shallow angle while this
    // surface is relatively close in front of the HoloLens. Another case in which the HoloLens's invalidation algorithm
    // fails to correctly determine the invalidation reason happens when there is an object very close in front of the
    // HoloLens (just a few centimeters away from the sensor). While this close object is correctly determined as being
    // too close in front of the sensor, other objects which are further away from the HoloLens (but still within the
    // depth range in which the HoloLens usually returns valid depth values) may be partially invalidated as being too
    // far away. Due to these reasons, there may be more free space inserted than there is in reality, resulting in
    // voxels being falsely removed from the spatial map of static objects.
    if (octreeInsertionUseArtificialEndpoints)
    {
        octomap::Pointcloud artificialEndpointsOctomap = octomap::Pointcloud();
        octomap::pointCloud2ToOctomap(msg->artificialEndpointsWorldSpace, artificialEndpointsOctomap);

        octree->insertPointCloud(artificialEndpointsOctomap, sensorOrigin, msg->maxReliableDepth,
                octreeInsertionLazyEval, octreeInsertionDiscretize);
    }
    
    if (octreeInsertionLazyEval)
    {
        octree->updateInnerOccupancy();
    }

    return octree;
}

octomap::OcTree* SpatialMapper::filterOctreeFreespace(octomap::OcTree* octreeToFilter)
{
    // The unfiltered octree needs to be expanded so that we can traverse all its leaves. This method assumes that the
    // given octree is already expanded (as we can therefore save some time by not expanding and pruning the same octree
    // multiple times). However, this method will therefore produce unwanted results when given an octree which is not
    // already expanded.
    
    // To begin with, we're storing the occupancy information of all known voxels in a hashmap. While we could always
    // look that up on the octree whenever we need to access this information, it is actually faster to store the known
    // occupancy states in a hashmap first and to perform the lookups on the hashmap.
    std::unordered_map<octomap::point3d, bool> voxels;
    for (auto it = octreeToFilter->begin_leafs(); it != octreeToFilter->end_leafs(); ++it)
    {
        const octomap::point3d& coordinates = it.getCoordinate();
        bool occupied = octreeToFilter->isNodeOccupied(*it);
        voxels[coordinates] = occupied;
    }

    // Now we can iterate over all voxels and check their neighboring voxels to find out whether we can keep the voxel
    // or whether we have to discard it. This is the part where the previously created hashmap will help a lot to speed
    // things up as we will now perform many lookups on whether there is occupancy information about neighboring voxels.
    octomap::OcTree* filteredOctree = new octomap::OcTree(leafSize);
    for (auto it = voxels.begin(); it != voxels.end(); ++it)
    {
        const octomap::point3d& coordinates = it->first;
        const bool occupied = it->second;

        if (occupied)
        {
            // Current voxel is occupied. Add it to the filtered octree without any further checks.
            octomap::OcTreeNode* insertedNode = filteredOctree->updateNode(coordinates, true);
            insertedNode->setLogOdds(1.0);
        }
        else
        {
            // Current voxel is free. Check that none of its neighbors is an unknown voxel.
            for (size_t i = 0; i < octreeFilteringNeighborhood.size(); i++)
            {
                // The conversion from coord to key back to coord is needed to ensure that the calculated voxel center
                // point is actually in the center of the voxel. Due to floating point imprecisions the calculated
                // center point may not be exactly in the voxel's center and might need to be ever so slightly adjusted.
                // If this wasn't done, it may happen that some of the neighbors may not be found during lookup even
                // though we have some occupancy information about these voxels stored.
                octomap::point3d neighborCenterPoint = coordinates + octreeFilteringNeighborhood[i];
                neighborCenterPoint = octreeToFilter->keyToCoord(octreeToFilter->coordToKey(neighborCenterPoint));

                if (voxels.find(neighborCenterPoint) == voxels.end())
                {
                    // We have no occupancy information about the neighboring voxel. The current voxel might have been
                    // falsely classified as being free, so we don't add it to the filtered octree.

                    // I know, I know, goto is considered bad. However, we need to continue with the next iteration of
                    // the outer loop (i.e. the one iterating over all leaves of the octree) and according to
                    // https://stackoverflow.com/a/41179682 using a goto statement seems to be the easiest solution in
                    // this case.
                    goto next_loop_iteration;
                }
            }

            // Current voxel is free and does not neighbor any unknown voxel. We can add it to the filtered octree.
            filteredOctree->updateNode(coordinates, false)->setLogOdds(-1.0);
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

VoxelClassificationResult SpatialMapper::classifyVoxelsWithSpatialMapUpdate(octomap::OcTree* currentFrameOctree)
{
    VoxelClassificationResult result;

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

VoxelClassificationResult SpatialMapper::classifyVoxelsWithoutSpatialMapUpdate(octomap::OcTree* currentFrameOctree)
{
    VoxelClassificationResult result;

    spatialMapMutex.lock();
    std::vector<VoxelDiffInfo> diff = calculateOctreeVoxelDiff(staticObjectsOctree, currentFrameOctree);
    spatialMapMutex.unlock();

    for (auto it = diff.begin(); it != diff.end(); it++)
    {
        if (it->type == VoxelDiffType::FREE_OCCUPIED || it->type == VoxelDiffType::UNKNOWN_OCCUPIED)
        {
            // Azim 2012 paper states: "If the transition between these two states for a specific voxel of the grid is
            // such that S_{t-1} = free and S_t = occupied then this is the case when an object is detected on a
            // location previously seen as free space and it is possibly a moving object. We add it to the list of
            // possible dynamic voxels.
            
            // Azim 2012 paper states: "If an object appears at a location which was previously unobserved, then we can
            // say nothing about that object. For such measurements, a priori we will suppose that they are static until
            // later evidences come."
            // As this function doesn't update the global spatial map, we can't add the voxel to the spatial map
            // (indicating that we assumed it being static), so I'll deviate here a bit from the paper by assuming that
            // voxels with no previous occupancy information are dynamic. However, considering that this function should
            // only be used after the spatial map of the environment was already built, it is also safe to say that we
            // will ever receive information about voxels for which we already have previous information so this case
            // should actually (almost) never occur.

            result.dynamicVoxelCenterPoints.push_back(it->coordinates);
        }
        else if (it->type == VoxelDiffType::OCCUPIED_OCCUPIED)
        {
            // Azim 2012 paper states: "If S_{t-1} = occupied and S_t = occupied, it means that an object is observed on
            // a location previously occupied then it probably is static."
            result.staticVoxelCenterPoints.push_back(it->coordinates);
        }
    }

    return result;
}

void SpatialMapper::updateFloorHeight(const std::vector<octomap::point3d>& voxelCenterPoints)
{
    // Check if there is a voxel which is lower than the currently assumed floor height.
    float potentialNewFloorHeight = floorHeight;
    for (auto it = voxelCenterPoints.begin(); it != voxelCenterPoints.end(); it++)
    {
        const float& voxelHeight = it->y();
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
            const float& voxelHeight = it->y();
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

std::vector<octomap::point3d> SpatialMapper::removeFloorVoxels(const std::vector<octomap::point3d>& voxelCenterPointsToFilter)
{
    std::vector<octomap::point3d> nonFloorVoxels;
    float floorVoxelHeight = floorHeight + leafSize * floorRemovalRelativeNoiseHeight + 1e-6;
    for (auto it = voxelCenterPointsToFilter.begin(); it != voxelCenterPointsToFilter.end(); it++)
    {
        const float& voxelHeight = it->y();
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

octomap::OcTree* SpatialMapper::voxelCenterPointsToOctree(const std::vector<octomap::point3d>& voxelCenterPoints)
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
                    clusterCandidateVoxels.push_back(neighbor);
                }
            }
        }

        result.push_back(clusterVoxels);
    }

    return result;
}

std::vector<std::vector<octomap::point3d>> SpatialMapper::removeSmallVoxelClusters(
        const std::vector<std::vector<octomap::point3d>>& voxelClustersToFilter)
{
    std::vector<std::vector<octomap::point3d>> filteredClusters;

    for (const auto& cluster : voxelClustersToFilter)
    {
        if (cluster.size() >= voxelClusteringMinClusterSize)
        {
            filteredClusters.push_back(cluster);
        }
    }

    return filteredClusters;
}

std::pair<std::vector<std::vector<octomap::point3d>>, std::vector<std::vector<octomap::point3d>>>
        SpatialMapper::removeNoiseVoxelClusters(const std::vector<std::vector<octomap::point3d>>& voxelClustersToFilter)
{
    std::vector<std::vector<octomap::point3d>> objectClusters;
    std::vector<std::vector<octomap::point3d>> noiseClusters;

    spatialMapMutex.lock();

    for (const auto& cluster : voxelClustersToFilter)
    {
        // Determine how many voxels with or without static neighbors we have to find in the current cluster in order to
        // determine whether the current cluster contains only sensor noise or some object of interest.
        double clusterSize = static_cast<double>(cluster.size());
        uint32_t staticNeighborThreshold = static_cast<uint32_t>(
                std::round(clusterSize * noiseClusterRemovalStaticNeighborPercentage));
        uint32_t noStaticNeighborThreshold = static_cast<uint32_t>(
                std::round(clusterSize * noiseClusterRemovalNoStaticNeighborPercentage));

        uint32_t numVoxelsWithStaticNeighbors = 0;
        uint32_t numVoxelsWithoutStaticNeighbors = 0;

        // Iterate over all voxels of the current cluster and determine whether they are neighboring a static voxel.
        for (auto voxelIt = cluster.begin(); voxelIt != cluster.end(); ++voxelIt)
        {
            for (size_t i = 0; i < noiseClusterRemovalNeighborhood.size(); i++)
            {
                const octomap::point3d& voxelCenterPoint = *voxelIt;
                octomap::point3d neighborCenterPoint = voxelCenterPoint + noiseClusterRemovalNeighborhood[i];

                octomap::OcTreeNode* neighborNode = staticObjectsOctree->search(neighborCenterPoint);
                if (neighborNode != NULL && staticObjectsOctree->isNodeOccupied(neighborNode)) 
                {
                    // The current voxel has a static neighbor.
                    ++numVoxelsWithStaticNeighbors;
                    if (numVoxelsWithStaticNeighbors >= staticNeighborThreshold)
                    {
                        // The current clusters contains too many voxels with at least one static neighbor, so we can
                        // assume that this cluster contains only sensor noise.
                        noiseClusters.push_back(cluster);

                        // I know, I know, goto is considered bad. However, we need to break the outer loop iterating
                        // over all voxels of the current cluster as we can already safely say that this cluster must
                        // be a cluster containing only sensor noise. According to https://stackoverflow.com/a/41179682
                        // using a goto statement seems to be the easiest solution in this case.
                        goto next_cluster;
                    }

                    // I know, I know, goto is considered bad. However, we need to continue with the next iteration of
                    // the outer loop (i.e. the loop iterating over all voxels of the current cluster) as we only want
                    // to know IF the current voxel has a static neighbor, not how many static neighbors the current
                    // voxel has. According to https://stackoverflow.com/a/41179682 using a goto statement seems to be
                    // the easiest solution in this case.
                    goto next_voxel_neighbor_search;
                }
            }

            ++numVoxelsWithoutStaticNeighbors;
            if (numVoxelsWithoutStaticNeighbors > noStaticNeighborThreshold)
            {
                // The current cluster contains too many voxels without a static neighbor, so we can assume that this
                // cluster contains some object of interest and not only sensor noise.
                objectClusters.push_back(cluster);
                break;
            }

            // Label for jumping to the next loop iteration (or for leaving the loop if we're in the last iteration). As
            // already mentioned, using goto is generally considered bad, but we need this in this case for continuing
            // the current loop iterating over all voxels of the current cluster from an inner loop.
            next_voxel_neighbor_search:;
        }

        // Label for jumping to the next loop iteration (or for leaving the loop if we're in the last iteration). As
        // already mentioned, using goto is generally considered bad, but we need this in this case for continuing
        // the current loop iterating over all clusters from an inner loop.
        next_cluster:;
    }

    spatialMapMutex.unlock();

    return std::make_pair(objectClusters, noiseClusters);
}

void SpatialMapper::addClustersToStaticObjectsMap(const std::vector<std::vector<octomap::point3d>>& voxelClustersToAdd)
{
    std::vector<size_t> indices;
    for (size_t i = 0; i < voxelClustersToAdd.size(); i++)
    {
        indices.push_back(i);
    }

    addClustersToStaticObjectsMap(voxelClustersToAdd, indices);
}

void SpatialMapper::addClustersToStaticObjectsMap(
        const std::vector<std::vector<octomap::point3d>>& clusters, const std::vector<size_t>& indices)
{
    spatialMapMutex.lock();

    // Add all clusters which were determined as containing only sensor noise to the spatial map of static objects.
    for (const auto& index : indices)
    {
        const auto& cluster = clusters[index];
        for (const auto& voxelCenterPoint : cluster)
        {
            octomap::OcTreeNode* insertedNode = staticObjectsOctree->updateNode(voxelCenterPoint, true);
            insertedNode->setLogOdds(1.0);

            possibleDynamicVoxels.erase(voxelCenterPoint);
        }
    }

    spatialMapMutex.unlock();
}

octomap::ColorOcTree* SpatialMapper::createVoxelClusterOctree(
        const std::vector<std::vector<octomap::point3d>>& clusters)
{
    octomap::ColorOcTree* colorizedClusterOctree = new octomap::ColorOcTree(leafSize);
    
    for (int i = 0; i < clusters.size(); i++)
    {
        const std::vector<octomap::point3d>& cluster = clusters[i];

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

std::vector<BoundingBox> SpatialMapper::calculateBoundingBoxes(
        const std::vector<std::vector<octomap::point3d>>& clusters)
{
    std::vector<BoundingBox> boundingBoxes;
    const float halfLeafSize = leafSize / 2.0f;

    for (auto clusterIt = clusters.begin(); clusterIt != clusters.end(); clusterIt++)
    {
        const std::vector<octomap::point3d>& cluster = *clusterIt;

        float minX = cluster[0].x();
        float maxX = cluster[0].x();
        float minY = cluster[0].y();
        float maxY = cluster[0].y();
        float minZ = cluster[0].z();
        float maxZ = cluster[0].z();

        for (int i = 1; i < cluster.size(); i++)
        {
            const float& x = cluster[i].x();
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);

            const float& y = cluster[i].y();
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);

            const float& z = cluster[i].z();
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);
        }

        pcl::PointXYZ min = pcl::PointXYZ(minX - halfLeafSize, minY - halfLeafSize, minZ - halfLeafSize);
        pcl::PointXYZ max = pcl::PointXYZ(maxX + halfLeafSize, maxY + halfLeafSize, maxZ + halfLeafSize);
        boundingBoxes.push_back(BoundingBox(min, max));
    }

    return boundingBoxes;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> SpatialMapper::extractPointsCorrespondingToBoundingBoxes(
        const std::vector<BoundingBox>& boundingBoxes, pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterClouds;

    for (int i = 0; i < boundingBoxes.size(); i++)
    {
        const BoundingBox& boundingBox = boundingBoxes[i];

        pcl::CropBox<pcl::PointXYZI> cropbox;
        cropbox.setMin(Eigen::Vector4f(boundingBox.min.x, boundingBox.min.y, boundingBox.min.z, 1.0f));
        cropbox.setMax(Eigen::Vector4f(boundingBox.max.x, boundingBox.max.y, boundingBox.max.z, 1.0f));
        cropbox.setInputCloud(points);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsInBoundingBox (new pcl::PointCloud<pcl::PointXYZI>());
        cropbox.filter(*pointsInBoundingBox);
        clusterClouds.push_back(pointsInBoundingBox);
    }

    return clusterClouds;
}


std::vector<pcl::PointXYZ> SpatialMapper::calculateCentroids(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusterClouds)
{
    std::vector<pcl::PointXYZ> centroids;
    
    for (int i = 0; i < clusterClouds.size(); i++)
    {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& clusterCloud = clusterClouds[i];

        Eigen::Vector4f centroidVec4f;
        pcl::compute3DCentroid(*clusterCloud, centroidVec4f);

        pcl::PointXYZ centroidPoint;
        centroidPoint.getVector4fMap() = centroidVec4f;
        centroids.push_back(centroidPoint);
    }

    return centroids;
}

std::vector<ClassificationResult> SpatialMapper::classifyVoxelClusters(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusterClouds,
        const std::vector<pcl::PointXYZ>& clusterCentroids,
        geometry_msgs::Point sensorPosition)
{
    std::vector<ClassificationResult> classificationResults;

    if (clusterClouds.size() == 0)
        return classificationResults;

    object3d_detector::ClassifyClusters classificationMsg;
    for (auto& clusterCloud : clusterClouds)
    {
        sensor_msgs::PointCloud2 clusterCloudMsg;
        pcl::toROSMsg(*clusterCloud, clusterCloudMsg);
        classificationMsg.request.clusters.push_back(clusterCloudMsg);
    }
    classificationMsg.request.clusterCentroids.header.seq = octomapSequenceNumber;
    classificationMsg.request.clusterCentroids.header.stamp = ros::Time::now();
    classificationMsg.request.clusterCentroids.header.frame_id = "hololens_world";
    for (const auto& clusterCentroid : clusterCentroids)
    {
        geometry_msgs::Pose pose;
        pose.position.x = clusterCentroid.x;
        pose.position.y = clusterCentroid.y;
        pose.position.z = clusterCentroid.z;
        pose.orientation.w = 1.0;

        classificationMsg.request.clusterCentroids.poses.push_back(pose);
    }
    classificationMsg.request.sensorPosition = sensorPosition;

    bool classificationSuccess = clusterClassifierService.call(classificationMsg);

    if (classificationSuccess)
    {
        for (size_t i = 0; i < clusterClouds.size(); i++)
        {
            const object3d_detector::ClassificationResult& clusterResult
                    = classificationMsg.response.classificationResults[i];

            const ObjectClass objectClass = ObjectClass(clusterResult.objectClass);
            const float& probability = clusterResult.probability;
            const int64_t& trackingId = clusterResult.trackingId;
            classificationResults.push_back(ClassificationResult(objectClass, probability, trackingId));
        }
    }
    else
    {
        for (size_t i = 0; i < clusterClouds.size(); i++)
        {
            classificationResults.push_back(ClassificationResult());
        }
    }

    return classificationResults;
}

std::vector<size_t> SpatialMapper::selectClustersByClass(
        const std::vector<ClassificationResult>& classificationResults,
        const std::unordered_set<ObjectClass>& classesToSelect)
{
    std::vector<size_t> indices;

    for (size_t i = 0; i < classificationResults.size(); i++)
    {
        const ObjectClass& classOfCurrentCluster = classificationResults[i].objectClass;
        if (classesToSelect.find(classOfCurrentCluster) != classesToSelect.end())
        {
            indices.push_back(i);
        }
    }

    return indices;
}

void SpatialMapper::publishBoundingBoxes(
        const std::vector<BoundingBox>& boundingBoxes,
        const std::vector<ClassificationResult>& classificationResults,
        ros::Publisher& publisher,
        const ros::Time& timestamp)
{
    std::vector<size_t> indices;
    for (size_t i = 0; i < boundingBoxes.size(); i++)
    {
        indices.push_back(i);
    }

    publishBoundingBoxes(boundingBoxes, classificationResults, indices, publisher, timestamp);
}

void SpatialMapper::publishBoundingBoxes(
        const std::vector<BoundingBox>& boundingBoxes,
        const std::vector<ClassificationResult>& classificationResults,
        const std::vector<size_t>& indices,
        ros::Publisher& publisher,
        const ros::Time& timestamp)
{
    if (indices.size() == 0)
        return;

    visualization_msgs::MarkerArray markers;

    for (const auto& index : indices)
    {
        const BoundingBox& boundingBox = boundingBoxes[index];

        // The following blocks of code used to generate the marker points are somewhat copied (and slightly modified)
        // from object3d_detector (published in Yan2020online / https://github.com/yzrobot/online_learning)
        visualization_msgs::Marker marker;
        marker.header.seq = octomapSequenceNumber;
        marker.header.stamp = timestamp;
        marker.header.frame_id = "hololens_world";

        marker.ns = "octomap_based_object_detection";
        marker.id = index;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = 0;

        geometry_msgs::Point p[24];
        p[0].x = boundingBox.max.x; p[0].y = boundingBox.max.y; p[0].z = boundingBox.max.z;
        p[1].x = boundingBox.min.x; p[1].y = boundingBox.max.y; p[1].z = boundingBox.max.z;
        p[2].x = boundingBox.max.x; p[2].y = boundingBox.max.y; p[2].z = boundingBox.max.z;
        p[3].x = boundingBox.max.x; p[3].y = boundingBox.min.y; p[3].z = boundingBox.max.z;
        p[4].x = boundingBox.max.x; p[4].y = boundingBox.max.y; p[4].z = boundingBox.max.z;
        p[5].x = boundingBox.max.x; p[5].y = boundingBox.max.y; p[5].z = boundingBox.min.z;
        p[6].x = boundingBox.min.x; p[6].y = boundingBox.min.y; p[6].z = boundingBox.min.z;
        p[7].x = boundingBox.max.x; p[7].y = boundingBox.min.y; p[7].z = boundingBox.min.z;
        p[8].x = boundingBox.min.x; p[8].y = boundingBox.min.y; p[8].z = boundingBox.min.z;
        p[9].x = boundingBox.min.x; p[9].y = boundingBox.max.y; p[9].z = boundingBox.min.z;
        p[10].x = boundingBox.min.x; p[10].y = boundingBox.min.y; p[10].z = boundingBox.min.z;
        p[11].x = boundingBox.min.x; p[11].y = boundingBox.min.y; p[11].z = boundingBox.max.z;
        p[12].x = boundingBox.min.x; p[12].y = boundingBox.max.y; p[12].z = boundingBox.max.z;
        p[13].x = boundingBox.min.x; p[13].y = boundingBox.max.y; p[13].z = boundingBox.min.z;
        p[14].x = boundingBox.min.x; p[14].y = boundingBox.max.y; p[14].z = boundingBox.max.z;
        p[15].x = boundingBox.min.x; p[15].y = boundingBox.min.y; p[15].z = boundingBox.max.z;
        p[16].x = boundingBox.max.x; p[16].y = boundingBox.min.y; p[16].z = boundingBox.max.z;
        p[17].x = boundingBox.max.x; p[17].y = boundingBox.min.y; p[17].z = boundingBox.min.z;
        p[18].x = boundingBox.max.x; p[18].y = boundingBox.min.y; p[18].z = boundingBox.max.z;
        p[19].x = boundingBox.min.x; p[19].y = boundingBox.min.y; p[19].z = boundingBox.max.z;
        p[20].x = boundingBox.max.x; p[20].y = boundingBox.max.y; p[20].z = boundingBox.min.z;
        p[21].x = boundingBox.min.x; p[21].y = boundingBox.max.y; p[21].z = boundingBox.min.z;
        p[22].x = boundingBox.max.x; p[22].y = boundingBox.max.y; p[22].z = boundingBox.min.z;
        p[23].x = boundingBox.max.x; p[23].y = boundingBox.min.y; p[23].z = boundingBox.min.z;

        for(int i = 0; i < 24; i++)
            marker.points.push_back(p[i]);

        marker.color = objectClassColors[classificationResults[index].objectClass];

        // Contrary to what one would assume, this does not scale the points along the x-axis. Instead, it defines the
        // width to use when rendering the lines. A value of 0.02 therefore indicates that a line should have a width of
        // 0.02 units.
        marker.scale.x = 0.02;

        marker.lifetime = ros::Duration(0.25);

        markers.markers.push_back(marker);
    }

    publisher.publish(markers);
}

void SpatialMapper::publishCentroids(
        const std::vector<pcl::PointXYZ>& centroids,
        ros::Publisher& publisher,
        const ros::Time& timestamp)
{
    std::vector<size_t> indices;
    for (size_t i = 0; i < centroids.size(); i++)
    {
        indices.push_back(i);
    }

    publishCentroids(centroids, indices, publisher, timestamp);
}

void SpatialMapper::publishCentroids(
        const std::vector<pcl::PointXYZ>& centroids,
        const std::vector<size_t>& indices,
        ros::Publisher& publisher,
        const ros::Time& timestamp)
{
    geometry_msgs::PoseArray poseArray;
    poseArray.header.seq = octomapSequenceNumber;
    poseArray.header.stamp = timestamp;
    poseArray.header.frame_id = "hololens_world";

    for (const auto& index : indices)
    {
        const pcl::PointXYZ& centroid = centroids[index];

        geometry_msgs::Pose pose;
        pose.position.x = centroid.x;
        pose.position.y = centroid.y;
        pose.position.z = centroid.z;
        pose.orientation.w = 1.0;

        poseArray.poses.push_back(pose);
    }

    publisher.publish(poseArray);
}

void SpatialMapper::setUpdateSpatialMap(bool _updateSpatialMap)
{
    updateSpatialMap = _updateSpatialMap;
    ROS_INFO("Set updateSpatialMap to %s.", updateSpatialMap ? "true" : "false");
}

void SpatialMapper::clearSpatialMap()
{
    ROS_INFO("Clearing spatial map...");

    spatialMapMutex.lock();
    delete staticObjectsOctree;
    staticObjectsOctree = new octomap::OcTree(leafSize);
    spatialMapMutex.unlock();

    octomapSequenceNumber++;
    publishOctree(staticObjectsOctree, octomapStaticObjectsPublisher, ros::Time::now());
}