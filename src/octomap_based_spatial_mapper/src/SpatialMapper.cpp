#include "SpatialMapper.h"

SpatialMapper::SpatialMapper(ros::NodeHandle n)
{
    ROS_INFO("Creating SpatialMapper...");

    // TODO: leafSize should be a configurable parameter.
    // TODO: maxRange should be fetched from the hololens_depth_data_receiver package.
    leafSize = 0.1;
    maxRange = -1.0;

    previousFrameOctree = new octomap::OcTree(leafSize);
    staticObjectsOctree = new octomap::OcTree(leafSize);

    octomapCurrentFramePublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_CURRENT_FRAME_TOPIC, 10);
    octomapStaticObjectsPublisher = n.advertise<octomap_msgs::Octomap>(OCTOMAP_STATIC_OBJECTS_TOPIC, 10);
}

SpatialMapper::~SpatialMapper()
{
    delete previousFrameOctree;
    delete staticObjectsOctree;
}

void SpatialMapper::handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    octomap::OcTree* currentFrameOctree = pointCloudToOctree(msg);
    //octomap::OcTree* dynamicVoxels = detectDynamicVoxels(previousFrameOctree, currentFrameOctree);
    updateStaticObjectsOctree(currentFrameOctree);

    // Publish the results.
    ros::Time time = ros::Time::now();
    octomapSequenceNumber++;
    publishOctree(currentFrameOctree, octomapCurrentFramePublisher, time);
    publishOctree(staticObjectsOctree, octomapStaticObjectsPublisher, time);
    // TODO: The following line of code exists just for debugging purposes. It needs to be removed later on.
    //publishOctree(dynamicVoxels, octomapStaticObjectsPublisher, time);

    delete previousFrameOctree;
    previousFrameOctree = currentFrameOctree;
    //delete dynamicVoxels;
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

std::vector<VoxelDiffInfo> SpatialMapper::calculateOctreeVoxelDiff(
        octomap::OcTree* oldOctree,
        octomap::OcTree* newOctree,
        bool returnOnlyChanges)
{
    // Some parts of the following code on how to traverse an octree whilst comparing it with another octree (e.g. for
    // merging two octrees) were adopted from https://github.com/OctoMap/octomap/issues/227#issuecomment-571336801

    // The new octree needs to be expanded so that we can traverse all its leaves.
    newOctree->expand();

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

    // The new octree can be pruned again to save space.
    newOctree->prune();

    return result;
}

octomap::OcTree* SpatialMapper::detectDynamicVoxels(
        octomap::OcTree* previousFrameOctree,
        octomap::OcTree* currentFrameOctree)
{
    std::vector<VoxelDiffInfo> diff = calculateOctreeVoxelDiff(previousFrameOctree, currentFrameOctree);

    for (auto it = diff.begin(); it != diff.end(); it++)
    {
        if (it->type == VoxelDiffType::FREE_OCCUPIED)
        {
            // Azim 2012 paper states: "If the transition between these two states for a specific voxel of the grid is
            // such that S_{t-1} = free and S_t = occupied then this is the case when an object is detected on a
            // location previously seen as free space and it is possibly a moving object. We add it to the list of
            // possible dynamic voxels."
            possibleDynamicVoxels[it->coordinates] = PossibleDynamicVoxelInfo(dynamicVoxelsUpdateSequenceNumber);
        }
        else if (it->type == VoxelDiffType::OCCUPIED_FREE || it->type == VoxelDiffType::FREE_FREE)
        {
            // Azim 2012 paper states: "In contrary, if S_{t-1} = occupied and S_t = free, it menas that the location
            // which was previously observed as occupied is free now. This can possibly be caused by a missed detection
            // by the sensor or it was a voxel occupied by a dynamic object which may have displaced now. We search this
            // voxel in our list of dynamic voxels maintained from previous scans. If it is found, we wait for the next
            // few scans instead of removing it from the dynamic voxels list immediately. If it is observed as free in
            // the next scans was well, then we delete it from the list."
            auto search = possibleDynamicVoxels.find(it->coordinates);
            if (search != possibleDynamicVoxels.end())
            {
                search->second.freeCounter++;
                search->second.lastUpdate = dynamicVoxelsUpdateSequenceNumber;

                // TODO: 5 should be a configurable parameter.
                if (search->second.freeCounter >= 5)
                {
                    possibleDynamicVoxels.erase(it->coordinates);
                }
            }
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
        }
    }

    // The list of possible dynamic voxels might grow over time if there are no further observations for voxels in that
    // list. Maybe remove voxels that are too long in that list again? This is not mentioned in the paper...
    // TODO: 10 should be a configurable parameter.
    uint32_t removeIfLastUpdateBefore = dynamicVoxelsUpdateSequenceNumber - 10;
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

    octomap::OcTree* dynamicVoxels = new octomap::OcTree(leafSize);
    for (auto it = possibleDynamicVoxels.begin(); it != possibleDynamicVoxels.end(); it++)
    {
        dynamicVoxels->updateNode(it->first, true);
    }
    return dynamicVoxels;
}

void SpatialMapper::updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree)
{
    std::vector<VoxelDiffInfo> diff = calculateOctreeVoxelDiff(staticObjectsOctree, currentFrameOctree);

    for (auto it = diff.begin(); it != diff.end(); it++)
    {
        if (it->type == VoxelDiffType::FREE_OCCUPIED)
        {
            // Azim 2012 paper states: "If the transition between these two states for a specific voxel of the grid is
            // such that S_{t-1} = free and S_t = occupied then this is the case when an object is detected on a
            // location previously seen as free space and it is possibly a moving object. We add it to the list of
            // possible dynamic voxels."
            possibleDynamicVoxels[it->coordinates] = PossibleDynamicVoxelInfo(dynamicVoxelsUpdateSequenceNumber);

            // What if the initial observation about the voxel being free was a measurement error? This is not mentioned
            // in the paper...
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

            // TODO: 5 should be a configurable parameter.
            if (dynamicVoxelInfo.freeCounter >= 5)
            {
                possibleDynamicVoxels.erase(it->coordinates);

                it->nodeOldOctree->setValue(-1000.0);
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
        }
        else if (it->type == VoxelDiffType::UNKNOWN_FREE || it->type == VoxelDiffType::UNKNOWN_OCCUPIED)
        {
            // Azim 2012 paper states: "If an object appears at a location which was previously unobserved, then we can
            // say nothing about that object. For such measurements, a priori we will suppose that they are static until
            // later evidences come."
            octomap::OcTreeNode* insertedNode = staticObjectsOctree->updateNode(it->coordinates, true);
            insertedNode->setLogOdds(it->nodeNewOctree->getLogOdds());
        }
    }

    // The list of possible dynamic voxels might grow over time if there are no further observations for voxels in that
    // list. Maybe remove voxels that are too long in that list again? This is not mentioned in the paper...
    // TODO: 10 should be a configurable parameter.
    uint32_t removeIfLastUpdateBefore = dynamicVoxelsUpdateSequenceNumber - 10;
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