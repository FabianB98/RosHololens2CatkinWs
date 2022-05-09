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
    octomap::OcTree* dynamicVoxels = detectDynamicVoxels(previousFrameOctree, currentFrameOctree);
    //updateStaticObjectsOctree(currentFrameOctree);

    // Publish the results.
    ros::Time time = ros::Time::now();
    octomapSequenceNumber++;
    publishOctree(currentFrameOctree, octomapCurrentFramePublisher, time);
    //publishOctree(staticObjectsOctree, octomapStaticObjectsPublisher, time);
    // TODO: The following line of code exists just for debugging purposes. It needs to be removed later on.
    publishOctree(dynamicVoxels, octomapStaticObjectsPublisher, time);

    delete previousFrameOctree;
    previousFrameOctree = currentFrameOctree;
    delete dynamicVoxels;
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

octomap::OcTree* SpatialMapper::detectDynamicVoxels(
        octomap::OcTree* previousFrameOctree,
        octomap::OcTree* currentFrameOctree)
{
    octomap::OcTree* dynamicVoxels = new octomap::OcTree(leafSize);

    // Some parts of the following code on how to traverse an octree whilst comparing it with another octree (e.g. for
    // merging two octrees) were adopted from https://github.com/OctoMap/octomap/issues/227#issuecomment-571336801

    // The octree of the current frame needs to be expanded so that we can traverse all its leaves.
    currentFrameOctree->expand();

    // Iterate over all voxels for which we have acquired some information in the current frame.
    for (octomap::OcTree::leaf_iterator it = currentFrameOctree->begin_leafs(); it != currentFrameOctree->end_leafs(); ++it)
    {
        // Check whether the octree of the previous frame already contains some information about the node/voxel of the
        // octree with information obtained in the current frame which we're currently iterating over.
        octomap::point3d coordinates = it.getCoordinate();
        octomap::OcTreeNode* nodeInPreviousFrameOctree = previousFrameOctree->search(coordinates);
        if (nodeInPreviousFrameOctree != NULL) {
            // There already is a node/voxel in the global spatial map. We therefore have occupancy information from
            // some previous frame to which we can compare the new occupancy information against.
            bool currentFrameOccupied = currentFrameOctree->isNodeOccupied(*it);
            bool previousFrameOccupied = previousFrameOctree->isNodeOccupied(nodeInPreviousFrameOctree);

            if (!previousFrameOccupied && currentFrameOccupied)
            {
                // The current voxel was free in the previous frame, but it is now observed as being occupied. We might
                // have observed a moving object entering this voxel.

                // Add the current voxel to the list of possible dynamic voxels.
                possibleDynamicVoxels[coordinates] = 0;
            }
            else if (previousFrameOccupied && !currentFrameOccupied)
            {
                // The current voxel was occupied in the previous frame, but it is now observed as being free. This
                // might be caused by a missed detection or there was an object which was previously assumed to be
                // static when in fact it is a dynamic object which has moved since then.

                // Search the list of possible dynamic voxels for this voxel.
                auto search = possibleDynamicVoxels.find(coordinates);

                // If the voxel is found in the list of possible dynamic voxels, increment a counter how often we have
                // seen it as free.
                if (search != possibleDynamicVoxels.end())
                {
                    search->second++;

                    // If that counter exceeds some threshold, remove the voxel from the list of possible dynamic
                    // voxels.
                    // TODO: 5 should be a configurable parameter.
                    if (search->second >= 5)
                    {
                        possibleDynamicVoxels.erase(coordinates);
                    }
                }
            }
            else if (previousFrameOccupied && currentFrameOccupied)
            {
                // The current voxel was occupied in the previous frame and is now also observed as being occupied. It
                // is therefore probably static.

                // Remove the voxel from the list of dynamic voxels in case it is on that list.
                auto search = possibleDynamicVoxels.find(coordinates);
                if (search != possibleDynamicVoxels.end())
                {
                    possibleDynamicVoxels.erase(coordinates);
                }
            }

            // TODO: The list of possible dynamic voxels might grow over time if there are no further observations for
            // voxels in that list. Maybe remove voxels that are too long in that list again? This is not mentioned in
            // the paper...
        }

        for (auto it = possibleDynamicVoxels.begin(); it != possibleDynamicVoxels.end(); it++)
        {
            dynamicVoxels->updateNode(it->first, true);
        }
    }

    // The octree of the current frame can be pruned again to save space and time when converting it to a ROS message.
    currentFrameOctree->prune();

    return dynamicVoxels;
}

void SpatialMapper::updateStaticObjectsOctree(octomap::OcTree* currentFrameOctree)
{
    // Some parts of the following code on how to traverse an octree whilst comparing it with another octree (e.g. for
    // merging two octrees) were adopted from https://github.com/OctoMap/octomap/issues/227#issuecomment-571336801

    // The octree of the current frame needs to be expanded so that we can traverse all its leaves.
    currentFrameOctree->expand();

    // Iterate over all voxels for which we have acquired some information in the current frame.
    for (octomap::OcTree::leaf_iterator it = currentFrameOctree->begin_leafs(); it != currentFrameOctree->end_leafs(); ++it)
    {
        // Check whether the global spatial map already contains some information about the node/voxel of the octree
        // with information obtained in the current frame which we're currently iterating over.
        octomap::point3d coordinates = it.getCoordinate();
        octomap::OcTreeNode* nodeInStaticObjectsOctree = staticObjectsOctree->search(coordinates);
        if (nodeInStaticObjectsOctree != NULL) {
            // There already is a node/voxel in the global spatial map. We therefore have occupancy information from
            // some previous frame to which we can compare the new occupancy information against.
            bool currentFrameOccupied = currentFrameOctree->isNodeOccupied(*it);
            bool staticObjectsOccupied = staticObjectsOctree->isNodeOccupied(nodeInStaticObjectsOctree);

            if (currentFrameOccupied == staticObjectsOccupied)
            {
                // No change observed between the current frame and the previous information. The current voxel is very
                // likely a static object or a static free space. It is therefore sufficient to simply update the
                // occupancy probability of the current voxel/node.
                octomap::OcTreeKey nodeKey = staticObjectsOctree->coordToKey(coordinates);
                staticObjectsOctree->updateNode(nodeKey, it->getLogOdds());
            }
            else if (!staticObjectsOccupied && currentFrameOccupied)
            {
                // The current voxel was free in some previous frame, but it is now observed as being occupied. We might
                // have observed a moving object entering this voxel.

                // TODO: Add the current voxel to the list of possible dynamic voxels.

                // What if the initial observation about the voxel being free was a measurement error? This is not
                // mentioned in the paper...
            }
            else if (staticObjectsOccupied && !currentFrameOccupied)
            {
                // The current voxel was occupied in some previous frame, but it is now observed as being free. This
                // might be caused by a missed detection or there was an object which was previously assumed to be
                // static when in fact it is a dynamic object which has moved since then.

                // TODO: Search the list of dynamic voxels for this voxel.
                // What if we only observed the voxel in the first frame as being occupied and after that always as free
                // (e.g. due to a measurement error)? In this case it will never be added to the list of possible
                // dynamic voxels meaning that it will never be marked as free even though it is free... This case is
                // not mentioned in the paper...

                // TODO: If the voxel is found in the list of possible dynamic voxels, increment a counter how often we
                // have seen it as free.

                // TODO: If that counter exceeds some threshold, mark the voxel as free and remove it from the list of
                // possible dynamic voxels.
            }

            // TODO: The list of possible dynamic voxels might grow over time if there are no further observations for
            // voxels in that list. Maybe remove voxels that are too long in that list again? This is not mentioned in
            // the paper...
        }
        else
        {
            // We don't have any information about the current voxel from any previous frame. We'll therefore assume
            // that the current voxel is static until we're proven wrong by future observations. The node to be inserted
            // into the global spatial map will have the same probability as the corresponding node in the octree of the
            // current frame.
            octomap::OcTreeNode* newNode = staticObjectsOctree->updateNode(coordinates, true);
            newNode->setLogOdds(it->getLogOdds());
        }
    }

    // The octree of the current frame can be pruned again to save space and time when converting it to a ROS message.
    currentFrameOctree->prune();
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