#include "FasterDepthDataReceiver.h"

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

#include <pcl_conversions/pcl_conversions.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                       SWITCHES FOR CHANGING THE DEFAULT BEHAVIOUR OF THE DEPTH DATA RECEIVER                       //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches for whether short throw and/or long throw depth frames should be used for calculating the point cloud.
// Should short throw depth frames be used for calculating the point cloud?
#define USE_SHORT_THROW false
// Should long throw depth frames be used for calculating the point cloud?
#define USE_LONG_THROW true

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                           DEFAULT DEPTH RANGES (TOTAL AND RELIABLE) OF THE DEPTH SENSORS                           //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// The depth sensor intrinsics of the short throw depth sensor.
#define SHORT_THROW_MIN_DEPTH 0.2f          // The minimum depth value which the sensor can report.
#define SHORT_THROW_MIN_RELIABLE_DEPTH 0.2f // The minimum depth value to use. Smaller depth values will be discarded.
#define SHORT_THROW_MAX_RELIABLE_DEPTH 1.0f // The maximum depth value to use. Greater depth values will be discarded.
#define SHORT_THROW_MAX_DEPTH 1.0f          // The maximum depth value which the sensor can report.

// The depth sensor intrinsics of the long throw depth sensor.
#define LONG_THROW_MIN_DEPTH 0.2f           // The minimum depth value which the sensor can report.
#define LONG_THROW_MIN_RELIABLE_DEPTH 0.75f // The minimum depth value to use. Smaller depth values will be discarded.
#define LONG_THROW_MAX_RELIABLE_DEPTH 8.0f  // The maximum depth value to use. Greater depth values will be discarded.
#define LONG_THROW_MAX_DEPTH 16.0f          // The maximum depth value which the sensor can report.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                      DEFAULT HYPER PARAMETERS RELATED TO REMOVAL OF NOISY VALUES AND OUTLIERS                      //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches and hyper parameters for clustering of similar pixels in received depth maps.
#define PIXEL_CLUSTERING_USE_PCL true                       // Whether PCL clustering or the custom region growing should be used.
#define PIXEL_CLUSTERING_NEIGHBORING_PIXEL_DISTANCE 2.5     // The euclidean distance between "neighboring" pixels.
#define PIXEL_CLUSTERING_ABSOLUTE_DEPTH_VALUE_SIMILARITY 50 // The maximum depth difference for pixels to be in the same cluster.
#define PIXEL_CLUSTERING_MIN_CLUSTER_SIZE 400               // Clusters with less pixels than this value will be discarded.

// Parameters for the voxel grid filter used for downsampling.
#define DOWNSAMPLING_LEAF_SIZE 0.01f    // At most one point allowed in a voxel within this edge length.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Well, this is kind of a horrible solution to an issue I don't know how to solve otherwise...
// For reference why this variable is declared as a static variable instead of a member variable: PCL's conditional
// euclidean clustering takes a static callback for deciding whether two points meet the clustering condition. However,
// in this callback we have to compare the depth values of the two points to check whether they are close enough to each
// other, which in term requires access to the threshold stored as a member variable in the FasterDepthDataReceiver.
// As the callback has to be a static callback (i.e. it can't be a function inside FasterDepthDataReceiver), it also
// can't access the required threshold value... As workaround to this issue I've decided to store a global copy of the
// required threshold value here in this global variable. However, it has to be kept in mind that this comes with the
// drawback that all instances of FasterDepthDataReceivers will share the same clustering threshold if there are ever
// multiple FasterDepthDataReceivers active in the same ROS node. Fortunately, this drawback should never be of any
// concern as I can't think of any situation in which one could ever want to have more than one DepthDataReceiver at
// the same time.
float pixelClusteringDepthValueSimilarity;

FasterDepthDataReceiver::FasterDepthDataReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating FasterDepthDataReceiver...");

    // Initialize all switches as specified by parameters (or their default value).
    n.param("useShortThrow", useShortThrow, USE_SHORT_THROW);
    n.param("useLongThrow", useLongThrow, USE_LONG_THROW);

    // Initialize all sensor intrinsics as specified by parameters (or their default value).
    n.param("shortThrowMinDepth", shortThrowMinDepth, SHORT_THROW_MIN_DEPTH);
    n.param("shortThrowMinReliableDepth", shortThrowMinReliableDepth, SHORT_THROW_MIN_RELIABLE_DEPTH);
    n.param("shortThrowMaxReliableDepth", shortThrowMaxReliableDepth, SHORT_THROW_MAX_RELIABLE_DEPTH);
    n.param("shortThrowMaxDepth", shortThrowMaxDepth, SHORT_THROW_MAX_DEPTH);
    n.param("longThrowMinDepth", longThrowMinDepth, LONG_THROW_MIN_DEPTH);
    n.param("longThrowMinReliableDepth", longThrowMinReliableDepth, LONG_THROW_MIN_RELIABLE_DEPTH);
    n.param("longThrowMaxReliableDepth", longThrowMaxReliableDepth, LONG_THROW_MAX_RELIABLE_DEPTH);
    n.param("longThrowMaxDepth", longThrowMaxDepth, LONG_THROW_MAX_DEPTH);

    // Initialize all hyper parameters for clustering of similar pixels in received depth maps as specified by
    // parameters (or their default value).
    n.param("pixelClusteringUsePcl", pixelClusteringUsePcl, PIXEL_CLUSTERING_USE_PCL);
    n.param("pixelClusteringNeighboringPixelDistance", pixelClusteringNeighboringPixelDistance, PIXEL_CLUSTERING_NEIGHBORING_PIXEL_DISTANCE);
    n.param("pixelClusteringAbsoluteDepthValueSimilarity", pixelClusteringAbsoluteDepthValueSimilarity, PIXEL_CLUSTERING_ABSOLUTE_DEPTH_VALUE_SIMILARITY);
    n.param("pixelClusteringMinClusterSize", pixelClusteringMinClusterSize, PIXEL_CLUSTERING_MIN_CLUSTER_SIZE);
    pixelClusteringDepthValueSimilarity = static_cast<float>(pixelClusteringAbsoluteDepthValueSimilarity);
    pixelClusteringSquaredDepthValueSimilarity = pixelClusteringAbsoluteDepthValueSimilarity * pixelClusteringAbsoluteDepthValueSimilarity;
    pixelClusteringNeighborhood = initializeEuclideanDistanceNeighborhood(pixelClusteringNeighboringPixelDistance);

    // Initialize all hyper parameters for downsampling as specified by parameters (or their default value).
    n.param("downsamplingLeafSize", downsamplingLeafSize, DOWNSAMPLING_LEAF_SIZE);

    // Advertise the topics to which the results will be published.
    shortThrowImagePublisher = n.advertise<sensor_msgs::Image>(SHORT_THROW_IMAGE_TOPIC, 10);
    longThrowImagePublisher = n.advertise<sensor_msgs::Image>(LONG_THROW_IMAGE_TOPIC, 10);
    shortThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(SHORT_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10);
    longThrowPointCloudWorldSpacePublisher = n.advertise<sensor_msgs::PointCloud2>(LONG_THROW_POINT_CLOUD_WORLD_SPACE_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);
    shortThrowPointCloudFramePublisher = n.advertise<hololens_depth_data_receiver_msgs::PointCloudFrame>(SHORT_THROW_POINT_CLOUD_FRAME_TOPIC, 10);
    longThrowPointCloudFramePublisher = n.advertise<hololens_depth_data_receiver_msgs::PointCloudFrame>(LONG_THROW_POINT_CLOUD_FRAME_TOPIC, 10);

    // Define the colors to use for coloring pixel clusters. I'm just going to assume that there will never be more
    // than 16 clusters in one depth image. If there will ever be more clusters than that, the assigned colors will
    // wrap around (i.e. the 17th cluster will have the same color as the first cluster, the 18th cluster will have the
    // same color as the second cluster, and so on). If that will ever be an issue, it is sufficient to add more colors
    // to this list. On the other hand, the human eye can only distinguish between a limited amount of colors easily,
    // and the fact that the colors defined in this list will be multiplied by the depth perceived at the corresponding
    // pixels in the depth map only increases the amount of color variations seen in the published depth image...
    pixelClusterColors =
            {
                {1.0, 0.0, 0.0},    // Red
                {1.0, 0.5, 0.0},    // Orange
                {1.0, 1.0, 0.0},    // Yellow
                {0.5, 1.0, 0.0},    // Chartreuse / Greenish yellow
                {0.0, 1.0, 0.0},    // Lime green
                {0.0, 1.0, 0.5},    // Spring green
                {0.0, 1.0, 1.0},    // Cyan
                {0.0, 0.5, 1.0},    // Light blue
                {0.0, 0.0, 1.0},    // Blue
                {0.5, 0.0, 1.0},    // Blueish magenta
                {1.0, 0.0, 1.0},    // Magenta
                {1.0, 0.0, 0.5},    // Redish magenta
                {0.5, 1.0, 1.0},    // Cyanish white
                {1.0, 0.5, 1.0},    // Magentaish white
                {1.0, 1.0, 0.5},    // Yellowish white
                {1.0, 1.0, 1.0}     // White
            };
}

std::vector<Pixel> FasterDepthDataReceiver::initializeEuclideanDistanceNeighborhood(double neighborDistance)
{
    std::vector<Pixel> neighborhood;

    double squaredNeighborDistance = neighborDistance * neighborDistance;
    int halfNeighborhoodSize = static_cast<int>(ceil(neighborDistance));
    for (int u = -halfNeighborhoodSize; u <= halfNeighborhoodSize; u++)
    {
        for (int v = -halfNeighborhoodSize; v <= halfNeighborhoodSize; v++)
        {
            // Don't add the centermost pixel (i.e. (0, 0)) to its neighborhood.
            bool isCentermostPixel = u == 0 && v == 0;

            double squaredDistanceToCenter = u * u + v * v;
            bool satisfiesEuclideanDistance = squaredDistanceToCenter <= squaredNeighborDistance;

            if (satisfiesEuclideanDistance && !isCentermostPixel)
            {
                neighborhood.push_back(Pixel(u, v));
            }
        }
    }

    ROS_INFO("Calculated neighborhood with neighbor distance of %f contains %lu pixels.", neighborDistance,
            neighborhood.size());

    return neighborhood;
}

void FasterDepthDataReceiver::handleShortThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    if (useShortThrow)
        handleDepthFrame(msg, shortThrowDirectionLookupTable, shortThrowMinDepth, shortThrowMinReliableDepth,
                shortThrowMaxReliableDepth, shortThrowMaxDepth, shortThrowImagePublisher,
                shortThrowPointCloudWorldSpacePublisher, shortThrowPointCloudFramePublisher,
                &shortThrowSequenceNumber);
}

void FasterDepthDataReceiver::handleLongThrowDepthFrame(const hololens_msgs::DepthFrame::ConstPtr& msg)
{
    if (useLongThrow)
        handleDepthFrame(msg, longThrowDirectionLookupTable, longThrowMinDepth, longThrowMinReliableDepth,
                longThrowMaxReliableDepth, longThrowMaxDepth, longThrowImagePublisher,
                longThrowPointCloudWorldSpacePublisher, longThrowPointCloudFramePublisher,
                &longThrowSequenceNumber);
}

void FasterDepthDataReceiver::handleShortThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    handlePixelDirections(msg, &shortThrowDirectionLookupTable);
}

void FasterDepthDataReceiver::handleLongThrowPixelDirections(const hololens_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    handlePixelDirections(msg, &longThrowDirectionLookupTable);
}

void FasterDepthDataReceiver::handlePixelDirections(
        const hololens_msgs::PixelDirections::ConstPtr& pixelDirectionsMsg,
        std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>* pixelDirectionsLookupTableTarget)
{
    uint32_t max_u = pixelDirectionsMsg->pixelDirections.at(0).u;
    uint32_t max_v = pixelDirectionsMsg->pixelDirections.at(0).v;

    for (uint32_t i = 0; i < pixelDirectionsMsg->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirectionsMsg->pixelDirections[i];
        if (dir.u > max_u) max_u = dir.u;
        if (dir.v > max_v) max_v = dir.v;
    }

    uint32_t width = max_u + 1;
    uint32_t height = max_v + 1;

    std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>> pixelDirectionsLookupTable
            = std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>(width, std::vector<hololens_msgs::PixelDirection::Ptr>(height, nullptr));
    
    for (uint32_t i = 0; i < pixelDirectionsMsg->pixelDirections.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = pixelDirectionsMsg->pixelDirections[i];
        pixelDirectionsLookupTable[dir.u][dir.v] = hololens_msgs::PixelDirection::Ptr(new hololens_msgs::PixelDirection(dir));
    }

    *pixelDirectionsLookupTableTarget = pixelDirectionsLookupTable;
}
        

// Handles the arrival of a new depth frame.
void FasterDepthDataReceiver::handleDepthFrame(
        const hololens_msgs::DepthFrame::ConstPtr& depthFrame,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        const ros::Publisher& pointCloudWorldSpacePublisher,
        const ros::Publisher& pointCloudFramePublisher,
        uint32_t* sequenceNumber)
{
    // Decode the depth map.
    std::string decoded = base64_decode(depthFrame->base64encodedDepthMap);
    DepthMap depthMap = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight,
            depthFrame->depthMapPixelStride, false);

    // Decode the reflectivity image.
    decoded = base64_decode(depthFrame->base64encodedReflectivity);
    DepthMap reflectivityImage = DepthMap(decoded, depthFrame->depthMapWidth, depthFrame->depthMapHeight,
            depthFrame->reflectivityPixelStride, false);

    // Detect clusters of pixels with similar depth values in the depth map.
    std::vector<std::vector<Pixel>> depthClusters = pixelClusteringUsePcl
            ? detectDepthClustersWithPclClustering(depthMap)
            : detectDepthClustersWithRegionGrowing(depthMap);

    // Reconstruct a point cloud from all pixels which were assigned to a cluster.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudCamSpace = createPointCloudFromClusters(depthClusters, depthMap,
            reflectivityImage, pixelDirectionsLookupTable, minDepth, minReliableDepth, maxReliableDepth, maxDepth);

    // Calculate the transformation from camera space to world space and transform the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromDepthFrame(depthFrame);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);

    // Create a point cloud containing the artificial endpoints of all pixels in the depth map which are too far away
    // from the depth sensor.
    // Please note that this depth data receiver does NOT compute any artificial endpoints (contrary to the initial
    // depth data receiver) as tests made using the initial depth data receiver have shown that generating artificial
    // endpoints is in fact not helpful at all (at least when using the depth sensor of the HoloLens 2). As the depth
    // sensor employed by the HoloLens 2 can't differentiate between points too far away from the sensor and points too
    // close in front in front of the sensor, the generated artificial endpoints would therefore sometimes falsely
    // indicate that there is more free space than there is in reality (which caused for voxels to be falsely removed
    // from the spatial map in the octomap based spatial mapper). It was therefore chosen to leave the calculation of
    // artificial endpoints out in this depth data receiver which in term also saves some time.
    // The initialization of an empty point cloud and the conversion to a ROS message is only here for the purpose of
    // not crashing ROS nodes (i.e. the octomap based spatial mapper at the time of writing this comment) which listen
    // to published point cloud frame messages and try to access their artificial endpoints.
    pcl::PointCloud<pcl::PointXYZI>::Ptr artificialEndpointsWorldSpace (new pcl::PointCloud<pcl::PointXYZI>());

    // Publish the depth map, the HoloLens's current position and the computed point cloud.
    ros::Time time = ros::Time::now();
    sensor_msgs::PointCloud2 pointCloudWorldSpaceMsg;
    sensor_msgs::PointCloud2 artificialEndpointsWorldSpaceMsg;
    pointCloudToMsg(pointCloudWorldSpace, pointCloudWorldSpaceMsg, *sequenceNumber, time, "hololens_world");
    pointCloudToMsg(artificialEndpointsWorldSpace, artificialEndpointsWorldSpaceMsg, *sequenceNumber, time,
            "hololens_world");
    publishHololensPosition(depthFrame, hololensPositionPublisher, positionSequenceNumber, time);
    publishHololensCamToWorldTf(depthFrame, hololensCamPublisher, time);
    publishDepthImage(depthMap, depthClusters, pixelDirectionsLookupTable, imagePublisher, *sequenceNumber, time,
            minDepth, minReliableDepth, maxReliableDepth, maxDepth);
    pointCloudWorldSpacePublisher.publish(pointCloudWorldSpaceMsg);
    publishPointCloudFrame(pointCloudFramePublisher, pointCloudWorldSpaceMsg, artificialEndpointsWorldSpaceMsg,
            depthFrame, maxReliableDepth);
    sequenceNumber++;
    positionSequenceNumber++;
}

std::vector<std::vector<Pixel>> FasterDepthDataReceiver::detectDepthClustersWithRegionGrowing(DepthMap& depthMap)
{
    // Generally speaking, this algorithm is quite similar to the voxel clustering implemented in the Octomap based
    // spatial mapper, which in term implemented "an approach similar to a region growing algorithm" described in the
    // Azim 2012 paper. The only real difference to the clustering algorithm described in that paper is the fact that
    // we're now clustering pixels in 2D image space instead of voxels in 3D world space (hence less potential neighbors
    // to check when adding a pixel to a cluster).

    // Initially, none of the pixels is checked, so we need to create a list (or a lookup table in this implementation)
    // of all pixels that need to be checked (which obviously starts of containing all pixels of the depth map). A
    // custom lookup table was chosen over a list or a hash table as it offers the best lookup performance (i.e. lookup
    // in O(1) without any additional hashing required).
    std::vector<std::vector<bool>> pixelCheckedLookupTable 
            = std::vector<std::vector<bool>>(depthMap.width, std::vector<bool>(depthMap.height, false));
    
    std::vector<std::vector<Pixel>> result;

    // The following loop takes care of the region-growing part. While it's not an exact implementation of what is
    // described in the Azim 2012 paper, it essentially still produces the same results: "[...] all possible dynamic
    // [pixels] are stored in a data list. Our clustering algorithm starts with stepping through this list. [...] If the
    // current [pixel] in the list is not yet assigned to any cluster, a new cluster is initialized. We find the set
    // Neighbor(v) of its neighboring [pixels] from the list. As criterion for adding a [pixel] to the cluster, we use
    // the Euclidean distance between the center of the current [pixel] and the [pixel] in consideration. If this
    // criterion is satisfied by the current [pixel] then it is added to the cluster. Now, we use this newly added
    // [pixel] further and continue the search within its neighborhood in a recursive manner."
    for (uint32_t u = 0; u < depthMap.width; ++u)
    {
        for (uint32_t v = 0; v < depthMap.height; ++v)
        {
            // Ensure that we didn't already check this pixel (potentially assigning it to a cluster).
            if (pixelCheckedLookupTable[u][v])
                continue;
            
            // As long as there are still pixels left to check (i.e. there are pixels which were not assigned to any
            // cluster yet), there is at least one cluster left. Therefore, we initialize a new cluster.
            std::vector<Pixel> clusterPixels;

            // All free pixels in the set of pixels to check are not assigned to any cluster, so we can arbitrarily
            // choose one and use it as the seed pixel of the next cluster.
            Pixel seedPixel = Pixel(u, v);
            
            // Cluster candidate pixels are all pixels in Neighbor(v) which were not checked yet, where v is any pixel
            // which is part of the current cluster.
            // Starting from the cluster's seed pixel, we'll recursively check search the cluster's neighborhood for any
            // pixels which must also be part of the current cluster.
            std::vector<Pixel> clusterCandidatePixels;
            clusterCandidatePixels.push_back(seedPixel);
            while (!clusterCandidatePixels.empty())
            {
                // All of the cluster candidates must be checked eventually, so we can just arbitrarily choose any to
                // check next. In this implementation, I'm using the one which was added most recently to the list of
                // candidates as this is the most efficient way (at least as long as the cluster candidates are stored
                // in a vector).
                Pixel pixel = clusterCandidatePixels.back();
                int32_t pixelDepth = static_cast<int32_t>(depthMap.valueAt(pixel));
                clusterCandidatePixels.pop_back();
                
                // Ensure that we didn't already check this pixels. Otherwise we would run into an endless loop caused
                // by traversing loops in the neighborhood graph of the pixels.
                if (!pixelCheckedLookupTable[pixel.u][pixel.v])
                {
                    // Current pixel was not checked yet. Mark it as checked and add it to the current cluster.
                    pixelCheckedLookupTable[pixel.u][pixel.v] = true;
                    clusterPixels.push_back(pixel);

                    // Add all neighbors of the current pixel to the candidate list. This is the part which would cause
                    // an infinite loop if we didn't check whether the current pixel was already checked.
                    for (size_t i = 0; i < pixelClusteringNeighborhood.size(); i++)
                    {
                        Pixel neighbor = pixel + pixelClusteringNeighborhood[i];
                        if (depthMap.hasValueForPixel(neighbor))
                        {
                            int32_t depthDifference = static_cast<int32_t>(depthMap.valueAt(neighbor)) - pixelDepth;
                            int32_t squaredDepthDifference = depthDifference * depthDifference;
                            if (squaredDepthDifference <= pixelClusteringSquaredDepthValueSimilarity)
                            {
                                // Neighoring pixel has a similar depth value than the current pixel. It needs to be
                                // added to the current cluster if it isn't already part of the cluster.
                                clusterCandidatePixels.push_back(neighbor);
                            }
                        }
                    }
                }
            }

            // Only keep clusters which contain at least a certain amount of pixels.
            if (clusterPixels.size() >= pixelClusteringMinClusterSize)
            {
                result.push_back(clusterPixels);
            }
        }
    }

    return result;
}

bool enforceDepthSimilarity(const pcl::PointXYZI& pointA, const pcl::PointXYZI& pointB, float squaredDistance)
{
    return std::abs(pointA.intensity - pointB.intensity) <= pixelClusteringDepthValueSimilarity;
}

std::vector<std::vector<Pixel>> FasterDepthDataReceiver::detectDepthClustersWithPclClustering(DepthMap& depthMap)
{
    // Convert the depth map to an organized point cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr depthMapCloud (new pcl::PointCloud<pcl::PointXYZI>());
    depthMapCloud->width = depthMap.width;
    depthMapCloud->height = depthMap.height;
    depthMapCloud->is_dense = true;
    depthMapCloud->points.resize(depthMap.width * depthMap.height);

    for (uint32_t u = 0; u < depthMap.width; u++)
    {
        for (uint32_t v = 0; v < depthMap.height; v++)
        {
            pcl::PointXYZI& point = (*depthMapCloud)(u, v);

            point.x = static_cast<float>(u);
            point.y = static_cast<float>(v);
            point.z = 0.0f;
            point.intensity = static_cast<float>(depthMap.valueAt(u, v));
        }
    }
    
    // Create a vector for storing the detected point indices of each cluster.
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters());

    // Extract the clusters of the point cloud.
    pcl::ConditionalEuclideanClustering<pcl::PointXYZI> clustering;
    clustering.setClusterTolerance(pixelClusteringNeighboringPixelDistance);
    clustering.setConditionFunction(&enforceDepthSimilarity);
    clustering.setMinClusterSize(pixelClusteringMinClusterSize);
    clustering.setInputCloud(depthMapCloud);
    clustering.segment(*clusters);

    // Iterate over all found clusters and add all big enough clusters to the resulting pixels.
    std::vector<std::vector<Pixel>> result;
    for (const auto& cluster : (*clusters))
    {
        std::vector<Pixel> clusterPixels;
        clusterPixels.reserve(cluster.indices.size());

        for (const auto& index : cluster.indices)
        {
            // This conversion from floats back to integers seems unnecessary given that these floating point values
            // were previously casted to floats from integer values. There may be a more elegant and more performant
            // way of solving this problem than performing a type cast...
            pcl::PointXYZI& point = (*depthMapCloud)[index];
            uint32_t u = static_cast<uint32_t>(point.x);
            uint32_t v = static_cast<uint32_t>(point.y);
            clusterPixels.push_back(Pixel(u, v));
        }

        result.push_back(clusterPixels);
    }

    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FasterDepthDataReceiver::createPointCloudFromClusters(
        const std::vector<std::vector<Pixel>>& depthClusters,
        DepthMap& depthMap,
        DepthMap& reflectivityImage,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth)
{
    // A scaling factor for scaling reflectivity values into the range [0, 255].
    float activeBrightnessScalingFactor = 1.0 / pow(2.0, static_cast<float>((reflectivityImage.pixelStride - 1) * 8));

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZI>());
    for (auto clusterIt = depthClusters.begin(); clusterIt != depthClusters.end(); ++clusterIt)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZI>());
        for (auto pixelIt = clusterIt->begin(); pixelIt != clusterIt->end(); ++pixelIt)
        {
            // Get the depth at the current pixel.
            uint32_t pixelValue = depthMap.valueAt(*pixelIt);
            float depth = static_cast<float>(pixelValue) / 1000.0f;

            // Skip pixels whose depth values are not within the reliable depth range.
            if (depth < minReliableDepth || depth > maxReliableDepth)
                continue;
            
            // Skip pixels for which we didn't receive any pixel direction. This should in theory only ever happen when
            // this depth data receiver is used with the HoloLens 1, which only captures depth data inside a circular
            // area of the depth image. The HoloLens 2 captures depth data for the whole depth image, so we should also
            // always have a corresponding direction for each pixel.
            if (pixelIt->u >= pixelDirectionsLookupTable.size() || pixelIt->v >= pixelDirectionsLookupTable[0].size())
                continue;
            hololens_msgs::PixelDirection::Ptr dir = pixelDirectionsLookupTable[pixelIt->u][pixelIt->v];
            if (dir == nullptr)
                continue;

            // Get the active brightness of the current pixel, scale it to fit into the range [0, 255] and try to
            // compensate for intensity falloff for longer distances caused by the inverse-square law in order to
            // roughly estimate the reflectivity at the current pixel.
            uint32_t activeBrightnessValue = reflectivityImage.valueAt(*pixelIt);
            float activeBrightnessScaled = static_cast<float>(activeBrightnessValue) * activeBrightnessScalingFactor;
            float reflectivity = activeBrightnessScaled * (depth * depth);

            // Calculate the point for the current pixel based on the pixel's depth value.
            pcl::PointXYZI point;
            point.x = dir->direction.x * depth;
            point.y = dir->direction.y * depth;
            point.z = dir->direction.z * depth;
            point.intensity = reflectivity;
            clusterCloud->push_back(point);
        }

        // Downsampling each cluster individually (instead of the whole point cloud containing all clusters) avoids
        // comparing points which are already known to be too far apart. This should in theory be a bit faster than
        // downsampling the complete point cloud at once.
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloudDownsampled 
                = downsamplePointCloud(clusterCloud, downsamplingLeafSize);

        // Combine all clusters into a single point cloud.
        *pointCloudCamSpace += *clusterCloudDownsampled;
    }

    return pointCloudCamSpace;
}

void FasterDepthDataReceiver::publishDepthImage(
        const DepthMap depthMap,
        const std::vector<std::vector<Pixel>>& depthClusters,
        const std::vector<std::vector<hololens_msgs::PixelDirection::Ptr>>& pixelDirectionsLookupTable,
        const ros::Publisher& publisher, 
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth)
{
    // Create the ROS message for the image.
    sensor_msgs::Image image;

    // Set the header of the image.
    image.header.seq = sequenceNumber;
    image.header.stamp = timestamp;
    image.header.frame_id = "hololens_cam";

    // Set the meta data (width, height, encoding, ...) of the image.
    image.height = depthMap.height;
    image.width = depthMap.width;
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = 0;
    image.step = depthMap.width * 3;

    // Initialize the image to be completely black.
    image.data.resize(image.height * image.width * 3, 0);

    // Iterate over all clusters and assign all pixels belonging to the same cluster the same color.
    for (int i = 0; i < depthClusters.size(); i++)
    {
        const std::vector<Pixel>& cluster = depthClusters[i];

        int colorIndex = i % pixelClusterColors.size();
        std::vector<float>& clusterColor = pixelClusterColors[colorIndex];

        // Iterate over all pixels of the current cluster and assign them the same color.
        for (auto pixelIt = cluster.begin(); pixelIt != cluster.end(); pixelIt++)
        {
            uint32_t index = (pixelIt->v * image.width + pixelIt->u) * 3;

            // Get the depth of the current pixel.
            uint32_t pixelValue = depthMap.valueAt(*pixelIt);
            float depth = static_cast<float>(pixelValue) / 1000.0f;

            // Calculate the greyscale value which will be used for the pixel.
            float greyscaleValue = 255.0f * depth / maxDepth;

            image.data[index] = static_cast<uint8_t>(greyscaleValue * clusterColor[0]);
            image.data[index + 1] = static_cast<uint8_t>(greyscaleValue * clusterColor[1]);
            image.data[index + 2] = static_cast<uint8_t>(greyscaleValue * clusterColor[2]);
        }
    }

    // Publish the message.
    publisher.publish(image);
}