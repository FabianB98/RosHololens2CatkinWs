#include "SpatialMapper.h"

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

#include <pcl_conversions/pcl_conversions.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                         SWITCHES FOR CHANGING THE DEFAULT BEHAVIOUR OF THE SPATIAL MAPPER                          //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Switches indicating which filter algorithms should be executed.
// Should the global point cloud be downsampled after the new point cloud was registered to it?
#define DOWNSAMPLE_SPATIAL_MAP true

// Switches regarding the registration of new point clouds to the global point cloud.
// Should ICP be used for registering the new point cloud to the global point cloud?
#define USE_ICP false

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                  DEFAULT HYPER PARAMETERS FOR THE USED ALGORITHMS                                  //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// Parameters for the voxel grid filter used for downsampling.
#define DOWNSAMPLING_LEAF_SIZE 0.01f    // At most one point allowed in a voxel within this edge length.

// Parameters for ICP used for the registration of new point clouds to the global cloud.
#define ICP_MAX_ITERATIONS 20                           // The maximum amount of iterations to perform.
#define ICP_TRANSFORMATION_EPSILON 1e-12                // Changes less than this value result in convergence.
#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.1             // The maximum distance to use for checking for correspondences.
#define ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD 0.001    // Points closer than this value can't be outliers.
#define ICP_EUCLIDEAN_FITNESS_EPSILON 1.0               // A fitness worse than this value results in an abortion.

// Parameters for smoothing the point cloud.
#define MLS_POLYNOMIAL_ORDER 2  // The order of the polynomial function used to approximate the surface.
#define MLS_SEARCH_RADIUS 0.04  // The search radius to use for determining the polynomial function.

// Paramters for the radius outlier removal filter used after MLS has been applied.
#define MLS_OUTLIER_RADIUS_SEARCH 0.03          // The radius in which to search for neighbors.
#define MLS_OUTLIER_MIN_NEIGHBORS_IN_RADIUS 6   // The minimum amount of neighbors which have to be in the radius.

// Parameters for estimating normals.
#define NORMAL_ESTIMATION_SEARCH_RADIUS 0.07    // The radius in which to search for nearest neighbors.

// Parameters for detecting planes.
#define PLANE_DETECTION_EPS_ANGLE 1.0               // The maximum angle deviation from horizontal or vertical planes.
#define PLANE_DETECTION_NORMAL_DISTANCE_WEIGHT 0.06 // Lower -> Points normal might deviate more from the planes normal.
#define PLANE_DETECTION_DISTANCE_THRESHOLD 0.04     // Points closer than this values are considered as inliers.
#define PLANE_DETECTION_MAX_RANSAC_ITERATIONS 1000  // The maximum amount of RANSAC iterations to perform per plane.
#define PLANE_DETECTION_MIN_INLIERS_ABSOLUTE 10000  // A plane must have at least this many absolute inliers and...
#define PLANE_DETECTION_MIN_INLIERS_RELATIVE 0.02   // ...this many inliers relative to the size of the global cloud.
#define PLANE_DETECTION_KEEP_CLUSTER_ABSOLUTE 10000 // A cluster of a plane must have at least this many points or...
#define PLANE_DETECTION_KEEP_CLUSTER_RELATIVE 0.25  // ...this many points relative to the planes inliers.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                                                                                    //
//                                              ACTUAL CODE STARTS HERE                                               //
//                                                                                                                    //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

SpatialMapper::SpatialMapper(ros::NodeHandle n, const std::string spatialMapTopic)
{
    ROS_INFO("Creating SpatialMapper...");

    // Initialize all switches as specified by parameters (or their default value).
    n.param("downsampleSpatialMap", downsampleSpatialMap, DOWNSAMPLE_SPATIAL_MAP);
    n.param("useICP", useICP, USE_ICP);

    // Initialize all hyper parameters as specified by parameters (or their default value).
    n.param("downsamplingLeafSize", downsamplingLeafSize, DOWNSAMPLING_LEAF_SIZE);
    n.param("icpMaxIterations", icpMaxIterations, ICP_MAX_ITERATIONS);
    n.param("icpTransformationEpsilon", icpTransformationEpsilon, ICP_TRANSFORMATION_EPSILON);
    n.param("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance, ICP_MAX_CORRESPONDENCE_DISTANCE);
    n.param("icpRansacOutlierRejectionThreshold", icpRansacOutlierRejectionThreshold, ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD);
    n.param("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon, ICP_EUCLIDEAN_FITNESS_EPSILON);
    n.param("normalEstimationSearchRadius", normalEstimationSearchRadius, NORMAL_ESTIMATION_SEARCH_RADIUS);
    n.param("mlsPolynomialOrder", mlsPolynomialOrder, MLS_POLYNOMIAL_ORDER);
    n.param("mlsSearchRadius", mlsSearchRadius, MLS_SEARCH_RADIUS);
    n.param("mlsOutlierRadiusSearch", mlsOutlierRadiusSearch, MLS_OUTLIER_RADIUS_SEARCH);
    n.param("mlsOutlierMinNeighborsInRadius", mlsOutlierMinNeighborsInRadius, MLS_OUTLIER_MIN_NEIGHBORS_IN_RADIUS);
    n.param("planeDetectionEpsAngle", planeDetectionEpsAngle, PLANE_DETECTION_EPS_ANGLE);
    n.param("planeDetectionNormalDistanceWeight", planeDetectionNormalDistanceWeight, PLANE_DETECTION_NORMAL_DISTANCE_WEIGHT);
    n.param("planeDetectionDistanceThreshold", planeDetectionDistanceThreshold, PLANE_DETECTION_DISTANCE_THRESHOLD);
    n.param("planeDetectionMaxRansacIterations", planeDetectionMaxRansacIterations, PLANE_DETECTION_MAX_RANSAC_ITERATIONS);
    n.param("planeDetectionMinInliersAbsolute", planeDetectionMinInliersAbsolute, PLANE_DETECTION_MIN_INLIERS_ABSOLUTE);
    n.param("planeDetectionMinInliersRelative", planeDetectionMinInliersRelative, PLANE_DETECTION_MIN_INLIERS_RELATIVE);
    n.param("planeDetectionKeepClusterAbsolute", planeDetectionKeepClusterAbsolute, PLANE_DETECTION_KEEP_CLUSTER_ABSOLUTE);
    n.param("planeDetectionKeepClusterRelative", planeDetectionKeepClusterRelative, PLANE_DETECTION_KEEP_CLUSTER_RELATIVE);

    // Initialize the spatial map.
    spatialMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Advertise the topics to which the spatial map will be published.
    spatialMapPublisher = n.advertise<sensor_msgs::PointCloud2>(spatialMapTopic, 10);
}

void SpatialMapper::handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace
            = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pointCloudWorldSpace);

    // Register the new point cloud to the spatial map.
    pcl::PointCloud<pcl::PointXYZ>::Ptr combinedPointCloud = registerPointCloud(pointCloudWorldSpace);

    // TODO: Remove the following line of code later on...
    ROS_INFO("Spatial map consists of %zu points.", combinedPointCloud->size());

    // Publish the point cloud.
    publishPointCloud(combinedPointCloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
    const float leafSize)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDownsampled (new pcl::PointCloud<pcl::PointXYZ>());

    // Downsample the given point cloud using a voxel grid filter.
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(pointCloudToDownsample);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*pointCloudDownsampled);

    // Return the downsampled point cloud.
    return pointCloudDownsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::removeOutliersRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
    const float radiusSearch,
    const int minNeighborsInRadius)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>());

    // Remove outliers by using a radius outlier removal.
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval;
    radiusOutlierRemoval.setInputCloud(pointCloudToFilter);
    radiusOutlierRemoval.setRadiusSearch(radiusSearch);
    radiusOutlierRemoval.setMinNeighborsInRadius(minNeighborsInRadius);
    radiusOutlierRemoval.filter(*pointCloudFiltered);

    // Return the point cloud with the outliers removed.
    return pointCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::registerPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPrealignedWorldSpace)
{
    // Lock the spatial map mutex as we're about to start the registration of the new point cloud.
    spatialMapMutex.lock();

    // Transform the point cloud from camera space to world space.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZ>());
    if (spatialMap->size() != 0 && useICP) 
    {
        // There exists some part of the spatial map, so we need to align the given point cloud. Use ICP to
        // align the given point cloud to the spatial map.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Set the parameters for ICP.
        icp.setMaximumIterations(1);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
        icp.setRANSACOutlierRejectionThreshold(icpRansacOutlierRejectionThreshold);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        // Set source (i.e. the given point cloud) and target (i.e. the spatial map) point clouds for ICP.
        icp.setInputSource(pointCloudPrealignedWorldSpace);
        icp.setInputTarget(spatialMap);

        // Align the new point cloud using the camera to world transformation as an initial guess.
        icp.align(*pointCloudWorldSpace);

        unsigned int iteration = 1;
        while (!icp.hasConverged() && iteration < icpMaxIterations)
        {
            icp.align(*pointCloudWorldSpace, icp.getFinalTransformation());
            iteration++;
        }

        // Print information about the results of ICP.
        ROS_INFO("ICP converged? %s", icp.hasConverged() ? "yes" : "no");
        ROS_INFO("Num ICP iterations: %u", iteration);
        ROS_INFO("ICP Fitness score: %f", icp.getFitnessScore());
    }
    else
    {
        // There is no spatial map yet or we shouldn't use ICP. We therefore don't need to align the given point cloud.
        // Simply assume that the given point cloud is already correctly aligned in world space.
        pointCloudWorldSpace = pointCloudPrealignedWorldSpace;
    }

    // Concatenate the point clouds (i.e. add the new point cloud calculated from the new depth frame to the point cloud
    // calculated from the previous frames).
    *spatialMap += *pointCloudWorldSpace;

    // Downsample the concatenated point cloud to avoid a point density which is higher than what is needed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = spatialMap;
    if (downsampleSpatialMap) 
    {
        cloud = downsamplePointCloud(spatialMap, downsamplingLeafSize);
        spatialMap = cloud;
    }

    // Unlock the spatial map mutex as we're done with the registration of the new point cloud.
    spatialMapMutex.unlock();

    // Return the downsampled concatenated point cloud.
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::doPostProcessing()
{
    // Lock the point cloud mutex as we're about to perform the post processing steps.
    spatialMapMutex.lock();

    // Perform the post processing steps (MLS, outlier removal, RANSAC).
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud = smoothenPointCloud(spatialMap);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud = detectPlanes(smoothedCloud);
    spatialMap = projectedCloud;

    // We can unlock the point cloud mutex again as the post processing is done.
    spatialMapMutex.unlock();

    // Return the postprocessed spatial map.
    return projectedCloud;
}

void SpatialMapper::smoothenSpatialMap()
{
    spatialMapMutex.lock();
    spatialMap = smoothenPointCloud(spatialMap);
    spatialMapMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::smoothenPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    ROS_INFO("Smoothing point cloud... Depending on the cloud's size, this may take a few minutes.");

    // Initialize a KD tree and a MLS instance.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZ> mls (boost::thread::hardware_concurrency());

    unsigned int numThreads = boost::thread::hardware_concurrency();

    // Set up all parameters of MLS.
    mls.setNumberOfThreads(numThreads);
    mls.setComputeNormals(false);
    mls.setPolynomialOrder(mlsPolynomialOrder);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mlsSearchRadius);

    // Smoothen the point cloud using MLS.
    pcl::PointCloud<pcl::PointXYZ>::Ptr mlsPoints (new pcl::PointCloud<pcl::PointXYZ>());
    mls.setInputCloud(cloud);
    mls.process(*mlsPoints);

    // Remove outliers from the smoothed point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsFiltered = 
            removeOutliersRadius(mlsPoints, mlsOutlierRadiusSearch, mlsOutlierMinNeighborsInRadius);

    // Publish the point cloud.
    publishPointCloud(pointsFiltered);

    ROS_INFO("Smoothed point cloud!");

    return pointsFiltered;
}

pcl::PointCloud<pcl::Normal>::Ptr SpatialMapper::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToEstimateNormalsFor,
    const double searchRadius)
{
    // Create a normal estimation instance as well as a KD tree.
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());

    // Set the parameters for the normal estimation.
    ne.setInputCloud(cloudToEstimateNormalsFor);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(searchRadius);

    // Estimate the normals and return the result.
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloudNormals);
    return cloudNormals;
}

std::vector<pcl::PointIndices> SpatialMapper::detectClusterIndices(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
        const double clusterTolerance)
{
    // Set up a KD tree for searching inside the cloud.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloudToCluster);
    
    // Create a vector for storing the detected point indices of each cluster.
    std::vector<pcl::PointIndices> clusters;

    // Extract the clusters of the point cloud.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance);
    clusterExtraction.setSearchMethod(tree);
    clusterExtraction.setInputCloud(cloudToCluster);
    clusterExtraction.extract(clusters);

    // Return the resulting cluster indices.
    return clusters;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> SpatialMapper::detectClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
        const double clusterTolerance)
{
    // Detect the point indices of all clusters.
    std::vector<pcl::PointIndices> clusters = detectClusterIndices(cloudToCluster, clusterTolerance);

    // Create an indices extractor for removing each cluster from the cloud.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudToCluster);

    // Iterate over all found clusters and create a point cloud for each cluster.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds;
    for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
    {
        // Create a pointer to the point indices of the current cluster. The copying is needed because boost will free
        // the pointer after the point cloud was created, which would in term free some part of the clusters vector
        // causing weird side effects (i.e. the application might crash) in later iterations of this loop.
        pcl::PointIndices::Ptr indices (new pcl::PointIndices());
        std::copy(iter->indices.begin(), iter->indices.end(), std::back_inserter(indices->indices));

        // Create a point cloud representing the current cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>());
        extract.setIndices(indices);
        extract.filter(*cluster);

        // Add the point cloud to the vector of clusters.
        clusterClouds.push_back(cluster);
    }

    // Return the point clouds representing the clusters.
    return clusterClouds;
}

std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> >
        SpatialMapper::detectClusters(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
                const pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
                const double clusterTolerance)
{
    // Detect the point indices of all clusters.
    std::vector<pcl::PointIndices> clusters = detectClusterIndices(cloudToCluster, clusterTolerance);

    // Create an indices extractor for removing the points of each cluster from the point cloud.
    pcl::ExtractIndices<pcl::PointXYZ> extractPoints;
    extractPoints.setInputCloud(cloudToCluster);

    // Create an indices extractor for removing the normals of each cluster from the normals cloud.
    pcl::ExtractIndices<pcl::Normal> extractNormals;
    extractNormals.setInputCloud(cloudNormals);

    // Iterate over all found clusters and create a point cloud for each cluster.
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> > clusterClouds;
    for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
    {
        // Create a pointer to the point indices of the current cluster. The copying is needed because boost will free
        // the pointer after the point cloud was created, which would in term free some part of the clusters vector
        // causing weird side effects (i.e. the application might crash) in later iterations of this loop.
        pcl::PointIndices::Ptr indices (new pcl::PointIndices());
        std::copy(iter->indices.begin(), iter->indices.end(), std::back_inserter(indices->indices));

        // Create a point cloud representing the points of the current cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPoints (new pcl::PointCloud<pcl::PointXYZ>());
        extractPoints.setIndices(indices);
        extractPoints.filter(*clusterPoints);

        // Create a point cloud representing the normals of the current cluster.
        pcl::PointCloud<pcl::Normal>::Ptr clusterNormals (new pcl::PointCloud<pcl::Normal>());
        extractNormals.setIndices(indices);
        extractNormals.filter(*clusterNormals);

        // Add the point cloud to the vector of clusters.
        clusterClouds.push_back(std::make_pair(clusterPoints, clusterNormals));
    }

    // Return the point clouds representing the clusters.
    return clusterClouds;
}

void SpatialMapper::detectPlanes()
{
    spatialMapMutex.lock();
    spatialMap = detectPlanes(spatialMap);
    spatialMapMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    ROS_INFO("Detecting planes in the point cloud... Depending on the clouds size, this may take a few minutes.");

    // Create point clouds for the detected planes and the remainder.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planes (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr remainder (new pcl::PointCloud<pcl::PointXYZ>());

    // Set up the parameters for RANSAC.
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setEpsAngle(pcl::deg2rad(planeDetectionEpsAngle));
    seg.setNormalDistanceWeight(planeDetectionNormalDistanceWeight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMaxIterations(planeDetectionMaxRansacIterations);
    seg.setDistanceThreshold(planeDetectionDistanceThreshold);
    seg.setOptimizeCoefficients(true);

    // Recursively detect and project planes using the global point cloud as the initial cluster.
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals = estimateNormals(cloud, normalEstimationSearchRadius);
    int numPlanes = detectPlanes(cloud, cloudNormals, planes, remainder, cloud->points.size(), seg);

    // Recreate the global point cloud from the projected planes and the remainder.
    pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>());
    *result += *planes;
    *result += *remainder;
    publishPointCloud(result);

    ROS_INFO("Detected %i planes!", numPlanes);

    return result;
}

int SpatialMapper::detectPlanes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    const pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr planes, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr remainder,
    const std::size_t totalCloudSize,
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg)
{
    // In case the input cloud contains too less points, abort.
    if (inputCloud->size() < planeDetectionMinInliersAbsolute)
    {
        *remainder += *inputCloud;
        return 0;
    }

    // Set the input cloud for determining plane candidates.
    seg.setInputCloud(inputCloud);
    seg.setInputNormals(cloudNormals);

    // Try to find a plane candidate.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);

    // Determine whether the plane candidate contains too less points. If that's the case, abort.
    std::size_t numInliers = inliers->indices.size();
    if (numInliers <= planeDetectionMinInliersRelative * totalCloudSize
            || numInliers <= planeDetectionMinInliersAbsolute)
    {
        *remainder += *inputCloud;
        return 0;
    }
    
    // Set up point extraction.
    pcl::ExtractIndices<pcl::PointXYZ> extractPoints;
    extractPoints.setInputCloud(inputCloud);
    extractPoints.setIndices(inliers);
    pcl::ExtractIndices<pcl::Normal> extractNormals;
    extractNormals.setInputCloud(cloudNormals);
    extractNormals.setIndices(inliers);

    // Extract the detected plane from the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planePoints (new pcl::PointCloud<pcl::PointXYZ>());
    extractPoints.setNegative(false);
    extractPoints.filter(*planePoints);
    pcl::PointCloud<pcl::Normal>::Ptr planeNormals (new pcl::PointCloud<pcl::Normal>());
    extractNormals.setNegative(false);
    extractNormals.filter(*planeNormals);

    // Extract the remainder (everything except the detected plane) from the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr remPoints (new pcl::PointCloud<pcl::PointXYZ>());
    extractPoints.setNegative(true);
    extractPoints.filter(*remPoints);
    pcl::PointCloud<pcl::Normal>::Ptr remNormals (new pcl::PointCloud<pcl::Normal>());
    extractNormals.setNegative(true);
    extractNormals.filter(*remNormals);

    // Show debug information consisting of the detected plane.
    ROS_INFO("Detected a plane consisting of %zu points!", planePoints->points.size());

    // Extract the clusters of the plane. This is done in order to avoid having points inside the plane which do not
    // belong to the object which should be represented by the plane.
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> > planeClusters =
            detectClusters(planePoints, planeNormals, planeDetectionDistanceThreshold);
    
    // Print debug information about the detected clusters.
    ROS_INFO("Detected %zu clusters in the detected plane!", planeClusters.size());

    // Iterate over all found clusters and add clusters which are big enough to the clustered plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeClustered (new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> >::const_iterator it;
    for (it = planeClusters.begin(); it != planeClusters.end(); ++it)
    {
        // Get the points and the normals of the current cluster.
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> cluster = *it;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPoints = cluster.first;
        pcl::PointCloud<pcl::Normal>::Ptr clusterNormals = cluster.second;

        // Determine whether the current cluster is part of the plane or the remainder.
        std::size_t absolutePoints = clusterPoints->points.size();
        double relativePoints = static_cast<double>(absolutePoints) / static_cast<double>(planePoints->points.size());
        bool keepCluster = absolutePoints >= planeDetectionKeepClusterAbsolute 
                || relativePoints >= planeDetectionKeepClusterRelative;

        // If the cluster is big enough, add it to the clustered plane, otherwise add it to the remainder.
        if (keepCluster)
        {
            *planeClustered += *clusterPoints;
            // We don't need the normals of the plane. We can therefore ignore the normals in this case.

            // Print debug information indicating what we're doing with the current cluster.
            ROS_INFO("Cluster contains of %zu points (%lf%% of the plane)! Adding it to the clustered plane.",
                    absolutePoints, relativePoints * 100.0);
        }
        else
        {
            *remPoints += *clusterPoints;
            *remNormals += *clusterNormals;

            // Print debug information indicating what we're doing with the current cluster.
            ROS_INFO("Cluster contains of %zu points (%lf%% of the plane)! Adding it to the remainder cloud.",
                    absolutePoints, relativePoints * 100.0);
        }
    }

    // Project the inliers onto the detected plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeProjected (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ProjectInliers<pcl::PointXYZ> projectInliers;
    projectInliers.setModelType(pcl::SACMODEL_PLANE);
    projectInliers.setModelCoefficients(coefficients);
    projectInliers.setInputCloud(planeClustered);
    projectInliers.filter(*planeProjected);

    // We have found a plane. Add it to the cloud of planes.
    int numPlanes = 1;
    *planes += *planeProjected;

    // Extract the clusters of the of the remainder and recursively detect planes in each cluster.
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> > remainderClusters =
            detectClusters(remPoints, remNormals, 1.5 * planeDetectionDistanceThreshold);
    for (it = remainderClusters.begin(); it != remainderClusters.end(); ++it)
    {
        // Get the points and the normals of the current cluster.
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> cluster = *it;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPoints = cluster.first;
        pcl::PointCloud<pcl::Normal>::Ptr clusterNormals = cluster.second;

        // Recursively detect planes in the current cluster.
        numPlanes += detectPlanes(clusterPoints, clusterNormals, planes, remainder, totalCloudSize, seg);
    }

    // Return the amount of detected planes.
    return numPlanes;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SpatialMapper::getSpatialMap(bool performPostProcessing) 
{
    if (performPostProcessing)
    {
        return doPostProcessing();
    }
    else
    {
        return spatialMap;
    }
}

void SpatialMapper::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*cloud, pointCloudMessage);

    // Set the header of the message.
    pointCloudMessage.header.seq = spatialMapSequenceNumber++;
    pointCloudMessage.header.stamp = ros::Time::now();
    pointCloudMessage.header.frame_id = "hololens_world";

    // Publish the message.
    spatialMapPublisher.publish(pointCloudMessage);
}

void SpatialMapper::clearSpatialMap()
{
    ROS_INFO("Clearing spatial map...");

    // Remove all points from the spatial map and publish the cleared spatial map.
    spatialMapMutex.lock();
    spatialMap->clear();
    publishPointCloud(spatialMap);
    spatialMapMutex.unlock();
}

void SpatialMapper::saveSpatialMap()
{
    ROS_INFO("Saving spatial map...");

    // Create the prefix (file path and file name) of the file.
    std::string home = std::string(getenv("HOME"));
    std::string directory = home + "/spatial_maps/";
    std::string time = boost::lexical_cast<std::string>(ros::Time::now().toNSec());
    std::string prefix = directory + time;

    // Create the directory (if not already done) in which the point cloud will be saved.
    boost::filesystem::create_directories(directory);

    // Save the point cloud in both PCD and PLY file format.
    spatialMapMutex.lock();
    bool notEmpty = spatialMap->size() > 0;
    if (notEmpty)
    {
        pcl::io::savePCDFileASCII(prefix + ".pcd", *spatialMap);
        pcl::io::savePLYFileASCII(prefix + ".ply", *spatialMap);
    }
    spatialMapMutex.unlock();

    if (notEmpty)
        ROS_INFO("Saved spatial map!");
    else
        ROS_INFO("Couldn't save spatial map as it doesn't contain any points!");
}
