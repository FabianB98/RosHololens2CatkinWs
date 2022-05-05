#include <cmath>
#include <stdlib.h>
#include <string>

#include "Topics.h"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_broadcaster.h>

class SpatialMapper
{
public:
    // Constructors.
    SpatialMapper(ros::NodeHandle n, const std::string spatialMapTopic = SPATIAL_MAP_TOPIC);

    // Callbacks for handling the incoming point cloud frames.
    void handlePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

    // Callbacks for post processing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr doPostProcessing();
    void smoothenSpatialMap();
    void detectPlanes();

    // Callbacks for clearing and saving the point cloud.
    void clearSpatialMap();
    void saveSpatialMap();

    // Methods for accessing the spatial map.
    pcl::PointCloud<pcl::PointXYZ>::Ptr getSpatialMap(bool performPostProcessing);

private:
    // Downsamples a given point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
        const float leafSize);

    // Removes outliers from a given point cloud with a radius outlier removal filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersRadius(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float radiusSearch,
        const int minNeighborsInRadius);

    // Registers a given point cloud to the global point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPrealignedWorldSpace);

    // Smoothes the given cloud using moving least squares.
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothenPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Estimates the normals for the given point cloud.
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToEstimateNormalsFor,
        const double searchRadius);

    // Detects the point indices of all clusters in the given point cloud.
    std::vector<pcl::PointIndices> detectClusterIndices(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
        const double clusterTolerance);

    // Detects all clusters in the given point cloud.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detectClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
        const double clusterTolerance);

    // Detects all clusters in the given point cloud with normals.
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> > detectClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToCluster,
        const pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
        const double clusterTolerance);

    // Detects planes in the given cloud and projects all found inliers onto the corresponding plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Detects planes in the given input cloud and clusters the cloud into planes and remainder.
    int detectPlanes(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
        const pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
        pcl::PointCloud<pcl::PointXYZ>::Ptr planes, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr remainder,
        const std::size_t totalCloudSize,
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg);

    // Methods for publishing the results.
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Switches indicating which filter algorithms should be executed.
    bool downsampleSpatialMap;

    // Switches regarding the registration of new point clouds to the spatial map.
    bool useICP;

    // Hyper parameters used for downsampling.
    float downsamplingLeafSize;

    // Hyper parameters used for registration.
    int icpMaxIterations;
    double icpTransformationEpsilon;
    double icpMaxCorrespondenceDistance;
    double icpRansacOutlierRejectionThreshold;
    double icpEuclideanFitnessEpsilon;

    // Hyper parameters used for smoothing the spatial map.
    int mlsPolynomialOrder;
    double mlsSearchRadius;
    double mlsOutlierRadiusSearch;
    int mlsOutlierMinNeighborsInRadius;

    // Hyper parameters used for normal estimation.
    double normalEstimationSearchRadius;

    // Hyper parameters used for plane detection.
    double planeDetectionEpsAngle;
    double planeDetectionNormalDistanceWeight;
    double planeDetectionDistanceThreshold;
    int planeDetectionMaxRansacIterations;
    int planeDetectionMinInliersAbsolute;
    double planeDetectionMinInliersRelative;
    int planeDetectionKeepClusterAbsolute;
    double planeDetectionKeepClusterRelative;

    // ROS publishers.
    ros::Publisher spatialMapPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t spatialMapSequenceNumber;

    // The spatial map.
    pcl::PointCloud<pcl::PointXYZ>::Ptr spatialMap;

    // A mutex used for mutual exclusion when accessing the spatial map.
    boost::mutex spatialMapMutex;
};