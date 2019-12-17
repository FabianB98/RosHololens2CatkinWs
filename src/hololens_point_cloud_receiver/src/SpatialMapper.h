#include <cmath>
#include <stdlib.h>
#include <string>

#include "Base64.h"
#include "DepthMap.h"
#include "Topics.h"

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/PixelDirection.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_broadcaster.h>

class SpatialMapper
{
public:
    // Constructors.
    SpatialMapper(
        ros::NodeHandle n, 
        const std::string shortThrowImageTopic = SHORT_THROW_IMAGE_TOPIC, 
        const std::string longThrowImageTopic = LONG_THROW_IMAGE_TOPIC,
        const std::string pointCloudTopic = POINT_CLOUD_TOPIC,
        const std::string hololensPositionTopic = HOLOLENS_POSITION_TOPIC);

    // Callbacks for handling the incoming depth frames and pixel directions.
    void handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

    // Callbacks for post processing the point cloud.
    void smoothenPointCloud();
    void detectPlanes();

    // Callbacks for clearing and saving the point cloud.
    void clearPointCloud();
    void savePointCloud();

private:
    // Handles the arrival of a new depth frame.
    void handleDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame, 
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth,
        const ros::Publisher& imagePublisher,
        uint32_t* sequenceNumber);

    // Computes a point cloud (in camera space) from a given depth map.
    pcl::PointCloud<pcl::PointXYZ>::Ptr computePointCloudFromDepthMap(
        const DepthMap depthMap, 
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minReliableDepth,
        const float maxReliableDepth);

    // Computes the transformation matrix for transforming from camera space to world space.
    Eigen::Matrix4f computeCamToWorldFromDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);

    // Downsamples a given point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToDownsample,
        const float leafSize);
    
    // Removes outliers from a given point cloud with a radius outlier removal filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersRadius(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float radiusSearch,
        const int minNeighborsInRadius);

    // Removes outliers from a given point cloud with a statistical outlier removal filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersStatistical(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToFilter,
        const float numNeighborsToCheck,
        const float standardDeviationMultiplier);

    // Registers a given point cloud to the global point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamSpace,
        Eigen::Matrix4f camToWorld);

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

    // Detects planes in the given input cloud and clusters the cloud into planes and remainder.
    int detectPlanes(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
        const pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
        pcl::PointCloud<pcl::PointXYZ>::Ptr planes, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr remainder,
        const std::size_t totalCloudSize,
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg);

    // Methods for publishing the results.
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher* publisher = NULL);
    void publishHololensPosition(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);
    void publishHololensCamToWorldTf(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame);
    void publishDepthImage(
        const DepthMap depthMap, 
        const ros::Publisher& publisher, 
        uint32_t* sequenceNumber,
        const float minDepth,
        const float minReliableDepth,
        const float maxReliableDepth,
        const float maxDepth);

// Note: From a software engineering perspective, it would be better to have getters and setters for all of the 
// following public variables. However, I didn't find the time to do this yet.
// TODO: Change all the following variables to be private and add getters and setters for them.
public:
    // Switches for whether short throw and/or long throw depth frames should be used for calculating the point cloud.
    bool useShortThrow;
    bool useLongThrow;

    // Switches indicating which filter algorithms should be executed.
    bool downsampleNewCloud;
    bool removeOutliersRadiusNewCloud;
    bool removeOutliersStatisticalNewCloud;
    bool downsampleGlobalCloud;

    // Switches regarding the registration of new point clouds to the global point cloud.
    bool useICP;

    // Switches indicating which results should be published.
    bool publishCurrentPosition;
    bool publishCurrentDepthImage;
    bool publishGlobalPointCloud;

    // Switches regarding debugging information.
    bool printCentroid;
    bool printICPResults;
    bool printPlaneDetectionResults;

    // Hyper parameters used for downsampling.
    float downsamplingLeafSize;

    // Hyper parameters used for outlier removal.
    double outlierRemovalRadiusSearch;
    int outlierRemovalMinNeighborsInRadius;
    int outlierRemovalNeighborsToCheck;
    double outlierRemovalStdDeviationMultiplier;

    // Hyper parameters used for registration.
    int icpMaxIterations;
    double icpTransformationEpsilon;
    double icpMaxCorrespondenceDistance;
    double icpRansacOutlierRejectionThreshold;
    double icpEuclideanFitnessEpsilon;

    // Hyper parameters used for smoothing the point cloud.
    int mlsPolynomialOrder;
    double mlsSearchRadius;

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

    // Sensor intrinsics of the short throw depth sensor.
    float shortThrowMinDepth;
    float shortThrowMinReliableDepth;
    float shortThrowMaxReliableDepth;
    float shortThrowMaxDepth;

    // Sensor intrinsics of the long throw depth sensor.
    float longThrowMinDepth;
    float longThrowMinReliableDepth;
    float longThrowMaxReliableDepth;
    float longThrowMaxDepth;

private:
    // The directions (in camera space) in which each pixel of the depth frames points at.
    hololens_point_cloud_msgs::PixelDirections::ConstPtr shortThrowDirections;
    hololens_point_cloud_msgs::PixelDirections::ConstPtr longThrowDirections;

    // ROS publishers.
    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher pointCloudPublisher;
    ros::Publisher hololensPositionPublisher;
    tf::TransformBroadcaster hololensCamPublisher;
    std::vector<ros::Publisher> additionalPublishers;

    // Sequence numbers used for publishing the results.
    uint32_t shortThrowSequenceNumber;
    uint32_t longThrowSequenceNumber;
    uint32_t pointCloudSequenceNumber;

    // The global point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;

    // A mutex used for mutual exclusion when accessing the global point cloud.
    boost::mutex pointCloudMutex;
};