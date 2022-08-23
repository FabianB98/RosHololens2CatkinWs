// This file was not part of the repository published at https://github.com/yzrobot/online_learning. It was added as
// part of my work to be able to use multiple SVMs (such as the human classifier) to classify between multiple classes
// of objects and to be able to use these classifiers from other ROS nodes by the use of a service.
// Please note that my original intent was to also get rid of the code duplications found between object_3d_detector.cpp
// and object_3d_detector_ol.cpp, but upon further inspection I had to find that there are sometimes subtle differences
// between methods of the same name found in these two files. As I don't have the time to perform a complete refactoring
// of these two files, I decided to copy the code from object3d_detector.cpp and use that as a starting point for my
// modifications to make the classifiers run as a service. It should however be noted that this will definitely 
// result in even more code duplication...


// Standard libraries
#include <unordered_map>
#include <vector>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// Parameter parsing
#include <XmlRpcValue.h>
// Boost
#include <boost/circular_buffer.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// SVM
#include "svm.h"
// Service request and response messages
#include "object3d_detector/ClassificationResult.h"
#include "object3d_detector/ClassifyClusters.h"
#include "bayes_people_tracker/TrackClusters.h"
// Point cloud frame message (contains point cloud and sensor position).
#include "hololens_depth_data_receiver_msgs/PointCloudFrame.h"

// A compile time switch which defines whether the individual clusters should be saved to disk. This doesn't need to
// be a parameter which can be configured over a parameter file, as we only need save some clusters for manual annotation.
// Furthermore, this only needs to be done once to train the SVM from scratch such that it's able to classify objects
// correctly in most cases. Fine-tuning can then be done online (as mentioned in the paper).
#define SAVE_CLUSTERS_TO_DISK false
#define POINT_CLOUD_PATH_PREFIX_RELATIVE_TO_HOME "/pointCloudFrames/"

typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  /*** for classification ***/
  int number_points;
  float min_distance;
  Eigen::Matrix3f covariance_3d;
  Eigen::Matrix3f moment_3d;
  // float partial_covariance_2d[9];
  // float histogram_main_2d[98];
  // float histogram_second_2d[45];
  float slice[20];
  float intensity[27];
} Feature;

static const int FEATURE_SIZE = 61;

typedef struct classifier {
  int object_class_id_;

  struct svm_model *svm_model_;
  bool is_probability_model_;
  float svm_scale_range_[FEATURE_SIZE][2];
  float x_lower_;
  float x_upper_;
} Classifier;

struct TrackingClassificationInfo {
  boost::circular_buffer<std::vector<int> > detected_classes_over_time;
  std::unordered_map<int, int> class_detection_counters;
  int total_detections;

  TrackingClassificationInfo(int num_frames_to_use, std::vector<int> class_ids) {
    detected_classes_over_time = boost::circular_buffer<std::vector<int> >(num_frames_to_use);

    for (const auto& class_id : class_ids)
      class_detection_counters[class_id] = 0;
    total_detections = 0;
  }

  TrackingClassificationInfo() = default;
};

class Object3dDetector {
private:
  /*** Publishers and Subscribers ***/
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher marker_array_pub_;
  ros::ServiceServer classification_service;
  ros::ServiceClient tracking_service;
  
  std::string frame_id_;
  float floor_height;
  float ceiling_height;
  float floor_ceiling_update_threshold;
  float floor_ceiling_noise_threshold;
  int min_points_in_floor_ceiling;
  int cluster_size_min_;
  int cluster_size_max_;

  struct svm_node *svm_node_;
  std::vector<int> class_ids_;
  std::vector<Classifier> classifiers_;
  bool use_svm_model_;
  float detection_probability_threshold_;
  int num_frames_to_use_;

  std::unordered_map<long, TrackingClassificationInfo> classification_info_for_tracks;

  std::unordered_map<int32_t, std_msgs::ColorRGBA> objectClassColors;
  
public:
  Object3dDetector();
  ~Object3dDetector();
  
  void pointCloudCallback(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg);

  bool classifyClustersCallback(
    object3d_detector::ClassifyClusters::Request& req, object3d_detector::ClassifyClusters::Response& res);

  std::vector<std::pair<long, pcl::PointXYZ> > trackClusters(const geometry_msgs::PoseArray& clusterCentroids, Eigen::Matrix4f& hololensToObject3dDetector);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
  std::vector<Feature> extractFeatures(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters);
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
		      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid);
  void saveFeature(Feature &f, struct svm_node *x);
  std::vector<std::vector<std::pair<int, float> > > classifyCurrentFrame(std::vector<Feature> features);
  std::vector<object3d_detector::ClassificationResult> classifyMultiFrames(std::vector<Feature> features,
          std::vector<std::vector<std::pair<int, float> > > classificationsCurrentFrame, std::vector<std::pair<long, pcl::PointXYZ> > trackedClusters);
};

Object3dDetector::Object3dDetector() {
  ros::NodeHandle public_nh;
  ros::NodeHandle private_nh("~");

  bool subscribeToHoloLensPointCloud = false;
  private_nh.param("subscribeToHoloLensPointCloud", subscribeToHoloLensPointCloud, false);
  if (subscribeToHoloLensPointCloud) {
    point_cloud_sub_ = public_nh.subscribe<hololens_depth_data_receiver_msgs::PointCloudFrame>("hololensLongThrowPointCloudFrame", 1, &Object3dDetector::pointCloudCallback, this);
    
    marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
    pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  }

  classification_service = private_nh.advertiseService("classifyClusters", &Object3dDetector::classifyClustersCallback, this);
  tracking_service = public_nh.serviceClient<bayes_people_tracker::TrackClusters>("clustersTrackerAllClassification/trackClusters");
  
  private_nh.param<std::string>("frame_id", frame_id_, "odom");
  private_nh.param<float>("floor_ceiling_update_threshold", floor_ceiling_update_threshold, 0.15);
  private_nh.param<float>("floor_ceiling_noise_threshold", floor_ceiling_noise_threshold, 0.1);
  private_nh.param<int>("min_points_in_floor_ceiling", min_points_in_floor_ceiling, 5000);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 5);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 30000);
  private_nh.param("detection_probability_threshold_", detection_probability_threshold_, 0.7f);
  private_nh.param("num_frames_to_use", num_frames_to_use_, 25);

  class_ids_.push_back(0);

  /****** load pre-trained svm models ******/
  use_svm_model_ = true;
  svm_node_ = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
  XmlRpc::XmlRpcValue classifiers;
  private_nh.getParam("classifiers", classifiers);
  ROS_ASSERT(classifiers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = classifiers.begin(); it != classifiers.end(); ++it) {
    ROS_INFO_STREAM("Found classifier: " << (std::string)(it->first) << " ==> " << classifiers[it->first]);

    Classifier classifier;
    classifier.object_class_id_ = classifiers[it->first]["class_id"];
    class_ids_.push_back(classifier.object_class_id_);

    std::string model_file_name = classifiers[it->first]["model_file"];
    std::string range_file_name = classifiers[it->first]["range_file"];

    if((classifier.svm_model_ = svm_load_model(model_file_name.c_str())) == NULL) {
      ROS_WARN("[object3d detector] can not load SVM model, use model-free detection.");
      use_svm_model_ = false;
    } else {
      ROS_INFO("[object3d detector] load SVM model from '%s'.", model_file_name.c_str());
      classifier.is_probability_model_ = svm_check_probability_model(classifier.svm_model_) ? true : false;
      
      // load range file, for more details: https://github.com/cjlin1/libsvm/
      std::fstream range_file;
      range_file.open(range_file_name.c_str(), std::fstream::in);
      if(!range_file.is_open()) {
        ROS_WARN("[object3d detector] can not load range file, use model-free detection.");
        use_svm_model_ = false;
      } else {
        ROS_INFO("[object3d detector] load SVM range from '%s'.", range_file_name.c_str());
        std::string line;
        std::vector<std::string> params;
        std::getline(range_file, line);
        std::getline(range_file, line);
        boost::split(params, line, boost::is_any_of(" "));
        classifier.x_lower_ = atof(params[0].c_str());
        classifier.x_upper_ = atof(params[1].c_str());
        int i = 0;
        while(std::getline(range_file, line)) {
          boost::split(params, line, boost::is_any_of(" "));
          classifier.svm_scale_range_[i][0] = atof(params[1].c_str());
          classifier.svm_scale_range_[i][1] = atof(params[2].c_str());
          i++;
        }
      }
    }

    classifiers_.push_back(classifier);
  }

  // Define the colors to use for visualizing the detected object class.
  std_msgs::ColorRGBA humanColor;
  humanColor.r = 0.0;
  humanColor.g = 1.0;
  humanColor.b = 0.5;
  humanColor.a = 1.0;
  objectClassColors[1] = humanColor;

  std_msgs::ColorRGBA robotColor;
  robotColor.r = 1.0;
  robotColor.g = 0.5;
  robotColor.b = 0.0;
  robotColor.a = 1.0;
  objectClassColors[2] = robotColor;

  std_msgs::ColorRGBA unknownColor;
  unknownColor.r = 0.5;
  unknownColor.g = 0.5;
  unknownColor.b = 0.5;
  unknownColor.a = 1.0;
  objectClassColors[0] = unknownColor;

  std_msgs::ColorRGBA classificationFailedColor;
  classificationFailedColor.r = 0.0;
  classificationFailedColor.g = 0.0;
  classificationFailedColor.b = 0.0;
  classificationFailedColor.a = 1.0;
  objectClassColors[-1] = classificationFailedColor;
}

Object3dDetector::~Object3dDetector() {
  for (size_t i = 0; i < classifiers_.size(); ++i) {
    if (classifiers_[i].svm_model_ != NULL)
      svm_free_and_destroy_model(&(classifiers_[i].svm_model_));
  }
  free(svm_node_);
}

int counter = 0;
bool Object3dDetector::classifyClustersCallback(
    object3d_detector::ClassifyClusters::Request& req, object3d_detector::ClassifyClusters::Response& res)
{
  if (!use_svm_model_)
    return false;

  // The HoloLens uses a slightly different coordinate system. Up corresponds to the y-axis instead of the z-axis.
  // We need to transform the point cloud accordingly such that the coordinate systems match up.
  Eigen::Matrix4f hololensToObject3dDetector = Eigen::Matrix4f::Identity();
  hololensToObject3dDetector.block(0, 0, 3, 3) = Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitX()).toRotationMatrix();

  // The same applies to the sensor position which also needs to be rotated accordingly.
  pcl::PointXYZ sensor_position = pcl::PointXYZ(req.sensorPosition.x, -req.sensorPosition.z, req.sensorPosition.y);

  // For classification the points need to be in a coordinate system relative to the sensor position (i.e. a coordinate
  // system in which the sensor is located at the origin), so we need to translate all points accordingly.
  // Please note that it is technically not quite correct to only translate along the XY plane, but not along the Z axis
  // (i.e. the height). However, it was chosen to only translate the points along the XY plane as this causes saved
  // point clouds being easier to annotate as the floor and ceiling threshold can be kept constant instead of needing to
  // be adjusted every so often (depending on how much the HoloLens is moved in height).
  Eigen::Matrix4f sensorPositionToOrigin
      = Eigen::Affine3f(Eigen::Translation3f(-sensor_position.x, -sensor_position.y, 0)).matrix();
  hololensToObject3dDetector = sensorPositionToOrigin * hololensToObject3dDetector;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  for (const auto& clusterMsg : req.clusters) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_hololens(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(clusterMsg, *cluster_hololens);

    // The HoloLens uses a slightly different coordinate system. Up corresponds to the y-axis instead of the z-axis.
    // We need to transform the point cloud accordingly such that the coordinate systems match up.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cluster_hololens, *cluster, hololensToObject3dDetector);

    clusters.push_back(cluster);
  }

  // Save the clusters to disk in case this is needed.
  if (SAVE_CLUSTERS_TO_DISK) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCombined(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& cluster : clusters)
      *clustersCombined += *cluster;

    std::string home = std::string(getenv("HOME"));
    std::string directory = home + POINT_CLOUD_PATH_PREFIX_RELATIVE_TO_HOME;
    std::string filename = boost::lexical_cast<std::string>(counter++);
    std::string prefix = directory + filename;
    boost::filesystem::create_directories(directory);
    pcl::io::savePCDFileBinary(prefix + ".pcd", *clustersCombined);
  }

  std::vector<std::pair<long, pcl::PointXYZ> > trackedClusters = trackClusters(req.clusterCentroids, hololensToObject3dDetector);
  std::vector<Feature> features = extractFeatures(clusters);
  std::vector<std::vector<std::pair<int, float> > > classificationsCurrentFrame = classifyCurrentFrame(features);
  res.classificationResults = classifyMultiFrames(features, classificationsCurrentFrame, trackedClusters);
  return true;
}

std::vector<std::pair<long, pcl::PointXYZ> > Object3dDetector::trackClusters(const geometry_msgs::PoseArray& clusterCentroids, Eigen::Matrix4f& hololensToObject3dDetector) {
  bayes_people_tracker::TrackClusters clusterTrackingMsg;
  clusterTrackingMsg.request.detectorName = "clusters_classifier";
  clusterTrackingMsg.request.clusterCenterPoints = clusterCentroids;
  bool trackingSuccess = tracking_service.call(clusterTrackingMsg);

  std::vector<std::pair<long, pcl::PointXYZ> > trackedPoints;

  if (trackingSuccess) {
    Eigen::Affine3f hololensToObject3dDetectorAffine;
    hololensToObject3dDetectorAffine.matrix() = hololensToObject3dDetector;

    for (size_t i = 0; i < clusterTrackingMsg.response.trackIds.size(); ++i) {
      const auto& trackId = clusterTrackingMsg.response.trackIds[i];
      const auto& pose = clusterTrackingMsg.response.trackedPoints[i];

      pcl::PointXYZ trackedPoint (pose.position.x, pose.position.y, pose.position.z);
      pcl::PointXYZ trackedPointTransformed = pcl::transformPoint(trackedPoint, hololensToObject3dDetectorAffine);
      
      trackedPoints.push_back(std::make_pair(trackId, trackedPointTransformed));
    }
  } else {
    ROS_WARN("Cluster tracking failed! Falling back to classifying clusters solely with information from the current frame...");
  }

  return trackedPoints;
}

int64_t clusterCentroidsSequenceNumber = 0;
void Object3dDetector::pointCloudCallback(const hololens_depth_data_receiver_msgs::PointCloudFrame::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_hololens(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg->pointCloudWorldSpace, *pcl_pc_hololens);

  // The HoloLens uses a slightly different coordinate system. Up corresponds to the y-axis instead of the z-axis.
  // We need to transform the point cloud accordingly such that the coordinate systems match up.
  Eigen::Matrix4f hololensToObject3dDetector = Eigen::Matrix4f::Identity();
  hololensToObject3dDetector.block(0, 0, 3, 3) = Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitX()).toRotationMatrix();

  // The same applies to the sensor position which also needs to be rotated accordingly.
  pcl::PointXYZ sensor_position
      = pcl::PointXYZ(msg->hololensPosition.point.x, -msg->hololensPosition.point.z, msg->hololensPosition.point.y);

  // For classification the points need to be in a coordinate system relative to the sensor position (i.e. a coordinate
  // system in which the sensor is located at the origin), so we need to translate all points accordingly.
  // Please note that it is technically not quite correct to only translate along the XY plane, but not along the Z axis
  // (i.e. the height). However, if the point cloud was also translated along the height axis, this would cause the
  // floor and ceiling detection to break. It was therefore chosen to only translate the points along the XY plane. This
  // also has the benefit of the saved point clouds being easier to annotate as the floor and ceiling threshold can be
  // kept constant instead of needing to be adjusted every so often (depending on how much the HoloLens is moved in
  // height).
  Eigen::Vector4f sensorTranslation(sensor_position.x, sensor_position.y, 0.0f, 0.0f);
  Eigen::Matrix4f sensorPositionToOrigin
      = Eigen::Affine3f(Eigen::Translation3f(-sensor_position.x, -sensor_position.y, 0)).matrix();
  hololensToObject3dDetector = sensorPositionToOrigin * hololensToObject3dDetector;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*pcl_pc_hololens, *pcl_pc, hololensToObject3dDetector);
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = extractCluster(pcl_pc);
  std::vector<Feature> features = extractFeatures(clusters);

  geometry_msgs::PoseArray clusterCentroids;
  clusterCentroids.header.seq = clusterCentroidsSequenceNumber++;
  clusterCentroids.header.stamp = msg->hololensPosition.header.stamp;
  clusterCentroids.header.frame_id = frame_id_;
  for (const auto& feature : features) {
    geometry_msgs::Pose pose;
    pose.position.x = feature.centroid.x();
    pose.position.y = feature.centroid.y();
    pose.position.z = feature.centroid.z();
    pose.orientation.w = 1.0;

    clusterCentroids.poses.push_back(pose);
  }
  std::vector<std::pair<long, pcl::PointXYZ> > trackedClusters = trackClusters(clusterCentroids, hololensToObject3dDetector);
  
  std::vector<std::vector<std::pair<int, float> > > classificationsCurrentFrame = classifyCurrentFrame(features);
  std::vector<object3d_detector::ClassificationResult> classificationsMultiFrame = classifyMultiFrames(features, classificationsCurrentFrame, trackedClusters);

  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseArray pose_array;
  for (size_t i = 0; i < classificationsMultiFrame.size(); i++)
  {
    const Feature& feature = features[i];
    const object3d_detector::ClassificationResult& classificationResult = classificationsMultiFrame[i];

    if (classificationResult.objectClass <= 0)
      continue;

    // Min, max and centroid need to be translated back into the original coordinate system.
    const Eigen::Vector4f centroid = feature.centroid + sensorTranslation;
    const Eigen::Vector4f min = feature.min + sensorTranslation;
    const Eigen::Vector4f max = feature.max + sensorTranslation;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.ns = "object3d";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = max[0]; p[0].y = max[1]; p[0].z = max[2];
    p[1].x = min[0]; p[1].y = max[1]; p[1].z = max[2];
    p[2].x = max[0]; p[2].y = max[1]; p[2].z = max[2];
    p[3].x = max[0]; p[3].y = min[1]; p[3].z = max[2];
    p[4].x = max[0]; p[4].y = max[1]; p[4].z = max[2];
    p[5].x = max[0]; p[5].y = max[1]; p[5].z = min[2];
    p[6].x = min[0]; p[6].y = min[1]; p[6].z = min[2];
    p[7].x = max[0]; p[7].y = min[1]; p[7].z = min[2];
    p[8].x = min[0]; p[8].y = min[1]; p[8].z = min[2];
    p[9].x = min[0]; p[9].y = max[1]; p[9].z = min[2];
    p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
    p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
    p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
    p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
    p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
    p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
    p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
    p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
    p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
    p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
    p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
    p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
    p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
    p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
    for(int i = 0; i < 24; i++)
      marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color = objectClassColors[classificationResult.objectClass];
    
    marker.lifetime = ros::Duration(0.25);
    marker_array.markers.push_back(marker);
    
    geometry_msgs::Pose pose;
    pose.position.x = centroid[0];
    pose.position.y = centroid[1];
    pose.position.z = centroid[2];
    pose.orientation.w = 1;
    pose_array.poses.push_back(pose);
  }

  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
  if(pose_array.poses.size()) {
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id_;
    pose_array_pub_.publish(pose_array);
  }
}

const int nested_regions_ = 14;
int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // for more details, see our IROS'17 paper.
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Object3dDetector::extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  float min_z_in_frame = 0.0;
  float max_z_in_frame = 0.0;
  for (int i = 0; i < pc->size(); i++) {
    float point_height = (*pc)[i].z;
    min_z_in_frame = std::min(point_height, min_z_in_frame);
    max_z_in_frame = std::max(point_height, max_z_in_frame);
  }

  if (min_z_in_frame < floor_height - floor_ceiling_update_threshold) {
    // There are points more than 15 cm below the currently assumed floor height. Check for a plane at that height.
    pcl::IndicesPtr point_indices_below_floor_height(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z_in_frame - floor_ceiling_noise_threshold, floor_height - floor_ceiling_noise_threshold);
    pass.filter(*point_indices_below_floor_height);

    if (point_indices_below_floor_height->size() >= min_points_in_floor_ceiling) {
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setDistanceThreshold(0.01);
      seg.setAxis(Eigen::Vector3f::UnitZ());
      seg.setEpsAngle(pcl::deg2rad(5.0));
      seg.setMaxIterations(1000);
      seg.setInputCloud(pc);
      seg.setIndices(point_indices_below_floor_height);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() >= min_points_in_floor_ceiling) {
        float plane_height = -coefficients->values[2] * coefficients->values[3]; // = -z * d
        float new_floor_height = plane_height + floor_ceiling_noise_threshold;
        if (new_floor_height < floor_height) {
          floor_height = new_floor_height;
          ROS_INFO("Updated floor height to %f", floor_height);
        }
      }
    }
  }

  if (max_z_in_frame > ceiling_height + floor_ceiling_update_threshold) {
    // There are points more than 15 cm above the currently assumed ceiling height. Check for a plane at that height.
    pcl::IndicesPtr point_indices_above_ceiling_height(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(ceiling_height + floor_ceiling_noise_threshold, max_z_in_frame + floor_ceiling_noise_threshold);
    pass.filter(*point_indices_above_ceiling_height);

    if (point_indices_above_ceiling_height->size() >= min_points_in_floor_ceiling) {
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setDistanceThreshold(0.01);
      seg.setAxis(Eigen::Vector3f::UnitZ());
      seg.setEpsAngle(pcl::deg2rad(5.0));
      seg.setMaxIterations(1000);
      seg.setInputCloud(pc);
      seg.setIndices(point_indices_above_ceiling_height);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() >= min_points_in_floor_ceiling) {
        float plane_height = -coefficients->values[2] * coefficients->values[3]; // = -z * d
        float new_ceiling_height = plane_height - floor_ceiling_noise_threshold;
        if (new_ceiling_height > ceiling_height) {
          ceiling_height = new_ceiling_height;
          ROS_INFO("Updated ceiling height to %f", ceiling_height);
        }
      }
    }
  }

  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(floor_height, ceiling_height);
  pass.filter(*pc_indices);
  
  boost::array<std::vector<int>, nested_regions_> indices_array;
  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < nested_regions_; j++) {
      float d2 = pc->points[(*pc_indices)[i]].x * pc->points[(*pc_indices)[i]].x +
	      pc->points[(*pc_indices)[i]].y * pc->points[(*pc_indices)[i]].y +
	      pc->points[(*pc_indices)[i]].z * pc->points[(*pc_indices)[i]].z;
      if(d2 > range*range && d2 <= (range+zone_[j])*(range+zone_[j])) {
      	indices_array[j].push_back((*pc_indices)[i]);
      	break;
      }
      range += zone_[j];
    }
  }
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

  float tolerance = 0.0;
  for(int i = 0; i < nested_regions_; i++) {
    tolerance += 0.1;
    if(indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pc, indices_array_ptr);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
      	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      	  cluster->points.push_back(pc->points[*pit]);
      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;

        clusters.push_back(cluster);
      }
    }
  }

  return clusters;
}

/* *** Feature Extraction ***
 * f1 (1d): the number of points included in a cluster.
 * f2 (1d): the minimum distance of the cluster to the sensor.
 * => f1 and f2 should be used in pairs, since f1 varies with f2 changes.
 * f3 (6d): 3D covariance matrix of the cluster.
 * f4 (6d): the normalized moment of inertia tensor.
 * => Since both f3 and f4 are symmetric, we only use 6 elements from each as features.
 * f5 (9d): 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
 * f6 (98d): The normalized 2D histogram for the main plane, 14 × 7 bins.
 * f7 (45d): The normalized 2D histogram for the secondary plane, 9 × 5 bins.
 * f8 (20d): Slice feature for the cluster.
 * f9 (27d): Intensity.
 */

void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment_3d) {
  moment_3d.setZero();
  for(size_t i = 0; i < pc.size(); i++) {
    moment_3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
    moment_3d(0,1) -= pc[i].x*pc[i].y;
    moment_3d(0,2) -= pc[i].x*pc[i].z;
    moment_3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
    moment_3d(1,2) -= pc[i].y*pc[i].z;
    moment_3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
  }
  moment_3d(1, 0) = moment_3d(0, 1);
  moment_3d(2, 0) = moment_3d(0, 2);
  moment_3d(2, 1) = moment_3d(1, 2);
}

/* Main plane is formed from the maximum and middle eigenvectors.
 * Secondary plane is formed from the middle and minimum eigenvectors.
 */
void computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Eigen::Matrix3f &eigenvectors, int axe, Eigen::Vector4f &centroid, pcl::PointCloud<pcl::PointXYZI>::Ptr plane) {
  Eigen::Vector4f coefficients;
  coefficients[0] = eigenvectors(0,axe);
  coefficients[1] = eigenvectors(1,axe);
  coefficients[2] = eigenvectors(2,axe);
  coefficients[3] = 0;
  coefficients[3] = -1 * coefficients.dot(centroid);
  for(size_t i = 0; i < pc->size(); i++) {
    float distance_to_plane =
      coefficients[0] * pc->points[i].x +
      coefficients[1] * pc->points[i].y +
      coefficients[2] * pc->points[i].z +
      coefficients[3];
    pcl::PointXYZI p;
    p.x = pc->points[i].x - distance_to_plane * coefficients[0];
    p.y = pc->points[i].y - distance_to_plane * coefficients[1];
    p.z = pc->points[i].z - distance_to_plane * coefficients[2];
    plane->points.push_back(p);
  }
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr plane, Eigen::Vector4f &mean, float *partial_covariance_2d) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr zone_decomposed[3];
  for(int i = 0; i < 3; i++)
    zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
  for(size_t i = 0; i < plane->size(); i++) {
    if(plane->points[i].z >= mean(2)) { // upper half
      zone_decomposed[0]->points.push_back(plane->points[i]);
    } else {
      if(plane->points[i].y >= mean(1)) // left lower half
        zone_decomposed[1]->points.push_back(plane->points[i]);
      else // right lower half
        zone_decomposed[2]->points.push_back(plane->points[i]);
    }
  }
  
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  for(int i = 0; i < 3; i++) {
    pcl::compute3DCentroid(*zone_decomposed[i], centroid);
    pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
    partial_covariance_2d[i*3+0] = covariance(0,0);
    partial_covariance_2d[i*3+1] = covariance(0,1);
    partial_covariance_2d[i*3+2] = covariance(1,1);
  }
}

void computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int horiz_bins, int verti_bins, float *histogram) {
  Eigen::Vector4f min, max, min_box, max_box;
  pcl::getMinMax3D(*pc, min, max);
  float horiz_itv, verti_itv;
  horiz_itv = (max[0]-min[0]>max[1]-min[1]) ? (max[0]-min[0])/horiz_bins : (max[1]-min[1])/horiz_bins;
  verti_itv = (max[2] - min[2])/verti_bins;
  
  for(int i = 0; i < horiz_bins; i++) {
    for(int j = 0; j < verti_bins; j++) {
      if(max[0]-min[0] > max[1]-min[1]) {
        min_box << min[0]+horiz_itv*i, min[1], min[2]+verti_itv*j, 0;
        max_box << min[0]+horiz_itv*(i+1), max[1], min[2]+verti_itv*(j+1), 0;
      } else {
        min_box << min[0], min[1]+horiz_itv*i, min[2]+verti_itv*j, 0;
        max_box << max[0], min[1]+horiz_itv*(i+1), min[2]+verti_itv*(j+1), 0;
      }
      std::vector<int> indices;
      pcl::getPointsInBox(*pc, min_box, max_box, indices);
      histogram[i*verti_bins+j] = (float)indices.size() / (float)pc->size();
    }
  }
}

void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, float *slice) {
  Eigen::Vector4f pc_min, pc_max;
  pcl::getMinMax3D(*pc, pc_min, pc_max);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr blocks[n];
  float itv = (pc_max[2] - pc_min[2]) / n;
  if(itv > 0) {
    for(int i = 0; i < n; i++) {
      blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    for(unsigned int i = 0, j; i < pc->size(); i++) {
      j = std::min((n-1), (int)((pc->points[i].z - pc_min[2]) / itv));
      blocks[j]->points.push_back(pc->points[i]);
    }
    
    Eigen::Vector4f block_min, block_max;
    for(int i = 0; i < n; i++) {
      if(blocks[i]->size() > 0) {
        // pcl::PCA<pcl::PointXYZI> pca;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr block_projected(new pcl::PointCloud<pcl::PointXYZI>);
        // pca.setInputCloud(blocks[i]);
        // pca.project(*blocks[i], *block_projected);
        pcl::getMinMax3D(*blocks[i], block_min, block_max);
      } else {
        block_min.setZero();
        block_max.setZero();
      }
      slice[i*2] = block_max[0] - block_min[0];
      slice[i*2+1] = block_max[1] - block_min[1];
    }
  } else {
    for(int i = 0; i < 20; i++)
      slice[i] = 0;
  }
}

void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, float *intensity) {
  float sum = 0, mean = 0, sum_dev = 0;
  float min = FLT_MAX, max = -FLT_MAX;
  for(int i = 0; i < 27; i++)
    intensity[i] = 0;
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum += pc->points[i].intensity;
    min = std::min(min, pc->points[i].intensity);
    max = std::max(max, pc->points[i].intensity);
  }
  mean = sum / pc->size();
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum_dev += (pc->points[i].intensity-mean)*(pc->points[i].intensity-mean);
    int ii = std::min(float(bins-1), std::floor((pc->points[i].intensity-min)/((max-min)/bins)));
    intensity[ii]++;
  }
  intensity[25] = sqrt(sum_dev/pc->size());
  intensity[26] = mean;
}

void Object3dDetector::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
				      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid) {
  f.centroid = centroid;
  f.min = min;
  f.max = max;
  
  if(use_svm_model_) {
    // f1: Number of points included the cluster.
    f.number_points = pc->size();
    // f2: The minimum distance to the cluster.
    f.min_distance = FLT_MAX;
    float d2; //squared Euclidean distance
    for(int i = 0; i < pc->size(); i++) {
      d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
      if(f.min_distance > d2)
	      f.min_distance = d2;
    }
    //f.min_distance = sqrt(f.min_distance);
    
    pcl::PCA<pcl::PointXYZI> pca;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZI>);
    pca.setInputCloud(pc);
    pca.project(*pc, *pc_projected);
    // f3: 3D covariance matrix of the cluster.
    pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
    // f4: The normalized moment of inertia tensor.
    computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
    // Navarro et al. assume that a pedestrian is in an upright position.
    //pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
    // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
    //compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
    // f6 and f7
    //computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
    //computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
    // f8
    computeSlice(pc, 10, f.slice);
    // f9
    computeIntensity(pc, 25, f.intensity);
  }
}

std::vector<Feature> Object3dDetector::extractFeatures(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters) {
  std::vector<Feature> features;
  for (const auto& cluster : clusters) {
    Eigen::Vector4f min, max, centroid;
    pcl::getMinMax3D(*cluster, min, max);
    pcl::compute3DCentroid(*cluster, centroid);

    Feature f;
    extractFeature(cluster, f, min, max, centroid);
    features.push_back(f);
  }

  return features;
}

void Object3dDetector::saveFeature(Feature &f, struct svm_node *x) {
  x[0].index  = 1;  x[0].value  = f.number_points; // libsvm indices start at 1
  x[1].index  = 2;  x[1].value  = f.min_distance;
  x[2].index  = 3;  x[2].value  = f.covariance_3d(0,0);
  x[3].index  = 4;  x[3].value  = f.covariance_3d(0,1);
  x[4].index  = 5;  x[4].value  = f.covariance_3d(0,2);
  x[5].index  = 6;  x[5].value  = f.covariance_3d(1,1);
  x[6].index  = 7;  x[6].value  = f.covariance_3d(1,2);
  x[7].index  = 8;  x[7].value  = f.covariance_3d(2,2);
  x[8].index  = 9;  x[8].value  = f.moment_3d(0,0);
  x[9].index  = 10; x[9].value  = f.moment_3d(0,1);
  x[10].index = 11; x[10].value = f.moment_3d(0,2);
  x[11].index = 12; x[11].value = f.moment_3d(1,1);
  x[12].index = 13; x[12].value = f.moment_3d(1,2);
  x[13].index = 14; x[13].value = f.moment_3d(2,2);
  // for(int i = 0; i < 9; i++) {
  //   x[i+14].index = i+15;
  //   x[i+14].value = f.partial_covariance_2d[i];
  // }
  // for(int i = 0; i < 98; i++) {
  // 	x[i+23].index = i+24;
  // 	x[i+23].value = f.histogram_main_2d[i];
  // }
  // for(int i = 0; i < 45; i++) {
  // 	x[i+121].index = i+122;
  // 	x[i+121].value = f.histogram_second_2d[i];
  // }
  for(int i = 0; i < 20; i++) {
    x[i+14].index = i+15;
    x[i+14].value = f.slice[i];
  }
  for(int i = 0; i < 27; i++) {
    x[i+34].index = i+35;
    x[i+34].value = f.intensity[i];
  }
  x[FEATURE_SIZE].index = -1;
  
  // for(int i = 0; i < FEATURE_SIZE; i++) {
  //   std::cerr << x[i].index << ":" << x[i].value << " ";
  //   std::cerr << std::endl;
  // }
}


std::vector<std::vector<std::pair<int, float> > > Object3dDetector::classifyCurrentFrame(std::vector<Feature> features) {
  std::vector<std::vector<std::pair<int, float> > > classificationResults;

  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    std::vector<std::pair<int, float> > classificationsCurrentCluster;
    
    if(use_svm_model_) {
      float largestProbability = 0.0f;
      for (size_t i = 0; i < classifiers_.size(); ++i) {
        Classifier& classifier = classifiers_[i];
      
        saveFeature(*it, svm_node_);

        // scale data
        for(int i = 0; i < FEATURE_SIZE; i++) {
          if(classifier.svm_scale_range_[i][0] == classifier.svm_scale_range_[i][1]) // skip single-valued attribute
            continue;
          if(svm_node_[i].value == classifier.svm_scale_range_[i][0])
            svm_node_[i].value = classifier.x_lower_;
          else if(svm_node_[i].value == classifier.svm_scale_range_[i][1])
            svm_node_[i].value = classifier.x_upper_;
          else
            svm_node_[i].value = classifier.x_lower_ + (classifier.x_upper_ - classifier.x_lower_) * (svm_node_[i].value - classifier.svm_scale_range_[i][0]) / (classifier.svm_scale_range_[i][1] - classifier.svm_scale_range_[i][0]);
        }

        // predict
        if(classifier.is_probability_model_) {
          double prob_estimates[classifier.svm_model_->nr_class];
          svm_predict_probability(classifier.svm_model_, svm_node_, prob_estimates);
          bool classified = prob_estimates[0] >= detection_probability_threshold_;
          if (classified) {
            classificationsCurrentCluster.push_back(std::make_pair(classifier.object_class_id_, prob_estimates[0]));
          }
          largestProbability = std::max(largestProbability, static_cast<float>(prob_estimates[0]));
        } else {
          bool classified = svm_predict(classifier.svm_model_, svm_node_) == 1;
          if (classified) {
            classificationsCurrentCluster.push_back(std::make_pair(classifier.object_class_id_, 1.0f));
            largestProbability = std::max(largestProbability, 1.0f);
          }
        }
      }

      if (classificationsCurrentCluster.size() == 0)
        classificationsCurrentCluster.push_back(std::make_pair(0, 1.0f - largestProbability));
    } else {
      // Can't predict any classes as we're not using the SVM due to some error during startup...
      classificationsCurrentCluster.push_back(std::make_pair(-1, 0.0f));
    }

    classificationResults.push_back(classificationsCurrentCluster);
  }

  return classificationResults;
}
  
std::vector<object3d_detector::ClassificationResult> Object3dDetector::classifyMultiFrames(std::vector<Feature> features, 
    std::vector<std::vector<std::pair<int, float> > > classificationsCurrentFrame, std::vector<std::pair<long, pcl::PointXYZ> > trackedClusters)
{
  std::vector<object3d_detector::ClassificationResult> classificationResults;

  for(size_t i = 0; i < features.size(); ++i) {
    Feature& feature = features[i];

    object3d_detector::ClassificationResult result;
    result.objectClass = 0;
    result.probability = 0.0;
    result.trackingId = -1;

    for(const auto& trackedCluster : trackedClusters) {
      // The tracker doesn't track along the height, meaning the tracked points all have a z-value of zero, so their
      // z-value can't be compared with the z-value of the cluster centroid.
      float dX = feature.centroid.x() - trackedCluster.second.x;
      float dY = feature.centroid.y() - trackedCluster.second.y;
      float squaredDistance = dX * dX + dY * dY;
      if (squaredDistance <= 0.1 * 0.1) {
        result.trackingId = trackedCluster.first;
        break;
      }
    }

    if(use_svm_model_) {
      std::vector<std::pair<int, float> >& clusterClassifications = classificationsCurrentFrame[i];

      if (result.trackingId == -1) {
        // We don't have any tracking information about the current cluster, so we have to classify it solely based on
        // the classifications made in this frame.
        for (const auto& classification : clusterClassifications) {
          if (classification.second > result.probability) {
            result.objectClass = classification.first;
            result.probability = classification.second;
          }
        }
      } else {
        if (classification_info_for_tracks.find(result.trackingId) == classification_info_for_tracks.end()) {
          classification_info_for_tracks[result.trackingId] = TrackingClassificationInfo(num_frames_to_use_, class_ids_);
        }
        TrackingClassificationInfo& trackingClassificationInfo = classification_info_for_tracks[result.trackingId];

        // Remove information for frames which are too far in the past.
        if (trackingClassificationInfo.detected_classes_over_time.full()) {
          for (const auto& classId : trackingClassificationInfo.detected_classes_over_time.front()) {
            trackingClassificationInfo.class_detection_counters[classId]--;
          }
          trackingClassificationInfo.total_detections -= trackingClassificationInfo.detected_classes_over_time.front().size();
          trackingClassificationInfo.detected_classes_over_time.pop_front();
        }

        // Add information about the current frame.
        std::vector<int> detectedClassesThisFrame;
        for (const auto& classification : clusterClassifications) {
          const int& classId = classification.first;
          detectedClassesThisFrame.push_back(classId);
          trackingClassificationInfo.class_detection_counters[classId]++;
        }
        trackingClassificationInfo.detected_classes_over_time.push_back(detectedClassesThisFrame);
        trackingClassificationInfo.total_detections += detectedClassesThisFrame.size();

        // Determine the class which was detected the most amount within the last frames.
        int greatestCount = 0;
        for (const auto& detectionCounter : trackingClassificationInfo.class_detection_counters) {
          const int& classId = detectionCounter.first;
          const int& detectionCount = detectionCounter.second;

          if (detectionCount > greatestCount) {
            result.objectClass = classId;
            greatestCount = detectionCount;
          }
        }
        result.probability = static_cast<float>(greatestCount) / static_cast<float>(trackingClassificationInfo.total_detections);
      }
    } else {
      // Can't predict any classes as we're not using the SVM due to some error during startup...
      result.objectClass = -1;
    }

    classificationResults.push_back(result);
  }

  return classificationResults;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object3d_detector");
  Object3dDetector d;
  ros::spin();
  return 0;
}
