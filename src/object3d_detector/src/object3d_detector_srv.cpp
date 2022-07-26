// This file was not part of the repository published at https://github.com/yzrobot/online_learning. It was added as
// part of my work to be able to use the human classifier from other ROS nodes by the use of a service.
// Please note that my original intent was to also get rid of the code duplications found between object_3d_detector.cpp
// and object_3d_detector_ol.cpp, but upon further inspection I had to find that there are sometimes subtle differences
// between methods of the same name found in these two files. As I don't have the time to perform a complete refactoring
// of these two files, I decided to copy the code from object3d_detector.cpp and use that as a starting point for my
// modifications to make the human classifier run as a service. It should however be noted that this will definitely 
// result in even more code duplication...



// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
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
#include "object3d_detector/ClassifyClusters.h"

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

class Object3dDetector {
private:
  /*** Publishers and Subscribers ***/
  ros::NodeHandle node_handle_;
  ros::ServiceServer classification_service;
  
  std::string model_file_name_;
  std::string range_file_name_;
  struct svm_node *svm_node_;
  struct svm_model *svm_model_;
  bool use_svm_model_;
  bool is_probability_model_;
  float svm_scale_range_[FEATURE_SIZE][2];
  float x_lower_;
  float x_upper_;
  float human_probability_;
  
public:
  Object3dDetector();
  ~Object3dDetector();
  
  bool classifyClustersCallback(
    object3d_detector::ClassifyClusters::Request& req, object3d_detector::ClassifyClusters::Response& res);
  std::vector<std::pair<bool, double>> classifyClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters, pcl::PointXYZ sensor_position);

  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
		      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid, pcl::PointXYZ sensor_position);
  void saveFeature(Feature &f, struct svm_node *x);
  std::vector<std::pair<bool, double>> classify(std::vector<Feature> features);
};

Object3dDetector::Object3dDetector() {
  ros::NodeHandle private_nh("~");
  classification_service = private_nh.advertiseService("classifyClusters", &Object3dDetector::classifyClustersCallback, this);
  
  /****** load a pre-trained svm model ******/
  private_nh.param<std::string>("model_file_name", model_file_name_, "");
  private_nh.param<std::string>("range_file_name", range_file_name_, "");
  
  use_svm_model_ = false;
  if((svm_model_ = svm_load_model(model_file_name_.c_str())) == NULL) {
    ROS_WARN("[object3d detector] can not load SVM model, use model-free detection.");
  } else {
    ROS_INFO("[object3d detector] load SVM model from '%s'.", model_file_name_.c_str());
    is_probability_model_ = svm_check_probability_model(svm_model_)?true:false;
    svm_node_ = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
    
    // load range file, for more details: https://github.com/cjlin1/libsvm/
    std::fstream range_file;
    range_file.open(range_file_name_.c_str(), std::fstream::in);
    if(!range_file.is_open()) {
      ROS_WARN("[object3d detector] can not load range file, use model-free detection.");
    } else {
      ROS_INFO("[object3d detector] load SVM range from '%s'.", range_file_name_.c_str());
      std::string line;
      std::vector<std::string> params;
      std::getline(range_file, line);
      std::getline(range_file, line);
      boost::split(params, line, boost::is_any_of(" "));
      x_lower_ = atof(params[0].c_str());
      x_upper_ = atof(params[1].c_str());
      int i = 0;
      while(std::getline(range_file, line)) {
        boost::split(params, line, boost::is_any_of(" "));
        svm_scale_range_[i][0] = atof(params[1].c_str());
        svm_scale_range_[i][1] = atof(params[2].c_str());
        i++;
        //std::cerr << i << " " <<  svm_scale_range_[i][0] << " " << svm_scale_range_[i][1] << std::endl;
      }
      use_svm_model_ = true;
    }
  }
}

Object3dDetector::~Object3dDetector() {
  if(use_svm_model_) {
    svm_free_and_destroy_model(&svm_model_);
    free(svm_node_);
  }
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

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  for (const auto& clusterMsg : req.clusters)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_hololens(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(clusterMsg, *cluster_hololens);

    // The HoloLens uses a slightly different coordinate system. Up corresponds to the y-axis instead of the z-axis.
    // We need to transform the point cloud accordingly such that the coordinate systems match up.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cluster_hololens, *cluster, hololensToObject3dDetector);

    clusters.push_back(cluster);
  }

  // The same applies to the sensor position which also needs to be rotated accordingly.
  pcl::PointXYZ sensor_position = pcl::PointXYZ(req.sensorPosition.x, -req.sensorPosition.z, req.sensorPosition.y);

  // Save the clusters to disk in case this is needed.
  if (SAVE_CLUSTERS_TO_DISK) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCombined(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& cluster : clusters)
      *clustersCombined += *cluster;

    // The annotation tool assumes that the sensor is located at the origin of the coordinate system, so we need to
    // translate the point cloud accordingly.
    // Note that I'm only translating along the XY plane in the object3d_detector coordinate system, but not along the
    // Z axis (which corresponds to the height). While this is technically not correct, it makes it a bit easier to
    // annotate the data as the floor and ceiling threshold can be kept constant in the annotation tool instead of
    // needing to be changed every so often (depending on how much the HoloLens is moved in height).
    Eigen::Matrix4f sensorPositionToOrigin
        = Eigen::Affine3f(Eigen::Translation3f(-sensor_position.x, -sensor_position.y, 0.0f)).matrix();
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCombinedTranslated(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*clustersCombined, *clustersCombinedTranslated, sensorPositionToOrigin);

    std::string home = std::string(getenv("HOME"));
    std::string directory = home + POINT_CLOUD_PATH_PREFIX_RELATIVE_TO_HOME;
    std::string filename = boost::lexical_cast<std::string>(counter++);
    std::string prefix = directory + filename;
    boost::filesystem::create_directories(directory);
    pcl::io::savePCDFileBinary(prefix + ".pcd", *clustersCombinedTranslated);
  }

  std::vector<std::pair<bool, double>> classificationResults = classifyClusters(clusters, sensor_position);
  for (std::pair<bool, double> result : classificationResults)
  {
    res.classificationResults.push_back(result.first);
    res.probabilities.push_back(result.second);
  }

  return true;
}

std::vector<std::pair<bool, double>> Object3dDetector::classifyClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters, pcl::PointXYZ sensor_position)
{
  std::vector<Feature> features;
  for (const auto& cluster : clusters)
  {
    Eigen::Vector4f min, max, centroid;
    pcl::getMinMax3D(*cluster, min, max);
    pcl::compute3DCentroid(*cluster, centroid);

    Feature f;
    extractFeature(cluster, f, min, max, centroid, sensor_position);
    features.push_back(f);
  }

  return classify(features);
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
				      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid, pcl::PointXYZ sensor_position) {
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
      const pcl::PointXYZI& point = pc->points[i];
      pcl::PointXYZ diff_to_sensor_position;
      diff_to_sensor_position.getArray3fMap() = point.getArray3fMap() - sensor_position.getArray3fMap();
      d2 = diff_to_sensor_position.x * diff_to_sensor_position.x + 
          diff_to_sensor_position.y * diff_to_sensor_position.y + 
          diff_to_sensor_position.z * diff_to_sensor_position.z;

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
  
std::vector<std::pair<bool, double>> Object3dDetector::classify(std::vector<Feature> features) {
  std::vector<std::pair<bool, double>> classificationResults;

  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    if(use_svm_model_) {
      saveFeature(*it, svm_node_);
      //std::cerr << "test_id = " << it->id << ", number_points = " << it->number_points << ", min_distance = " << it->min_distance << std::endl;

      // scale data
      for(int i = 0; i < FEATURE_SIZE; i++) {
      	if(svm_scale_range_[i][0] == svm_scale_range_[i][1]) // skip single-valued attribute
      	  continue;
      	if(svm_node_[i].value == svm_scale_range_[i][0])
      	  svm_node_[i].value = x_lower_;
      	else if(svm_node_[i].value == svm_scale_range_[i][1])
      	  svm_node_[i].value = x_upper_;
      	else
      	  svm_node_[i].value = x_lower_ + (x_upper_ - x_lower_) * (svm_node_[i].value - svm_scale_range_[i][0]) / (svm_scale_range_[i][1] - svm_scale_range_[i][0]);
      }

      // predict
      if(is_probability_model_) {
	      double prob_estimates[svm_model_->nr_class];
      	svm_predict_probability(svm_model_, svm_node_, prob_estimates);
        classificationResults.push_back(std::make_pair(prob_estimates[0] >= human_probability_, prob_estimates[0]));
      } else {
        bool classified = svm_predict(svm_model_, svm_node_) == 1;
        classificationResults.push_back(std::make_pair(classified, classified ? 1.0 : 0.0));
      }
    } else {
      classificationResults.push_back(std::make_pair(false, 0.0));
    }
  }

  return classificationResults;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object3d_detector");
  Object3dDetector d;
  ros::spin();
  return 0;
}
