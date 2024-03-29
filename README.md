## ROS HoloLens 2 data receiver

A Catkin workspace containing various ROS nodes for receiving sensor data streamed by the HoloLens 2 and for performing higher level stuff (mapping of the environment, tracking of moving objects, ...) with that data.

### Build instructions

Ensure that the following prerequisites are installed on your system:
 * [ROS](http://wiki.ros.org/ROS/Installation) (The PC used at the IPR had ROS Kinetic installed)
 * [Rosbridge](https://index.ros.org/p/rosbridge_suite/github-RobotWebTools-rosbridge_suite/)
 * [RViz](http://wiki.ros.org/rviz)
 * [rqt](https://index.ros.org/p/rqt/github-ros-visualization-rqt/)
 * PCL ([PCL itself](https://pointclouds.org/) (The PC used at the IPR had PCL 1.8.1 installed), [pcl_ros](https://index.ros.org/p/pcl_ros/github-ros-perception-perception_pcl/), [pcl_conversions](https://index.ros.org/p/pcl_conversions/github-ros-perception-perception_pcl/) and [pcl_msgs](https://index.ros.org/p/pcl_msgs/github-ros-perception-pcl_msgs/))
 * OctoMap ([octomap](https://index.ros.org/p/octomap/github-octomap-octomap/), [octomap_ros](https://index.ros.org/p/octomap_ros/github-OctoMap-octomap_ros/), [octomap_msgs](https://index.ros.org/p/octomap_msgs/github-octomap-octomap_msgs/) and [octomap_rviz_plugins](https://index.ros.org/p/octomap_rviz_plugins/github-OctoMap-octomap_rviz_plugins/))
 * OpenCV ([OpenCV itself](https://opencv.org/) (The PC used at the IPR had OpenCV 3.3.1 installed) and [cv_bridge](https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/))

The following instructions for building the ROS node will assume that Catkin is used as the build tool. In case you're using a different build tool, you may need to modify the command line prompts accordingly.

After the prerequisites were installed, create a new catkin workspace into which this repository will be cloned into. You should now be able to build everything using `catkin build`. This command may need to be executed multiple times as there are dependencies between the individual ROS packages contained in this repository. Do not worry if there are build errors during the first few executions of that command as these are very likely caused by these dependencies between packages. You should however start to worry if the build did not finish successfully after the fifth (or so) try. This very likely means that I may have forgotten to add some other required ROS package to the list of prerequisites or that there is in fact some other issue related to the versions of the used packages...

### Launch instructions

After the build finished successfully, there should now be multiple launch configurations to choose from. First of all, start off by ensuring that you have sourced the ROS nodes in your terminal by executing `source devel/setup.bash` (this assumes that the current working directory of your terminal is the root directory of the catkin workspace, so you may need to modify the path accordingly in case your current working directory is some other directory).

The following list states the ROS nodes and launch configurations which you may be most likely looking for. There are in fact more launch configurations than what is stated in that list. However, the launch configurations which are not listed in the following list were only used to test certain parts of the implemented code and as launch configurations to be included in other launch configurations.
 * [naive_spatial_mapper](src/naive_spatial_mapper/): A naive spatial mapper which adds point clouds obtained over multiple frames into a single joint point cloud. Does not have any special logic for removing points belonging to moving objects, so the resulting point cloud will contain ghost points of any object which (was) moved during the mapping process. This is the mapping algorithm developed for my Bachelor's thesis, and it was used during experiment 1. Can be used with the following launch configuration:
    * `roslaunch naive_spatial_mapper naive_spatial_mapper_with_rqt_gui.launch` (The corresponding RViz configuration file is located at src/naive_spatial_mapper/config/spatial_map.rviz)
 * [octomap_based_spatial_mapper](src/octomap_based_spatial_mapper/): A spatial mapper which creates a voxel representation of the environment based on the approach described in [1]. Can distinguish between static (i.e. not moving) and dynamic (i.e. moving) voxels. Dynamic voxels are clustered into separate objects and each cluster is tracked individually over time. This is the first MOT approach explained in my Master's thesis. Can be used with the following launch configuration:
    * `roslaunch octomap_based_spatial_mapper octomap_based_spatial_mapper_with_rqt_gui.launch` (The corresponding RViz configuration file is located at src/octomap_based_spatial_mapper/config/octomap_spatial_map.rviz)
 * [object3d_detector](src/object3d_detector/): An object detector based on the approach proposed in [3] and [4]. Clusters the captured point cloud using an adaptive clustering algorithm, classifies each cluster using an SVM and tracks each cluster using Kalman filters. The SVM can then be trained online using the information obtained by the SVM and the object tracker. This is the second MOT approach explained in my Master's thesis. Can be used with the following launch configurations:
    * `roslaunch object3d_detector object3d_detector.launch` (This launch configuration is almost identical to the code published with the original paper (modifications made: coordinate system transformation, detection of floor and ceiling height). Can only distinguish between human and non-human clusters. No online learning.)
    * `roslaunch object3d_detector object3d_detector_ol.launch` (This launch configuration is almost identical to the code published with the original paper (modifications made: coordinate system transformation, detection of floor and ceiling height). Can only distinguish between human and non-human clusters. With online learning (starts training the SVM from scratch).)
    * `roslaunch object3d_detector object3d_detector_multiclass_with_rviz.launch` (This launch configuration contains two SVMs (one for detecting humans and another one for detecting robot arms). No online learning. All experiments conducted with this MOT approach were performed using this launch configuration.)
 * [hololens_stereo_image_receiver](src/hololens_stereo_image_receiver/): A ROS node for calculating the disparity map and the corresponding point cloud of the stereo images streamed by the HoloLens 2. The original plan was to also implement the MOT approach presented in [2] (which is the third MOT approach explained in my Master's thesis), however based on the results observed so far for only the disparity map and the reconstructed point cloud, it was determined that it would not be worth even trying to implement the MOT approach. Can be used with the following launch configurations:
    * `roslaunch hololens_stereo_image_receiver stereo_image_receiver_with_rqt_gui.launch` (This launch configuration only calculates a disparity map and reconstructs a point cloud. The corresponding RViz configuration file is located at src/hololens_stereo_image_receiver/config/stereo_image_receiver.rviz)
    * `roslaunch hololens_stereo_image_receiver point_cloud_comparison_with_depth_sensor.launch` (This launch configuration calculates a disparity map, reconstructs a point cloud and uses the depth data receiver to also reconstruct a point cloud from the data obtained by the depth sensor. This launch configuration was used for comparing the point cloud reconstructed via disparity mapping with the point cloud reconstructed from the depth sensor data. The corresponding RViz configuration file is located at src/hololens_stereo_image_receiver/config/point_cloud_comparison_with_depth_sensor.rviz)

##### A note regarding RViz configuration files

It appears as if there is no way of automatically loading an RViz configuration file when RViz is embedded into rqt (at least for the versions of rqt and RViz installed on the PC which I used at the IPR), so the RViz configuration files need to be loaded manually for all launch configurations which open an rqt window in which RViz is embedded. The paths to the corresponding RViz configuration files are denoted in the list of launch configurations above were needed.

##### A note regarding recording Rosbag files

When recording rosbags containing depth data, it is important to start the rosbag recorder before the depth stream is started in the application running on the HoloLens 2. This ensures that the pixel directions which are only sent once are also part of the captured rosbag file.

### References

[1]: Asma Azim and Olivier Aycard. Detection, classification and tracking of moving objects in a 3d environment. In 2012 IEEE Intelligent Vehicles Symposium, pages 802-807. IEEE, 2012.

[2]: Goran Popović, Antea Hadviger, Ivan Marković and Ivan Petrović. Computationally efficient dense moving object detection based on reduced space disparity estimation. IFAC-PapersOnLine, 51(22):360-365, 2018.

[3]: Zhi Yan, Tom Duckett and Nicola Bellotto. Online learning for human classification in 3D LiDAR-based tracking. In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 864-871. IEEE, 2017.

[4]: Zhi Yan, Tom Duckett and Nicola Bellotto. Online learning for 3D LiDAR-based human detection: Experimental analysis of point cloud clustering and classification methods. Autonomous Robots, 44(2):147-164, 2020.
