#include "StereoImageReceiver.h"

// A compile time switch which defines whether the raw images should be saved to disk. This doesn't need to be a
// parameter which can be configured over a parameter file, as the images only need to be saved once for camera
// calibration.
#define SAVE_IMAGES_TO_DISK false
#define IMAGE_PATH_PREFIX_RELATIVE_TO_HOME "/stereoCameraImgs/"

StereoImageReceiver::StereoImageReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating StereoImageReceiver...");
    ROS_INFO("Using OpenCV version %s", CV_VERSION);

    // Initialize all sensor intrinsics of the stereo camera as defined by configuration (or default values).
    std::string stereoCamCalibrationFilePath;
    n.param("stereoCamCalibrationFilePath", stereoCamCalibrationFilePath, std::string(""));
    cv::FileStorage fs1(stereoCamCalibrationFilePath, cv::FileStorage::READ);
    fs1["K1"] >> KLeft;
    fs1["K2"] >> KRight;
    fs1["D1"] >> DLeft;
    fs1["D2"] >> DRight;
    fs1["R"] >> R;
    fs1["T"] >> T;
    fs1["R1"] >> RLeft;
    fs1["R2"] >> RRight;
    fs1["P1"] >> PLeft;
    fs1["P2"] >> PRight;
    fs1["Q"] >> Q;
    undistortRectifyMapInitialized = false;
    
    std::string leftCamCalibrationFilePath;
    std::string rightCamCalibrationFilePath;
    n.param("leftCamCalibrationFilePath", leftCamCalibrationFilePath, std::string(""));
    n.param("rightCamCalibrationFilePath", rightCamCalibrationFilePath, std::string(""));
    camera_calibration_parsers::readCalibration(leftCamCalibrationFilePath, leftCameraName, leftCameraInfo);
    camera_calibration_parsers::readCalibration(rightCamCalibrationFilePath, rightCameraName, rightCameraInfo);

    // Initialize all parameters for preprocessing as defined by configuration (or default values).
    n.param("preprocessingDoImageNormalization", preprocessingDoImageNormalization, false);
    n.param("preprocessingDoHistogramEqualization", preprocessingDoHistogramEqualization, false);
    n.param("preprocessingDoMedianFiltering", preprocessingDoMedianFiltering, false);
    n.param("preprocessingMedianFilterKernelSize", preprocessingMedianFilterKernelSize, 3);

    // Initialize all parameters for OpenCV's StereoSGBM algorithm as defined by configuration (or default values).
    n.param("sgbmMinDisparity", sgbmMinDisparity, 0);
    n.param("sgbmNumDisparities", sgbmNumDisparities, 64);
    if (sgbmNumDisparities % 16 != 0)
        sgbmNumDisparities += 16 - (sgbmNumDisparities % 16);
    n.param("sgbmBlockSize", sgbmBlockSize, 9);
    int sgbmBlockSizeSquared = sgbmBlockSize * sgbmBlockSize;
    n.param("sgbmP1Multiplier", sgbmP1Multiplier, 8);
    n.param("sgbmP2Multiplier", sgbmP2Multiplier, 32);
    n.param("sgbmDisp12MaxDiff", sgbmDisp12MaxDiff, 50);
    n.param("sgbmPreFilterCap", sgbmPreFilterCap, 0);
    n.param("sgbmUniquenessRatio", sgbmUniquenessRatio, 10);
    n.param("sgbmSpeckleWindowSize", sgbmSpeckleWindowSize, 100);
    n.param("sgbmSpeckleRange", sgbmSpeckleRange, 1);
    n.param("sgbmMode", sgbmMode, 1);
    leftMatcher = cv::StereoSGBM::create(sgbmMinDisparity, sgbmNumDisparities, sgbmBlockSize,
            sgbmP1Multiplier * sgbmBlockSizeSquared, sgbmP2Multiplier * sgbmBlockSizeSquared, sgbmDisp12MaxDiff,
            sgbmPreFilterCap, sgbmUniquenessRatio, sgbmSpeckleWindowSize, sgbmSpeckleRange,
            sgbmMode == 0 ? cv::StereoSGBM::MODE_SGBM : sgbmMode == 1 ? cv::StereoSGBM::MODE_HH : sgbmMode == 2 ? cv::StereoSGBM::MODE_SGBM_3WAY : cv::StereoSGBM::MODE_HH4);
    rightMatcher = cv::ximgproc::createRightMatcher(leftMatcher);

    // Initialize all parameters for OpenCV's DisparityWLSFilter as defined by configuration (or default values).
    n.param("wlsLambda", wlsLambda, 8000.0);
    n.param("wlsSigma", wlsSigma, 1.2);
    wlsFilter = cv::ximgproc::createDisparityWLSFilter(leftMatcher);
    wlsFilter->setLambda(wlsLambda);
    wlsFilter->setSigmaColor(wlsSigma);

    // Initialize all parameters for the disparity map visualization as defined by configuration (or default values).
    n.param("dispVisMultiplier", dispVisMultiplier, 2.0);

    // Initialize all parameters for the point cloud reconstruction as defined by configuration (or default values).
    n.param("reconstructPointCloudFromRawDisparityMap", reconstructPointCloudFromRawDisparityMap, false);
    n.param("minDisparityForReconstruction", minDisparityForReconstruction, 10.0f);

    // Initialize all parameters for point cloud downsampling as defined by configuration (or default values).
    n.param("doDownsampling", doDownsampling, true);
    n.param("downsamplingLeafSize", downsamplingLeafSize, 0.01f);

    // Initialize all parameters for point cloud outlier removal using a radius outlier removal filter as defined by configuration (or default values).
    n.param("doOutlierRemovalRadius", doOutlierRemovalRadius, true);
    n.param("outlierRemovalRadiusSearch", outlierRemovalRadiusSearch, 0.05);
    n.param("outlierRemovalMinNeighborsInRadius", outlierRemovalMinNeighborsInRadius, 12);

    // Initialize all parameters for point cloud outlier removal using a statistical outlier removal filter as defined by configuration (or default values).
    n.param("doOutlierRemovalStatistical", doOutlierRemovalStatistical, false);
    n.param("outlierRemovalNeighborsToCheck", outlierRemovalNeighborsToCheck, 10);
    n.param("outlierRemovalStdDeviationMultiplier", outlierRemovalStdDeviationMultiplier, 0.5);

    // Initialize all parameters for point cloud outlier removal using euclidean clustering as defined by configuration (or default values).
    n.param("doOutlierRemovalClustering", doOutlierRemovalClustering, false);
    n.param("outlierRemovalClusterTolerance", outlierRemovalClusterTolerance, 0.08);
    n.param("outlierRemovalMinClusterSize", outlierRemovalMinClusterSize, 500);

    cameraInfoLeftPublisher = n.advertise<sensor_msgs::CameraInfo>(STEREO_IMAGE_LEFT_CAM_INFO_TOPIC, 10);
    cameraInfoRightPublisher = n.advertise<sensor_msgs::CameraInfo>(STEREO_IMAGE_RIGHT_CAM_INFO_TOPIC, 10);
    stereoImageLeftRawPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_LEFT_RAW_TOPIC, 10);
    stereoImageRightRawPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_RIGHT_RAW_TOPIC, 10);
    stereoImageLeftPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_LEFT_TOPIC, 10);
    stereoImageRightPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_RIGHT_TOPIC, 10);
    disparityMapRawPublisher = n.advertise<sensor_msgs::Image>(DISPARITY_MAP_RAW_TOPIC, 10);
    disparityMapPublisher = n.advertise<sensor_msgs::Image>(DISPARITY_MAP_TOPIC, 10);
    stereoCamLeftPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_LEFT_POSITION_TOPIC, 10);
    stereoCamRightPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_RIGHT_POSITION_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);
    pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 10);

    sequenceNumber = 0;
}

void StereoImageReceiver::handleReconfiguration(
    hololens_stereo_image_receiver::StereoImageReceiverConfig& config,
    uint32_t level)
{
    if (level >= 256)
        return;

    ROS_INFO("Reconfiguring with level %u...", level);
    
    if (level & 1)
    {
        ROS_INFO("Preprocessing parameters were changed.");

        preprocessingDoImageNormalization = config.preprocessingDoImageNormalization;
        preprocessingDoHistogramEqualization = config.preprocessingDoHistogramEqualization;
        preprocessingDoMedianFiltering = config.preprocessingDoMedianFiltering;
        preprocessingMedianFilterKernelSize = config.preprocessingMedianFilterKernelSize;
    }

    if (level & 2)
    {
        ROS_INFO("SGBM parameters were changed.");

        sgbmMinDisparity = config.sgbmMinDisparity;
        sgbmNumDisparities = config.sgbmNumDisparities;
        if (sgbmNumDisparities % 16 != 0)
            sgbmNumDisparities += 16 - (sgbmNumDisparities % 16);
        sgbmBlockSize = config.sgbmBlockSize;
        sgbmP1Multiplier = config.sgbmP1Multiplier;
        sgbmP2Multiplier = config.sgbmP2Multiplier;
        sgbmDisp12MaxDiff = config.sgbmDisp12MaxDiff;
        sgbmPreFilterCap = config.sgbmPreFilterCap;
        sgbmUniquenessRatio = config.sgbmUniquenessRatio;
        sgbmSpeckleWindowSize = config.sgbmSpeckleWindowSize;
        sgbmSpeckleRange = config.sgbmSpeckleRange;
        sgbmMode = config.sgbmMode;

        updateMatchers = true;
        updateWlsFilter = true;
    }

    if (level & 4)
    {
        ROS_INFO("WLS parameters were changed.");

        wlsLambda = config.wlsLambda;
        wlsSigma = config.wlsSigma;

        updateWlsFilter = true;
    }

    if (level & 8)
    {
        ROS_INFO("Point cloud reconstruction parameters were changed.");

        reconstructPointCloudFromRawDisparityMap = config.reconstructPointCloudFromRawDisparityMap;
        minDisparityForReconstruction = config.minDisparityForReconstruction;
    }

    if (level & 16)
    {
        ROS_INFO("Point cloud downsampling parameters were changed.");

        doDownsampling = config.doDownsampling;
        downsamplingLeafSize = config.downsamplingLeafSize;
    }

    if (level & 32)
    {
        ROS_INFO("Radius outlier removal filter parameters were changed.");

        doOutlierRemovalRadius = config.doOutlierRemovalRadius;
        outlierRemovalRadiusSearch = config.outlierRemovalRadiusSearch;
        outlierRemovalMinNeighborsInRadius = config.outlierRemovalMinNeighborsInRadius;
    }

    if (level & 64)
    {
        ROS_INFO("Statistical outlier removal filter parameters were changed.");

        doOutlierRemovalStatistical = config.doOutlierRemovalStatistical;
        outlierRemovalNeighborsToCheck = config.outlierRemovalNeighborsToCheck;
        outlierRemovalStdDeviationMultiplier = config.outlierRemovalStdDeviationMultiplier;
    }

    if (level & 128)
    {
        ROS_INFO("Euclidean clustering outlier removal filter parameters were changed.");

        doOutlierRemovalClustering = config.doOutlierRemovalClustering;
        outlierRemovalClusterTolerance = config.outlierRemovalClusterTolerance;
        outlierRemovalMinClusterSize = config.outlierRemovalMinClusterSize;
    }
}

void StereoImageReceiver::handleStereoCameraFrame(const hololens_msgs::StereoCameraFrame::ConstPtr& msg)
{
    if (updateMatchers)
    {
        updateMatchers = false;

        int sgbmBlockSizeSquared = sgbmBlockSize * sgbmBlockSize;
        leftMatcher = cv::StereoSGBM::create(sgbmMinDisparity, sgbmNumDisparities, sgbmBlockSize,
            sgbmP1Multiplier * sgbmBlockSizeSquared, sgbmP2Multiplier * sgbmBlockSizeSquared, sgbmDisp12MaxDiff,
            sgbmPreFilterCap, sgbmUniquenessRatio, sgbmSpeckleWindowSize, sgbmSpeckleRange,
            sgbmMode == 0 ? cv::StereoSGBM::MODE_SGBM : sgbmMode == 1 ? cv::StereoSGBM::MODE_HH : sgbmMode == 2 ? cv::StereoSGBM::MODE_SGBM_3WAY : cv::StereoSGBM::MODE_HH4);
        rightMatcher = cv::ximgproc::createRightMatcher(leftMatcher);
    }

    if (updateWlsFilter)
    {
        updateWlsFilter = false;

        wlsFilter = cv::ximgproc::createDisparityWLSFilter(leftMatcher);
        wlsFilter->setLambda(wlsLambda);
        wlsFilter->setSigmaColor(wlsSigma);
    }

    // Decode the two images.
    std::string decodedLeft = base64_decode(msg->base64encodedImageLeft);
    std::string decodedRight = base64_decode(msg->base64encodedImageRight);
    Image imageLeft = Image(decodedLeft, msg->imageWidthLeft, msg->imageHeightLeft, msg->pixelStrideLeft, false);
    Image imageRight = Image(decodedRight, msg->imageWidthRight, msg->imageHeightRight, msg->pixelStrideRight, false);

    // Determine the HoloLens's position within its world coordinate system.
    const hololens_msgs::Point& translationLeft = msg->camToWorldTranslationLeft;
    const hololens_msgs::Point& translationRight = msg->camToWorldTranslationRight;
    hololens_msgs::Point translationCenter;
    translationCenter.x = (translationLeft.x + translationRight.x) / 2.0;
    translationCenter.y = (translationLeft.y + translationRight.y) / 2.0;
    translationCenter.z = (translationLeft.z + translationRight.z) / 2.0;

    // Determine the time delta between the two images.
    // One clock tick of the HoloLens 2 corresponds to 100 nanoseconds, so there are 10 000 000 ticks per second or 
    // 10 000 ticks per millisecond.
    const int64_t& timeLeftTicks = msg->timeLeft;
    const int64_t& timeRightTicks = msg->timeRight;
    double timeLeftMillis = timeLeftTicks / 10000.0;
    double timeRightMillis = timeRightTicks / 10000.0;
    double timeDifferenceMillis = timeRightMillis - timeLeftMillis;
    ROS_INFO("Time left: %f, time right: %f, time difference: %f ms", timeLeftMillis, timeRightMillis, timeDifferenceMillis);

    // Convert the two images to sensor_msgs::Image instances.
    ros::Time time = ros::Time::now();
    sensor_msgs::Image imageMsgLeft = imageToMsg(imageLeft, "hololens_stereo_cam_left", sequenceNumber, time);
    sensor_msgs::Image imageMsgRight = imageToMsg(imageRight, "hololens_stereo_cam_right", sequenceNumber, time);

    // Convert the two images to OpenCV images.
    cv_bridge::CvImageConstPtr imageLeftOpenCV = cv_bridge::toCvShare(imageMsgLeft, msg);
    cv_bridge::CvImageConstPtr imageRightOpenCV = cv_bridge::toCvShare(imageMsgRight, msg);

    // Rotate the images such that they are upright. Code copied from https://stackoverflow.com/a/16278334.
    cv::Mat imageLeftOpenCVUpright;
    cv::Mat imageRightOpenCVUpright;
    // The left image needs to be rotated 90 degrees clockwise.
    cv::transpose(imageLeftOpenCV->image, imageLeftOpenCVUpright);
    cv::flip(imageLeftOpenCVUpright, imageLeftOpenCVUpright, 1);
    // The right image needs to be rotated 90 degrees counterclockwise.
    cv::transpose(imageRightOpenCV->image, imageRightOpenCVUpright);
    cv::flip(imageRightOpenCVUpright, imageRightOpenCVUpright, 0);

    // Save the raw images to disk (for calibrating the cameras)
    if (SAVE_IMAGES_TO_DISK)
    {
        std::string home = std::string(getenv("HOME"));

        std::string pathPrefix = home + IMAGE_PATH_PREFIX_RELATIVE_TO_HOME;
        std::string leftDirectory = pathPrefix + "left/";
        std::string rightDirectory = pathPrefix + "right/";
        boost::filesystem::create_directories(leftDirectory);
        boost::filesystem::create_directories(rightDirectory);

        std::string fileName = std::to_string(sequenceNumber);
        std::string leftPath = leftDirectory + fileName + ".png";
        std::string rightPath = rightDirectory + fileName + ".png";
        cv::imwrite(leftPath.c_str(), imageLeftOpenCVUpright);
        cv::imwrite(rightPath.c_str(), imageRightOpenCVUpright);
    }

    // Perform image preprocessing if needed.
    if (preprocessingDoImageNormalization)
    {
        cv::normalize(imageLeftOpenCVUpright, imageLeftOpenCVUpright, 0, 255, cv::NORM_MINMAX);
        cv::normalize(imageRightOpenCVUpright, imageRightOpenCVUpright, 0, 255, cv::NORM_MINMAX);
    }
    if (preprocessingDoHistogramEqualization)
    {
        cv::equalizeHist(imageLeftOpenCVUpright, imageLeftOpenCVUpright);
        cv::equalizeHist(imageRightOpenCVUpright, imageRightOpenCVUpright);
    }
    if (preprocessingDoMedianFiltering)
    {
        cv::medianBlur(imageLeftOpenCVUpright, imageLeftOpenCVUpright, preprocessingMedianFilterKernelSize);
        cv::medianBlur(imageRightOpenCVUpright, imageRightOpenCVUpright, preprocessingMedianFilterKernelSize);
    }

    // Rectify the images.
    if (!undistortRectifyMapInitialized)
    {
        cv::stereoRectify(KLeft, DLeft, KRight, DRight, imageLeftOpenCVUpright.size(), R, T, RLeft, RRight, PLeft, PRight, Q,
                cv::CALIB_ZERO_DISPARITY, -1.0, cv::Size(), &validPixelsRectLeft, &validPixelsRectRight);

        cv::initUndistortRectifyMap(KLeft, DLeft, RLeft, PLeft, imageLeftOpenCVUpright.size(), CV_32F, map1Left, map2Left);
        cv::initUndistortRectifyMap(KRight, DRight, RRight, PRight, imageRightOpenCVUpright.size(), CV_32F, map1Right, map2Right);
        undistortRectifyMapInitialized = true;
    }
    cv::Mat leftRectified, rightRectified;
    cv::remap(imageLeftOpenCVUpright, leftRectified, map1Left, map2Left, cv::INTER_LINEAR);
    cv::remap(imageRightOpenCVUpright, rightRectified, map1Right, map2Right, cv::INTER_LINEAR);

    // Perform stereo matching using semi-global matching (SGM). Somewhat copied (and modified) from copied from
    // https://docs.opencv.org/3.3.1/d3/d14/tutorial_ximgproc_disparity_filtering.html
    cv::Mat leftDisparity, rightDisparity;
    leftMatcher->compute(leftRectified, rightRectified, leftDisparity);
    rightMatcher->compute(rightRectified, leftRectified, rightDisparity);

    // Perform filtering. Somewhat copied (and modified) from copied from
    // https://docs.opencv.org/3.3.1/d3/d14/tutorial_ximgproc_disparity_filtering.html
    cv::Mat filteredDisparity;
    wlsFilter->filter(leftDisparity, leftRectified, filteredDisparity, rightDisparity, validPixelsRectLeft);

    // Create a point cloud from the filtered disparity map.
    // Data type of the disparity map is 16SC1, so a 16 bit signed short with 4 fixed binary digits after the decimal
    // dot and 1 channel per pixel. According to the documentation of reprojectImageTo3D these values should therefore
    // be divided by 16 and scaled to float. If some other disparity map estimation algorithm is used, the disparity map
    // may have some other data type, so this may not be needed in case OpenCV's StereoSGBM algorithm is replaced with
    // some other algorithm.
    cv::Mat reconstructionDisparityAsFloats;
    cv::Mat& reconstructionDisparity = reconstructPointCloudFromRawDisparityMap ? leftDisparity : filteredDisparity;
    reconstructionDisparity.convertTo(reconstructionDisparityAsFloats, CV_32F, 1.0/16.0);
    cv::Mat points;
    cv::reprojectImageTo3D(reconstructionDisparityAsFloats, points, Q);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZI>());
    for (uint32_t v = validPixelsRectLeft.y; v < validPixelsRectLeft.height; v++)
    {
        float* disparityRow = reconstructionDisparityAsFloats.ptr<float>(v);
        cv::Vec3f* pointsRow = points.ptr<cv::Vec3f>(v);
        uchar* grayscaleRow = leftRectified.ptr<uchar>(v);

        for (uint32_t u = validPixelsRectLeft.x; u < validPixelsRectLeft.width; u++)
        {
            // Ensure that the current pixel was actually mapped to some valid point in 3D space.
            const float disparity = disparityRow[u];
            if (disparity <= minDisparityForReconstruction)
                continue;

            // Rotate each point by 90 degrees counterclockwise around the z-axis. This is needed to undo the rotation
            // performed on the left image. If we didn't rotate the points, the resulting point cloud would be rotated
            // by 90 degrees clockwise around the camera's z-axis instead of being upright.
            pcl::PointXYZI point;
            point.x = pointsRow[u][1];
            point.y = -pointsRow[u][0];
            point.z = pointsRow[u][2];
            point.intensity = static_cast<float>(grayscaleRow[u]) / 255.0;
            pointCloudCamSpace->push_back(point);
        }
    }

    // Downsample the point cloud to ensure that the point density is not too high.
    if (doDownsampling)
        pointCloudCamSpace = downsamplePointCloud(pointCloudCamSpace, downsamplingLeafSize);

    // Remove outliers (errors caused by wrong disparity estimations, extremely noisy values, ...) from the point cloud.
    if (doOutlierRemovalRadius)
    {
        pointCloudCamSpace = removeOutliersRadius(pointCloudCamSpace, outlierRemovalRadiusSearch,
                outlierRemovalMinNeighborsInRadius);
    }
    if (doOutlierRemovalStatistical)
    {
        pointCloudCamSpace = removeOutliersStatistical(pointCloudCamSpace, outlierRemovalNeighborsToCheck,
                outlierRemovalStdDeviationMultiplier);
    }
    if (doOutlierRemovalClustering)
    {
        pointCloudCamSpace = removeOutliersClustering(pointCloudCamSpace, outlierRemovalClusterTolerance,
                outlierRemovalMinClusterSize);
    }

    // Calculate the transformation from camera space to world space and transform the point cloud.
    Eigen::Matrix4f camToWorld = computeCamToWorldFromStereoCameraFrame(msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudWorldSpace (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*pointCloudCamSpace, *pointCloudWorldSpace, camToWorld);

    // Publish the images, the disparity map, the point cloud and the HoloLens's current position.
    publishHololensPosition(translationLeft, stereoCamLeftPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationRight, stereoCamRightPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationCenter, hololensPositionPublisher, sequenceNumber, time);
    publishHololensCamToWorldTf(translationLeft, msg->camToWorldRotationLeft, "hololens_stereo_cam_left", hololensCamLeftPublisher, time);
    publishHololensCamToWorldTf(translationRight, msg->camToWorldRotationRight, "hololens_stereo_cam_right", hololensCamRightPublisher, time);
    publishCameraInfo(leftCameraInfo, cameraInfoLeftPublisher, "hololens_stereo_cam_left", sequenceNumber, time);
    publishCameraInfo(rightCameraInfo, cameraInfoRightPublisher, "hololens_stereo_cam_right", sequenceNumber, time);
    stereoImageLeftRawPublisher.publish(imageToMsg(imageLeftOpenCVUpright, "hololens_stereo_cam_left", sequenceNumber, time));
    stereoImageRightRawPublisher.publish(imageToMsg(imageRightOpenCVUpright, "hololens_stereo_cam_right", sequenceNumber, time));
    stereoImageLeftPublisher.publish(imageToMsg(leftRectified, "hololens_stereo_cam_left", sequenceNumber, time));
    stereoImageRightPublisher.publish(imageToMsg(rightRectified, "hololens_stereo_cam_right", sequenceNumber, time));
    publishDisparityMap(leftDisparity, disparityMapRawPublisher, sequenceNumber, time);
    publishDisparityMap(filteredDisparity, disparityMapPublisher, sequenceNumber, time);
    publishPointCloud(pointCloudWorldSpace, pointCloudPublisher, sequenceNumber, time, "hololens_world");
    sequenceNumber++;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StereoImageReceiver::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToDownsample,
    const float leafSize)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudDownsampled (new pcl::PointCloud<pcl::PointXYZI>());

    // Downsample the given point cloud using a voxel grid filter.
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setInputCloud(pointCloudToDownsample);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*pointCloudDownsampled);

    // Return the downsampled point cloud.
    return pointCloudDownsampled;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StereoImageReceiver::removeOutliersRadius(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
    const float radiusSearch,
    const int minNeighborsInRadius)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZI>());

    // Remove outliers by using a radius outlier removal.
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierRemoval;
    radiusOutlierRemoval.setInputCloud(pointCloudToFilter);
    radiusOutlierRemoval.setRadiusSearch(radiusSearch);
    radiusOutlierRemoval.setMinNeighborsInRadius(minNeighborsInRadius);
    radiusOutlierRemoval.filter(*pointCloudFiltered);

    // Return the point cloud with the outliers removed.
    return pointCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StereoImageReceiver::removeOutliersStatistical(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
    const float numNeighborsToCheck,
    const float standardDeviationMultiplier)
{
    // Create a point cloud in which we will store the results.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZI>());

    // Remove outliers by using a statistical outlier removal.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(pointCloudToFilter);
    statisticalOutlierRemoval.setMeanK(numNeighborsToCheck);
    statisticalOutlierRemoval.setStddevMulThresh(standardDeviationMultiplier);
    statisticalOutlierRemoval.filter(*pointCloudFiltered);

    // Return the point cloud with the outliers removed.
    return pointCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StereoImageReceiver::removeOutliersClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
    const double clusterTolerance,
    const int minClusterSize)
{
    // Set up a KD tree for searching inside the cloud.
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(pointCloudToFilter);
    
    // Create a vector for storing the detected point indices of each cluster.
    std::vector<pcl::PointIndices> clusters;

    // Extract the clusters of the point cloud.
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance);
    clusterExtraction.setSearchMethod(tree);
    clusterExtraction.setInputCloud(pointCloudToFilter);
    clusterExtraction.extract(clusters);

    // Create an indices extractor for removing each cluster from the cloud.
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(pointCloudToFilter);

    // Iterate over all found clusters and add all big enough clusters to the filtered point cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPointCloud (new pcl::PointCloud<pcl::PointXYZI>());
    for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
    {
        if (iter->indices.size() >= minClusterSize)
        {
            for (const auto& idx : iter->indices)
            {
                filteredPointCloud->push_back((*pointCloudToFilter)[idx]);
            }
        }
    }

    return filteredPointCloud;
}

Eigen::Matrix4f StereoImageReceiver::computeCamToWorldFromStereoCameraFrame(
    const hololens_msgs::StereoCameraFrame::ConstPtr& stereoCamFrame)
{
    // Create the camera to world transformation matrix which will be returned later on.
    Eigen::Matrix4f camToWorld = Eigen::Matrix4f::Identity();

    // Set the rotational part of the camera to world transformation matrix.
    camToWorld.block(0, 0, 3, 3) = Eigen::Quaternionf(
        stereoCamFrame->camToWorldRotationLeft.w, 
        stereoCamFrame->camToWorldRotationLeft.x, 
        stereoCamFrame->camToWorldRotationLeft.y, 
        stereoCamFrame->camToWorldRotationLeft.z
    ).toRotationMatrix();

    // Set the translational part of the camera to world transformation matrix.
    camToWorld(0, 3) = stereoCamFrame->camToWorldTranslationLeft.x;
    camToWorld(1, 3) = stereoCamFrame->camToWorldTranslationLeft.y;
    camToWorld(2, 3) = stereoCamFrame->camToWorldTranslationLeft.z;

    // Return the camera to world transformation matrix.
    return camToWorld;
}

void StereoImageReceiver::publishCameraInfo(
    sensor_msgs::CameraInfo& cameraInfo,
    const ros::Publisher& publisher,
    const std::string frameId,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    cameraInfo.header.frame_id = frameId;
    cameraInfo.header.seq = sequenceNumber;
    cameraInfo.header.stamp = timestamp;

    publisher.publish(cameraInfo);
}

sensor_msgs::Image StereoImageReceiver::imageToMsg(
    const Image image,
    const std::string frameId,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    // Create the ROS message for the image.
    sensor_msgs::Image imageMsg;

    // Set the header of the image.
    imageMsg.header.seq = sequenceNumber;
    imageMsg.header.stamp = timestamp;
    imageMsg.header.frame_id = frameId;

    // Set the meta data (width, height, encoding, ...) of the image.
    imageMsg.height = image.height;
    imageMsg.width = image.width;
    imageMsg.encoding = sensor_msgs::image_encodings::MONO8;
    imageMsg.is_bigendian = 0;
    imageMsg.step = image.width;

    // Set the actual pixel data of the image. Image channels will be downscaled to 8 bit per pixel if necessary.
    uint32_t pixelValueDivisor = static_cast<uint32_t>(pow(2.0, static_cast<float>((image.pixelStride - 1) * 8)));
    imageMsg.data.reserve(image.height * image.width);
    for (int v = 0; v < image.height; v++)
    {
        for (int u = 0; u < image.width; u++)
        {
            uint8_t pixelValue = static_cast<uint8_t>(image.valueAt(u, v) / pixelValueDivisor);
            imageMsg.data.push_back(pixelValue);
        }
    }

    return imageMsg;
}

cv_bridge::CvImage StereoImageReceiver::imageToMsg(
    const cv::Mat& image,
    const std::string frameId,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    cv_bridge::CvImage imageMsg;

    imageMsg.header.seq = sequenceNumber;
    imageMsg.header.stamp = timestamp;
    imageMsg.header.frame_id = frameId;

    imageMsg.encoding = sensor_msgs::image_encodings::MONO8;
    imageMsg.image = image;

    return imageMsg;
}

void StereoImageReceiver::publishImage(
    const Image image,
    const ros::Publisher& publisher,
    const std::string frameId,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    // Create the ROS message for the image.
    sensor_msgs::Image imageMsg;

    // Set the header of the image.
    imageMsg.header.seq = sequenceNumber;
    imageMsg.header.stamp = timestamp;
    imageMsg.header.frame_id = frameId;

    // Set the meta data (width, height, encoding, ...) of the image.
    imageMsg.height = image.height;
    imageMsg.width = image.width;
    imageMsg.encoding = sensor_msgs::image_encodings::MONO8;
    imageMsg.is_bigendian = 0;
    imageMsg.step = image.width;

    // Set the actual pixel data of the image. Image channels will be downscaled to 8 bit per pixel if necessary.
    uint32_t pixelValueDivisor = static_cast<uint32_t>(pow(2.0, static_cast<float>((image.pixelStride - 1) * 8)));
    imageMsg.data.reserve(image.height * image.width);
    for (int v = 0; v < image.height; v++)
    {
        for (int u = 0; u < image.width; u++)
        {
            uint8_t pixelValue = static_cast<uint8_t>(image.valueAt(u, v) / pixelValueDivisor);
            imageMsg.data.push_back(pixelValue);
        }
    }

    // Publish the message.
    publisher.publish(imageMsg);
}

void StereoImageReceiver::publishDisparityMap(
    const cv::Mat& disparityMap,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    cv::Mat disparityVisualization;
    cv::ximgproc::getDisparityVis(disparityMap, disparityVisualization, dispVisMultiplier);

    cv_bridge::CvImage disparityMsg;
    disparityMsg.header.seq = sequenceNumber;
    disparityMsg.header.stamp = timestamp;
    disparityMsg.header.frame_id = "hololens_stereo_cam_left";
    disparityMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    disparityMsg.image = disparityVisualization;

    publisher.publish(disparityMsg);
}

void StereoImageReceiver::pointCloudToMsg(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    sensor_msgs::PointCloud2& message,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    // Create the ROS message for the point cloud and store the point cloud inside it.
    pcl::toROSMsg(*cloud, message);

    // Set the header of the message.
    message.header.seq = sequenceNumber;
    message.header.stamp = timestamp;
    message.header.frame_id = frameId;
}

void StereoImageReceiver::publishPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp,
    const std::string frameId)
{
    sensor_msgs::PointCloud2 pointCloudMessage;
    pointCloudToMsg(cloud, pointCloudMessage, sequenceNumber, timestamp, frameId);

    publisher.publish(pointCloudMessage);
}

void StereoImageReceiver::publishHololensPosition(
    const hololens_msgs::Point& position,
    const ros::Publisher& publisher,
    uint32_t sequenceNumber,
    const ros::Time& timestamp)
{
    // Create the ROS message for the current position of the HoloLens.
    geometry_msgs::PointStamped hololensPosition;

    // Set the header of the message.
    hololensPosition.header.seq = sequenceNumber;
    hololensPosition.header.stamp = timestamp;
    hololensPosition.header.frame_id = "hololens_world";

    // Add the position of the HoloLens to the message.
    hololensPosition.point.x = position.x;
    hololensPosition.point.y = position.y;
    hololensPosition.point.z = position.z;

    // Publish the message.
    publisher.publish(hololensPosition);
}

void StereoImageReceiver::publishHololensCamToWorldTf(
    const hololens_msgs::Point& translation,
    const hololens_msgs::Quaternion& rotation,
    const std::string frameId,
    tf::TransformBroadcaster& publisher,
    const ros::Time& timestamp)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(translation.x, translation.y, translation.z));
    transform.setRotation(tf::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
    
    publisher.sendTransform(tf::StampedTransform(transform, timestamp, "hololens_world", frameId));
}