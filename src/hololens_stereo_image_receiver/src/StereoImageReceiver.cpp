#include "StereoImageReceiver.h"

// A compile time switch which defines whether the raw images should be saved to disk. This doesn't need to be a
// parameter which can be configured over a parameter file, as the images only need to be saved once for camera
// calibration.
#define SAVE_IMAGES_TO_DISK false
// Hardcoding file paths is definitely not a good idea, but using ~ in the path didn't work...
#define IMAGE_PATH_PREFIX "/home/boesing_uvrtq/stereoCameraImgs/"

StereoImageReceiver::StereoImageReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating StereoImageReceiver...");

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
    n.param("sgbmUseModeHH", sgbmUseModeHH, true);
    leftMatcher = cv::StereoSGBM::create(sgbmMinDisparity, sgbmNumDisparities, sgbmBlockSize,
            sgbmP1Multiplier * sgbmBlockSizeSquared, sgbmP2Multiplier * sgbmBlockSizeSquared, sgbmDisp12MaxDiff,
            sgbmPreFilterCap, sgbmUniquenessRatio, sgbmSpeckleWindowSize, sgbmSpeckleRange,
            sgbmUseModeHH ? cv::StereoSGBM::MODE_HH : cv::StereoSGBM::MODE_SGBM);
    rightMatcher = cv::ximgproc::createRightMatcher(leftMatcher);

    // Initialize all parameters for OpenCV's DisparityWLSFilter as defined by configuration (or default values).
    n.param("wlsLambda", wlsLambda, 8000.0);
    n.param("wlsSigma", wlsSigma, 1.2);
    wlsFilter = cv::ximgproc::createDisparityWLSFilter(leftMatcher);
    wlsFilter->setLambda(wlsLambda);
    wlsFilter->setSigmaColor(wlsSigma);

    // Initialize all parameters for the disparity map visualization as defined by configuration (or default values).
    n.param("dispVisMultiplier", dispVisMultiplier, 2.0);
    n.param("dispVisVisualizeRawDisparityMap", dispVisVisualizeRawDisparityMap, false);

    stereoImageLeftPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_LEFT_TOPIC, 10);
    stereoImageRightPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_RIGHT_TOPIC, 10);
    disparityMapPublisher = n.advertise<sensor_msgs::Image>(DISPARITY_MAP_TOPIC, 10);
    stereoCamLeftPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_LEFT_POSITION_TOPIC, 10);
    stereoCamRightPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_RIGHT_POSITION_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);
    pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 10);

    stereoPixelDirections = hololens_msgs::StereoPixelDirections::Ptr(new hololens_msgs::StereoPixelDirections());
    sequenceNumber = 0;
}

void StereoImageReceiver::handleStereoPixelDirections(const hololens_msgs::StereoPixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu and %zu pixel directions for the left and right camera respectively.", 
            msg->pixelDirectionsLeft.size(), msg->pixelDirectionsRight.size());

    // The pixel directions are not used yet when a stereo camera frame is received, but we'll already store a copy of
    // them in case we need them later on.
    hololens_msgs::StereoPixelDirections::Ptr pixelDirections = hololens_msgs::StereoPixelDirections::Ptr(new hololens_msgs::StereoPixelDirections());
    for (uint32_t i = 0; i < msg->pixelDirectionsLeft.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = msg->pixelDirectionsLeft[i];
        pixelDirections->pixelDirectionsLeft.push_back(dir);
    }
    for (uint32_t i = 0; i < msg->pixelDirectionsRight.size(); ++i)
    {
        hololens_msgs::PixelDirection dir = msg->pixelDirectionsRight[i];
        pixelDirections->pixelDirectionsRight.push_back(dir);
    }
    stereoPixelDirections = pixelDirections;
}

void StereoImageReceiver::handleStereoCameraFrame(const hololens_msgs::StereoCameraFrame::ConstPtr& msg)
{
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
    float distX = translationLeft.x - translationRight.x;
    float distY = translationLeft.y - translationRight.y;
    float distZ = translationLeft.z - translationRight.z;
    float baselineDistance = sqrt(distX * distX + distY * distY + distZ * distZ);

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
        std::string pathPrefix = std::string(IMAGE_PATH_PREFIX);
        std::string fileName = std::to_string(sequenceNumber);
        std::string leftPath = pathPrefix + "left/" + fileName + ".png";
        std::string rightPath = pathPrefix + "right/" + fileName + ".png";
        cv::imwrite(leftPath.c_str(), imageLeftOpenCVUpright);
        cv::imwrite(rightPath.c_str(), imageRightOpenCVUpright);
    }

    // TODO: Check whether the images are already rectified by the HoloLens or whether this needs to be done here.

    // Perform stereo matching using semi-global matching (SGM). Somewhat copied (and modified) from copied from
    // https://docs.opencv.org/4.x/d3/d14/tutorial_ximgproc_disparity_filtering.html
    cv::Mat leftDisparity, rightDisparity;
    leftMatcher->compute(imageLeftOpenCVUpright, imageRightOpenCVUpright, leftDisparity);
    rightMatcher->compute(imageRightOpenCVUpright, imageLeftOpenCVUpright, rightDisparity);

    // Perform filtering. Somewhat copied (and modified) from copied from
    // https://docs.opencv.org/4.x/d3/d14/tutorial_ximgproc_disparity_filtering.html
    cv::Mat filteredDisparity;
    wlsFilter->filter(leftDisparity, imageLeftOpenCVUpright, filteredDisparity, rightDisparity);

    // Rotate the filtered disparity map by 90 degrees counterclockwise such that it matches with the raw image data
    // obtained by the left camera. This is done to allow for an easier access to the received pixel directions.
    cv::Mat filteredDisparityInLeftCamCoordSystem;
    cv::transpose(filteredDisparity, filteredDisparityInLeftCamCoordSystem);
    cv::flip(filteredDisparityInLeftCamCoordSystem, filteredDisparityInLeftCamCoordSystem, 0);

    // The HoloLens 2's RGB camera has a focal length of 4.87 mm +/- 5%. However, there is no data about the focal
    // length of the greyscale cameras. For now, I'm incorrectly assuming the greyscale cameras to have the same focal
    // length. The greyscale cameras need to be calibrated at some point in the future and then this value needs to be
    // changed accordingly.
    float focalLength = 4.87 * 100.0; // I don't quite understand why this value has to be scaled up so much...
    float minDisparityForReconstruction = 10.0;

    // Create a point cloud from the filtered disparity map.
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudCamSpace (new pcl::PointCloud<pcl::PointXYZI>());
    for (uint32_t i = 0; i < stereoPixelDirections->pixelDirectionsLeft.size(); ++i)
    {
        const hololens_msgs::PixelDirection& dir = stereoPixelDirections->pixelDirectionsLeft[i];
        // Data type of the disparity map is 16SC1, so a 16 bit signed short with 4 fixed binary digits after the
        // decimal dot and 1 channel per pixel. If some other disparity map estimation algorithm is used, the disparity
        // map may have some other data type, so this may also need to be changed in case OpenCV's StereoSGBM algorithm
        // is replaced with some other algorithm.
        // Yes, u and v really need to be flipped to be in the order v, u instead of the normally expected order u, v.
        float disparity = filteredDisparityInLeftCamCoordSystem.at<short>(dir.v, dir.u) / 16.0;
        if (disparity > minDisparityForReconstruction)
        {
            // Disparity d is equal to B * f / Z where B is the baseline distance between the two cameras, f is the
            // focal length and Z is the corresponding depth. Solving for Z, this equation becomes Z = B * f / d.
            float depth = baselineDistance * focalLength / disparity;

            pcl::PointXYZI point;
            point.x = dir.direction.x * depth;
            point.y = dir.direction.y * depth;
            point.z = dir.direction.z * depth;
            // TODO: These three lines are just here for debugging purposes. They need to be removed later on.
            // point.x = dir.u * 0.01;
            // point.y = dir.v * 0.01;
            // point.z = depth;
            point.intensity = static_cast<float>(imageLeft.valueAt(dir.u, dir.v)) / 255.0;
            pointCloudCamSpace->push_back(point);
        }
    }

    // Publish the point cloud.
    sensor_msgs::PointCloud2 pointCloudMessage;
    pcl::toROSMsg(*pointCloudCamSpace, pointCloudMessage);
    pointCloudMessage.header.seq = sequenceNumber;
    pointCloudMessage.header.stamp = time;
    pointCloudMessage.header.frame_id = "hololens_stereo_cam_left";
    pointCloudPublisher.publish(pointCloudMessage);

    // Publish the disparity map.
    cv::Mat disparityVisualization;
    cv::ximgproc::getDisparityVis(dispVisVisualizeRawDisparityMap ? leftDisparity : filteredDisparity,
            disparityVisualization, dispVisMultiplier);
    cv_bridge::CvImage disparityMsg;
    disparityMsg.header.seq = sequenceNumber;
    disparityMsg.header.stamp = time;
    disparityMsg.header.frame_id = "hololens_stereo_cam_left";
    disparityMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    disparityMsg.image = disparityVisualization;
    disparityMapPublisher.publish(disparityMsg);

    // Publish the images and the HoloLens's current position.
    publishHololensPosition(translationLeft, stereoCamLeftPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationRight, stereoCamRightPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationCenter, hololensPositionPublisher, sequenceNumber, time);
    publishHololensCamToWorldTf(translationLeft, msg->camToWorldRotationLeft, "hololens_stereo_cam_left", hololensCamLeftPublisher, time);
    publishHololensCamToWorldTf(translationRight, msg->camToWorldRotationRight, "hololens_stereo_cam_right", hololensCamRightPublisher, time);
    stereoImageLeftPublisher.publish(imageMsgLeft);
    stereoImageRightPublisher.publish(imageMsgRight);
    sequenceNumber++;
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