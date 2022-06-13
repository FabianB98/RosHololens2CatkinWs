#include "StereoImageReceiver.h"

StereoImageReceiver::StereoImageReceiver(ros::NodeHandle n)
{
    ROS_INFO("Creating StereoImageReceiver...");

    stereoImageLeftPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_LEFT_TOPIC, 10);
    stereoImageRightPublisher = n.advertise<sensor_msgs::Image>(STEREO_IMAGE_RIGHT_TOPIC, 10);
    stereoCamLeftPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_LEFT_POSITION_TOPIC, 10);
    stereoCamRightPositionPublisher = n.advertise<geometry_msgs::PointStamped>(STEREO_CAM_RIGHT_POSITION_TOPIC, 10);
    hololensPositionPublisher = n.advertise<geometry_msgs::PointStamped>(HOLOLENS_POSITION_TOPIC, 10);

    stereoPixelDirections = hololens_msgs::StereoPixelDirections::Ptr(new hololens_msgs::StereoPixelDirections());
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

    // Publish the images and the HoloLens's current position.
    ros::Time time = ros::Time::now();
    publishHololensPosition(translationLeft, stereoCamLeftPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationRight, stereoCamRightPositionPublisher, sequenceNumber, time);
    publishHololensPosition(translationCenter, hololensPositionPublisher, sequenceNumber, time);
    publishHololensCamToWorldTf(translationLeft, msg->camToWorldRotationLeft, "hololens_stereo_cam_left", hololensCamLeftPublisher, time);
    publishHololensCamToWorldTf(translationRight, msg->camToWorldRotationRight, "hololens_stereo_cam_right", hololensCamRightPublisher, time);
    publishImage(imageLeft, stereoImageLeftPublisher, "hololens_stereo_cam_left", sequenceNumber, time);
    publishImage(imageRight, stereoImageRightPublisher, "hololens_stereo_cam_right", sequenceNumber, time);
    sequenceNumber++;
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