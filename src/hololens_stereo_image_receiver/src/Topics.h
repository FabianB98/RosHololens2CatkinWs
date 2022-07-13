#pragma once

#define STEREO_IMAGE_TOPIC "/hololensStereoImage"
#define STEREO_PIXEL_DIRECTIONS_TOPIC "/hololensStereoCameraPixelDirections"

#define STEREO_IMAGE_LEFT_CAM_INFO_TOPIC "hololensStereoImage/left/camera_info"
#define STEREO_IMAGE_RIGHT_CAM_INFO_TOPIC "hololensStereoImage/right/camera_info"
#define STEREO_IMAGE_LEFT_RAW_TOPIC "hololensStereoImage/left/image_raw"
#define STEREO_IMAGE_RIGHT_RAW_TOPIC "hololensStereoImage/right/image_raw"
#define STEREO_IMAGE_LEFT_TOPIC "hololensStereoImage/left/image_filtered_rectified"
#define STEREO_IMAGE_RIGHT_TOPIC "hololensStereoImage/right/image_filtered_rectified"
#define DISPARITY_MAP_RAW_TOPIC "disparityMapRaw"
#define DISPARITY_MAP_TOPIC "disparityMap"

#define POINT_CLOUD_TOPIC "pointCloud"

#define STEREO_CAM_LEFT_POSITION_TOPIC "hololensStereoCamPositionLeft"
#define STEREO_CAM_RIGHT_POSITION_TOPIC "hololensStereoCamPositionRight"
#define HOLOLENS_POSITION_TOPIC "hololensPosition"