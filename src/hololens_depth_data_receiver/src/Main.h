#pragma once

#include "DepthDataReceiver.h"
#include "MainInitDepthDataReceiver.h"
#include "Topics.h"

#include <pcl_conversions/pcl_conversions.h>

DepthDataReceiver* depthDataReceiver;

int main(int argc, char **argv);

void shortThrowDepthFrameCallback(const hololens_msgs::DepthFrame::ConstPtr& msg);
void longThrowDepthFrameCallback(const hololens_msgs::DepthFrame::ConstPtr& msg);
void shortThrowPixelDirectionsCallback(const hololens_msgs::PixelDirections::ConstPtr& msg);
void longThrowPixelDirectionsCallback(const hololens_msgs::PixelDirections::ConstPtr& msg);
