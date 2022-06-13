#pragma once

#include "StereoImageReceiver.h"
#include "Topics.h"

StereoImageReceiver* stereoImageReceiver;

int main(int argc, char **argv);

void stereoImageCallback(const hololens_msgs::StereoCameraFrame::ConstPtr& msg);
void stereoPixelDirectionsCallback(const hololens_msgs::StereoPixelDirections::ConstPtr& msg);
