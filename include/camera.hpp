#pragma once

#include <libcaercpp/devices/davis.hpp>

#include <atomic>
#include <csignal>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <math.h>
#include <algorithm>
#include <map>

class camera
{
public:
    camera();
    ~camera();

    cv::Mat image;

    void globalShutdownSignalHandler(int signal);
    void usbShutdownHandler(void *ptr);
    void function();
};