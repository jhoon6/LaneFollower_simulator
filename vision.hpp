#ifndef __VISION_HPP__
#define __VISION_HPP__

#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include "dxl.hpp"

using namespace std;
using namespace cv;

int calc_err(Mat gray_img, int prev_error, bool isLeft);
Mat preprocess(Mat input);

#endif