#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

//extern 是 C++ 中的一个关键字，用于声明一个变量或者函数的外部链接。它告诉编译器该变量或函数的定义位于其他源文件中，而不是当前源文件中。
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
