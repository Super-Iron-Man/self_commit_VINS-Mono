#pragma once
    /*
    效果等同于
    #ifndef _XX_头文件.H
    #define _XX_头文件.H
    <code>
    */
#endif
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;//常量定义


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
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
/*
其实头文件对计算机而言没什么作用，她只是在预编译时在#include的地方展开一下，没别的意义了.

变量的定义：为变量分配存储空间，同时指明变量的类型和名字。另外变量的初始化，就是在变量的定义出给出值。
变量的声明：它主要向程序声明变量的类型和名字。

头文件主要作用有两个：
    一是把很多其他文件需要重复使用的函数变量等在此声明，在需要的地方include；切记是“声明”而非“定义”，否则会造成多次引用头文件带来的变量重复定义
    二是给使用你函数文件的人看的，便于快速了解源代码中的主要成分都有哪些

一般头文件里放：　全局变量声明、常量声明、函数声明、类的声明(包括类里面的成员变量和成员函数)

总结如下：
    如果是要定义全局变量，那么在头文件中用extern关键字声明，然后在另一个.cpp文件中定义；
    如果是要声明一个不想被其他文件使用、只能被本文件使用的变量，可以用static关键字在头文件中进行定义；
    如果所要定义的变量为局部变量，并且其值在编译时就已经可以确定，就可以用const关键词在头文件中进行定义。
*/


