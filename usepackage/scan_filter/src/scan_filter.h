#ifndef _SCAN_FILTER_H_
#define _SCAN_FILTER_H_

#include <iostream>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

class Point
{
public:
  double x,y;
  double range;
  int id;
};

class Index
{
public:
  int x,y,pi,pe;
};

#endif
