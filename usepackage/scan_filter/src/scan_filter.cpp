#include "scan_filter.h"
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
ros::Publisher laser_pub;
ros::Publisher pcl_cloud_pub;
ros::Publisher cloud_filter_pub;
void lasercallback(sensor_msgs::LaserScan::ConstPtr point_ptr)
{
  bool first = true;
  //定义存取坐标点的变量
  std::vector<Point> point;
  std::vector<Index> index;
  sensor_msgs::PointCloud2 pcl_cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Point data;
  Index order;
  int inf_num = 0;
  double angle_increment = point_ptr->angle_increment;
  //检测range为inf的个数
  for(int i = 0;i < point_ptr->ranges.size();i++)
  {
    if(point_ptr->ranges[i] == INFINITY)
      inf_num++;
  }
  //初始化pcl类型点云的大小为原始数据点减去无效点 cloud:pcl下的点云类型
  //cloud.resize(point_ptr->ranges.size() - inf_num);
  cloud.resize(point_ptr->ranges.size());

  //对于无穷远的点,不加入到point中这保证了point的个数和点云的大小相同
  for(int i = 0; i < point_ptr->ranges.size(); i++)
  {
//    if(point_ptr->ranges[i] == INFINITY)
//    {
//      continue;
//    }
//    else
//    {
    data.x = (point_ptr->ranges[i])*sin(angle_increment*i);
    data.y = -(point_ptr->ranges[i])*cos(angle_increment*i);
    data.range = point_ptr->ranges[i];
    data.id = i;
//    }
    point.push_back(data);
  }

  pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud.begin();
  for(int i = 0; i < point.size(); i++,iter++)
  {
    iter->x = point[i].x;
    iter->y = point[i].y;
    iter->z = 1;
  }
  std::cout << "inf_num:   " << inf_num << std::endl;
  std::cout << "size of ranges:   " << point_ptr->ranges.size() << std::endl;
  std::cout << "size of point:      " << point.size() << std::endl;
  pcl::toROSMsg(cloud,pcl_cloud);
  pcl_cloud.header.frame_id = "laser";
  pcl_cloud_pub.publish(pcl_cloud);

  //对的
  double distance;
  bool start = false;
  int i_start,i_end,s_point,e_point;
  int i_num = 0;
  for(int i = 0; i < point.size(); i++)
  {
    if(first)
    {
      first = false;
      continue;
    }
    distance = sqrt(pow(point[i].x-point[i-1].x,2)+pow(point[i].y-point[i-1].y,2));
    if(distance < 0.1)
    {
      if(!start)
      {
        start = true;
        i_start = point[i].id - 1;
        s_point = i;
        i_num++;
      }
      else
        i_num++;
    }
    else
    {
      if(start)
      {
        start = false;
        i_end = point[i].id - 1;
        e_point = i;
        if(i_num > 5)
        {
          order.x = i_start;
          order.y = i_end;
          order.pi = s_point;
          order.pe = e_point;
          index.push_back(order);
        }
        i_start = 0;
        i_end = 0;
        i_num = 0;
        s_point = 0;
        e_point = 0;
      }
    }
    if(i == point.size() -1)
    {
      if(start)
      {
        start = false;
        i_end = point[i].id - 1;
        e_point = i;
        if(i_num > 5)
        {
          order.x = i_start;
          order.y = i_end;
          order.pi = s_point;
          order.pe = e_point;
          index.push_back(order);
        }
        i_start = 0;
        i_end = 0;
        i_num = 0;
        s_point = 0;
        e_point = 0;
      }
    }
  }

/*
  pcl::PointCloud<pcl::PointXYZ> cloud_filter;
  cloud_filter.resize(point_ptr->ranges.size() - inf_num);
  sensor_msgs::PointCloud2 cloud_filter_msg;
  int j = 0;
  for(int i = 0; i < index.size(); i++)
  {
    j = index[i].x;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud_filter.begin()+index[i].x;iter != cloud_filter.begin()+index[i].y;iter++)
    {
      iter->x = point[j].x;
      iter->y = point[j].y;
      iter->z = 0;
      //laser_scan_new.ranges[j] = point[j].range;
      j++;
    }
  }



  pcl::toROSMsg(cloud_filter,cloud_filter_msg);
  cloud_filter_msg.header.frame_id = "hokuyo_link";
  cloud_filter_pub.publish(cloud_filter_msg);
  */


  std::cout << "index.size:     " << index.size() << std::endl;
//  double length_min = 0;
//  int start_id = 0;
//  int end_id = 0;
//  int intens = 200;
//  int count = 0;
//  for(int i = 0; i  < index.size(); i++)
//  {
//    length = sqrt(pow((point[index[i].x].x)-(point[index[i].y].x),2)+pow((point[index[i].x].y)-(point[index[i].y].y),2));
//    if(length_min > length)
//    {
//      length_min = length;
//      start_id = index[i].x;
//      end_id = index[i].y;
//    }
//  }

  sensor_msgs::LaserScan laser_scan_new;
  laser_scan_new.header.frame_id = "laser";
  laser_scan_new.ranges.resize(point_ptr->ranges.size());
  laser_scan_new.intensities.resize(point_ptr->ranges.size());
  std::cout << "-----------------------------------" << std::endl;
//  std::cout << "old ranges.size:    " << point_ptr->ranges.size() << std::endl;
//  std::cout << "new ranges.size:    " << laser_scan_new.ranges.size() << std::endl;
  laser_scan_new.angle_increment = angle_increment;
  laser_scan_new.angle_max = point_ptr->angle_max;
  laser_scan_new.angle_min = point_ptr->angle_min;
  laser_scan_new.range_max = point_ptr->range_max;
  laser_scan_new.range_min = point_ptr->range_min;
  laser_scan_new.time_increment = point_ptr->time_increment;
  double length;
  int count = 1;
  int intens = 1;

  for(int i = 0;i < index.size(); i++)
  {
        //length = sqrt(pow((point[index[i].x].x)-(point[index[i].y].x),2)+pow((point[index[i].x].y)-(point[index[i].y].y),2));
        //std::cout << "length:       " << length << std::endl;
//        if(length < 0.4)
//        {
            for(int j = index[i].x; j <= index[i].y; j++)
            {
              laser_scan_new.ranges[j] = point_ptr->ranges[j];
              laser_scan_new.intensities[j] = 400;
              //std::cout << "new ranges :       " << laser_scan_new.ranges[j] << std::endl;
            }

            intens += 400;
            if(intens > 3000)
              intens = 1;

//        }
//        count++;
//        if(count%2 ==0)
//            intens = 2000;
//        else
//            intens = 1;
    }
  laser_pub.publish(laser_scan_new);
  point.clear();
  index.clear();

}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"scan_filter");
  ros::NodeHandle n;
  ros::Subscriber laser_sub = n.subscribe("/scan",100,lasercallback);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("/scan_filtered",1);
  cloud_filter_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_filter",1);
  pcl_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/pcl_cloud",1);
  ros::spin();
}
