#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <vector>
#include <parking_car/map_builder.hpp>

MapBuilder mapBuilder;

bool newLaserFlag = false;
unsigned int countGlobal = 0;

Eigen::Matrix4f globalPose = Eigen::Isometry3f::Identity().matrix();
Eigen::Matrix4f globalPoseDelta = Eigen::Isometry3f::Identity().matrix();
//ros io
ros::Publisher pubMapCloud;
ros::Publisher pubLaserCloud;
ros::Publisher pubOdom;
tf::TransformBroadcaster *odom_broadcaster(NULL);
VPointCloud newScan;

void lasercallback(sensor_msgs::LaserScan::ConstPtr point_ptr)
{
  float range_limit = 30;
  newScan.clear();

  double angle_increment = point_ptr->angle_increment;
  int inf_num = 0;
  for(unsigned int i = 0; i < point_ptr->ranges.size(); i++)
  {
    if(point_ptr->ranges[i] == INFINITY || point_ptr->ranges[i]>range_limit)
      inf_num++;
  }
  newScan.resize(point_ptr->ranges.size() - inf_num);

  unsigned int cnt=0;
  //将单线激光数据转化为点云
  for(unsigned int i = 0; i < point_ptr->ranges.size(); i++)
  {
    if(point_ptr->ranges[i] == INFINITY || point_ptr->ranges[i]>range_limit)
    {
      continue;
    }
    else
    {
      newScan.at(cnt).x = (point_ptr->ranges[i])*sin(angle_increment*i);;
      newScan.at(cnt).y = -(point_ptr->ranges[i])*cos(angle_increment*i);
      newScan.at(cnt).z = 0;
      cnt++;
    }
  }
//  std::stringstream ss_pcd;
//  ss_pcd << countGlobal;
//  std::string s_pcd = "/home/xjh/catkin_ndt/src/perception_oru-port-kinetic/ndt_mcl/data/pcl_mapping/data" + ss_pcd.str() + ".pcd";
//  pcl::io::savePCDFileBinary(s_pcd, newScan);
//  std::cout << countGlobal <<std::endl;
  newLaserFlag = true;
  countGlobal++;

}

int main(int argc, char** argv)
{
  using namespace std;

  ros::init(argc,argv,"My_ndt_mapping");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan_filtered",200,lasercallback);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_laser",1);
  pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/map",1);
  pubOdom = nh.advertise<nav_msgs::Odometry>("/odom",1);

  tf::TransformBroadcaster odom_broadcaster_;//其实这个不能设为全局变量，而pub可以
  odom_broadcaster = &odom_broadcaster_;

  ros::Rate rate(100);

  while (ros::ok())//当按下ctrl+c 后，这个变量就会变成false
  {
    ros::spinOnce();
    if(!newLaserFlag)
        continue;

    newLaserFlag = false;

    Eigen::Matrix4f globalPoseTemp = globalPose;
    mapBuilder.addScan(newScan, globalPose);
    globalPoseDelta = globalPoseTemp.inverse() * globalPose;

{
    VPointCloud temp;
    pcl::transformPointCloud (newScan, temp, globalPose);

    sensor_msgs::PointCloud2 PonitMap2;
    pcl::toROSMsg(temp, PonitMap2);

    PonitMap2.header.stamp = ros::Time::now();
    PonitMap2.header.frame_id = "/odom";
    pubLaserCloud.publish(PonitMap2);
}

    {
        VPointCloud temp1, temp2;
        for(unsigned int i=0 ;i<mapBuilder.subMap.size(); i++)
        {
            pcl::transformPointCloud (mapBuilder.subMap[i].sumOfAllKeyScan, temp1, mapBuilder.subMap[i].subMapGlobalPose);
            temp2 += temp1;
        }
        if(temp2.size() != 0)
        {
            sensor_msgs::PointCloud2 PonitMap3;
            pcl::toROSMsg(temp2, PonitMap3);

            PonitMap3.header.stamp = ros::Time::now();
            PonitMap3.header.frame_id = "/odom";
            pubMapCloud.publish(PonitMap3);
        }
    }

    {
        ros::Time current_time = ros::Time::now();
        //这里有问题，因为.eulerAngles时相对轴，而createQuaternionMsgFromRollPitchYaw时绝对轴。
        Eigen::Matrix3f r_TransformAllmatrix = globalPose.block<3,3>(0,0);
        Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(r_(0), r_(1), r_(2));
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = globalPose(0,3);
        odom_trans.transform.translation.y = globalPose(1,3);
        odom_trans.transform.translation.z = globalPose(2,3);
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster->sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = globalPose(0,3);
        odom.pose.pose.position.y = globalPose(1,3);
        odom.pose.pose.position.z = globalPose(2,3);
        odom.pose.pose.orientation = odom_quat;

        Eigen::Matrix3f r_matrix = globalPoseDelta.block<3,3>(0,0);
        Eigen::Vector3f r_delta = r_matrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = globalPoseDelta(0,3);
        odom.twist.twist.linear.y = globalPoseDelta(1,3);
        odom.twist.twist.linear.z = globalPoseDelta(2,3);
        odom.twist.twist.angular.x = r_delta(0);
        odom.twist.twist.angular.y = r_delta(1);
        odom.twist.twist.angular.z = r_delta(2);

        //publish the message
        pubOdom.publish(odom);
    }
  }
  return 0;
}

