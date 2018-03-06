#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <pcl/registration/ndt.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <vector>

//数据类型不影响ndt
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

#define MATCH_QUEUE_NUMBER 10
#define MATCH_QUEUE_NUMBER_STEP 1//间隔几帧取一帧

bool SystemInit = false;
bool newLaserFlag = false;

ros::Publisher pubMapCloud;
ros::Publisher pubLaserCloud;
ros::Publisher pubOdom;
tf::TransformBroadcaster *odom_broadcaster(NULL);

VPointCloud PointCurrent;
VPointCloud PointLast;
VPointCloud PointMap;
VPointCloud PointMatch[MATCH_QUEUE_NUMBER];
VPointCloud PointMatched;
int PointMatch_count = -1;
unsigned int countGlobal=0;

//定义一个ndt对象
pcl::NormalDistributionsTransform<VPoint, VPoint> ndt;

Eigen::Matrix4f TransformLast;
Eigen::Matrix4f TransformAll;

void downsampleCloud(const VPointCloud::Ptr in_cloud_ptr, \
                     VPointCloud::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::VoxelGrid<VPoint> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}

bool addMatchedMap()
{
  //use length
  static Eigen::Matrix4f TransformAll_before = Eigen::Isometry3f::Identity().matrix();
  static double yaw=0;

  Eigen::Quaternionf ndt_Q(Eigen::Matrix3f(TransformAll.block(0,0,3,3)));
  double ndt_roll, ndt_pitch, ndt_yaw;
  tf::Quaternion ndt_orientation(ndt_Q.x(), ndt_Q.y(), ndt_Q.z(), ndt_Q.w());
  tf::Matrix3x3(ndt_orientation).getRPY(ndt_roll, ndt_pitch, ndt_yaw);//注意 这个rpy时绕固定轴！


  if(pow(TransformAll(0,3) - TransformAll_before(0,3), 2) +
     pow(TransformAll(1,3) - TransformAll_before(1,3), 2) +
     pow(TransformAll(2,3) - TransformAll_before(2,3), 2) > pow(1,2)
    )
  {
    TransformAll_before = TransformAll;
    yaw = ndt_yaw;
    PointMatch_count = (PointMatch_count+1)%MATCH_QUEUE_NUMBER;
    PointMatch[PointMatch_count] = PointCurrent;
    PointMap += PointCurrent;//for show
    return true;
  }
  return false;
}

void lasercallback(sensor_msgs::LaserScan::ConstPtr point_ptr)
{
  PointLast.clear();
  PointLast = PointCurrent;
  PointCurrent.clear();

  double angle_increment = point_ptr->angle_increment;
  int inf_num = 0;
  for(int i = 0;i < point_ptr->ranges.size(); i++)
  {
    if(point_ptr->ranges[i] == INFINITY || point_ptr->ranges[i]>30)
      inf_num++;
  }
  PointCurrent.resize(point_ptr->ranges.size() - inf_num);

  unsigned int cnt=0;
  //将单线激光数据转化为点云
  for(int i = 0; i < point_ptr->ranges.size(); i++)
  {
    if(point_ptr->ranges[i] == INFINITY || point_ptr->ranges[i]>30)
    {
      continue;
    }
    else
    {
      PointCurrent.at(cnt).x = (point_ptr->ranges[i])*sin(angle_increment*i);;
      PointCurrent.at(cnt).y = -(point_ptr->ranges[i])*cos(angle_increment*i);
      PointCurrent.at(cnt).z = 0;
      cnt++;
    }
  }
//  std::stringstream ss_pcd;
//  ss_pcd << countGlobal;
//  std::string s_pcd = "/home/xjh/catkin_ndt/src/perception_oru-port-kinetic/ndt_mcl/data/pcl_mapping/data" + ss_pcd.str() + ".pcd";
//  pcl::io::savePCDFileBinary(s_pcd, PointCurrent);
//  std::cout << countGlobal <<std::endl;
  newLaserFlag = true;
  countGlobal++;

}

int main(int argc, char** argv)
{
  using namespace std;

  ros::init(argc,argv,"My_ndt_mapping");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan",200,lasercallback);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_laser",1);
  pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/map",1);
  pubOdom = nh.advertise<nav_msgs::Odometry>("/odom",1);

  tf::TransformBroadcaster odom_broadcaster_;//其实这个不能设为全局变量，而pub可以
  odom_broadcaster = &odom_broadcaster_;

  Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitX());
  Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
  TransformAll = (init_translation * init_rotation).matrix ();
  TransformLast = TransformAll;

  ndt.setTransformationEpsilon (0.005);//设置迭代结束的条件
  ndt.setStepSize (0.1);//0.1改0.2没影响
  ndt.setResolution (0.5);//0.2在停车场只有10cm柱子的时候比较好，0.5会出现匹配问题
  ndt.setMaximumIterations (30);//30改成5 没影响,耗时不变，都是提前跳出的


  ros::Rate rate(100);
  bool status = ros::ok();

  bool reflash_map = true;
  cout << "init" << endl;

  while (status)
  {
    ros::spinOnce();

    if(newLaserFlag)
    {
      newLaserFlag = false;

      ros::Time Time;
      Time.init();

      if(PointCurrent.points.size()!=0 && PointLast.points.size()!=0)
      {
        if(!SystemInit)
        {
          PointMatch_count = (PointMatch_count+1)%MATCH_QUEUE_NUMBER;
          PointMatch[PointMatch_count] = PointCurrent;
          PointMap += PointCurrent;
          SystemInit = true;
          continue;
        }

        VPointCloud::Ptr PointLast_ptr(new VPointCloud(PointLast));
        VPointCloud::Ptr PointCurrent_ptr(new VPointCloud(PointCurrent));

        ros::Time time_start = Time.now();
        VPointCloud::Ptr filtered_cloud (new VPointCloud);
        downsampleCloud(PointCurrent_ptr,filtered_cloud, 0.5);//不使用approximate_voxel_filter,因为approximate对于场景中有大的动态物体会出问题，比如校车经过
        ndt.setInputSource (filtered_cloud);


        //基于距离的更新策略，暂时没有必要增加旋转，因为激光是360度的，实际使用时roll和pitch 不会有太大的变化，因此没必要。
        if(reflash_map)
        {
          PointMatched.clear();
          for(int i=0; i<MATCH_QUEUE_NUMBER; i+=MATCH_QUEUE_NUMBER_STEP)
          {
            if(i >= MATCH_QUEUE_NUMBER)
              break;
            if(PointMatch[i].points.size() == 0)
            {
              break;
            }
            PointMatched += PointMatch[i];
          }
          VPointCloud::Ptr PointMatched_ptr (new VPointCloud(PointMatched));//这个部分不耗时
          ndt.setInputTarget (PointMatched_ptr);//这部分耗费0.2s！内部有降采样的部分，ndt.setResolution
          //这一步可以进行优化，因为被匹配的地图可以是一个双缓冲，在其他线程进行地图的ndt初始化。
        }

        time_start = Time.now();
        // Calculating required rigid transform to align the input cloud to the target cloud.
        VPointCloud::Ptr output_cloud (new VPointCloud);

        ndt.align (*output_cloud, TransformAll*TransformLast);//必须加上预测，否则迭代会很慢，里程计的帧数跟不上激光发布的速度，直接导致建图效果很差

        ros::Time time_temp = Time.now();
        std::cout << "4 " << (time_temp - time_start).toSec() << std::endl;//耗时0.16-0.59s 0.2的分辨率


        time_start = Time.now();
        std::cout << "converged:" << ndt.hasConverged ()
                  << " score: " << ndt.getFitnessScore () << std::endl;

        //TransformLast = ndt.getFinalTransformation ();//刷新当前帧和上一帧的位姿关系P_last = T_(last,current) * P_current
        //TransformAll = TransformAll * TransformLast ;//更新全局位姿 P_map = T_(map,current) * P_current

          TransformLast = TransformAll.inverse() * ndt.getFinalTransformation();
          TransformAll = ndt.getFinalTransformation();


        std::cout << TransformAll << std::endl;
        pcl::transformPointCloud (PointCurrent, PointCurrent, TransformAll);//将当前帧变换到map坐标系下

        reflash_map = addMatchedMap();

        sensor_msgs::PointCloud2 PonitMap2;
        pcl::toROSMsg(PointCurrent, PonitMap2);

        PonitMap2.header.stamp = ros::Time::now();
        PonitMap2.header.frame_id = "/odom";
        pubLaserCloud.publish(PonitMap2);


        sensor_msgs::PointCloud2 PonitMap3;
        pcl::toROSMsg(PointMap, PonitMap3);

        PonitMap3.header.stamp = ros::Time::now();
        PonitMap3.header.frame_id = "/odom";
        pubMapCloud.publish(PonitMap3);

        ros::Time current_time = ros::Time::now();
        //这里有问题，因为.eulerAngles时相对轴，而createQuaternionMsgFromRollPitchYaw时绝对轴。
        Eigen::Matrix3f r_TransformAllmatrix = TransformAll.block<3,3>(0,0);
        Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(r_(0), r_(1), r_(2));
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = TransformAll(0,3);
        odom_trans.transform.translation.y = TransformAll(1,3);
        odom_trans.transform.translation.z = TransformAll(2,3);
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster->sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = TransformAll(0,3);
        odom.pose.pose.position.y = TransformAll(1,3);
        odom.pose.pose.position.z = TransformAll(2,3);
        odom.pose.pose.orientation = odom_quat;

        Eigen::Matrix3f r_matrix = TransformLast.block<3,3>(0,0);
        Eigen::Vector3f r_delta = r_matrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = TransformLast(0,3);
        odom.twist.twist.linear.y = TransformLast(1,3);
        odom.twist.twist.linear.z = TransformLast(2,3);
        odom.twist.twist.angular.x = r_delta(0);
        odom.twist.twist.angular.y = r_delta(1);
        odom.twist.twist.angular.z = r_delta(2);

        //publish the message
        pubOdom.publish(odom);

      }
    }

  }
  return 0;
}

