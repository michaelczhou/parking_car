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
typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

typedef std::vector<PCLPointCloud> VectorPCLPointCloud;
typedef std::vector<Eigen::Matrix4f> VectorEigen3d;





//ros io
ros::Publisher pubMapCloud;
ros::Publisher pubLaserCloud;
ros::Publisher pubOdom;
tf::TransformBroadcaster *odom_broadcaster(NULL);


//system
//global
bool SystemInit = false;
bool newLaserFlag = false;
int OdomMethod = 0;//0默认，1 里程计，2重定位。

//一些参数
float scanRange = 30;//雷达的扫描半径
int subMapMaxFrame = 10;//submap中最多被添加的关键帧个数
PCLPointCloud PointMatched;


PCLPointCloud newScan;

pcl::NormalDistributionsTransform<PCLPoint, PCLPoint> ndt;

unsigned int countGlobal=0;

//global
VectorPCLPointCloud allSubMap;//记录每一次完成submap构建后的点云
VectorEigen3d allSubMapPose;//记录第一帧subMap作为全局坐标原点，之后每一次完成submap构建后的submap之间的全局位置。
unsigned int inSubMapId;


//local
VectorPCLPointCloud subMap;//局部地图由多个关键帧组合而成
VectorEigen3d subMapPose;//局部地图每一关键帧的位置，局部优化时第一帧可以fix住，位置是相对第一帧的。
PCLPointCloud subMapSum;
Eigen::Matrix4f TransformLast;//局部地图中位置
Eigen::Matrix4f TransformAll;//局部地图中位置
Eigen::Matrix4f TransformAllMap;//局部地图中位置

unsigned int historySubMapId;




void downsampleCloud(const PCLPointCloud::Ptr in_cloud_ptr, \
                     PCLPointCloud::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::VoxelGrid<PCLPoint> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}

//需要添加角度方向的策略
bool addMatchedMap(bool bForceUpdate=false)//强制更新标志,当强制更新时，会重置内部计数器。
{
  //use length
  static Eigen::Matrix4f TransformAll_before = Eigen::Isometry3f::Identity().matrix();
  static double yaw = 0;
  static int couter = 0;

  Eigen::Quaternionf ndt_Q(Eigen::Matrix3f(TransformAll.block(0,0,3,3)));
  double ndt_roll, ndt_pitch, ndt_yaw;
  tf::Quaternion ndt_orientation(ndt_Q.x(), ndt_Q.y(), ndt_Q.z(), ndt_Q.w());
  tf::Matrix3x3(ndt_orientation).getRPY(ndt_roll, ndt_pitch, ndt_yaw);//注意 这个rpy时绕固定轴！

  if(bForceUpdate)
  {
      couter = 0;
      TransformAll_before = TransformAll;
      yaw = ndt_yaw;
      return true;
  }
  if(couter > subMapMaxFrame)
      return false;
//因为这个是360度激光，所以不用yaw角的策略
  if(pow(TransformAll(0,3) - TransformAll_before(0,3), 2) +
     pow(TransformAll(1,3) - TransformAll_before(1,3), 2) +
     pow(TransformAll(2,3) - TransformAll_before(2,3), 2) > pow(1.4, 2)
     )
  {
    TransformAll_before = TransformAll;
    yaw = ndt_yaw;
    couter++;
    return true;
  }
  return false;
}

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

//检测是否在subMap的某个范围内，输入在submap下的rt,半径范围系数
bool checkIfInSubMapRange(Eigen::Matrix4f TransformLocal, float value=0.5)
{
    if(pow(TransformLocal(0,3), 2) + pow(TransformLocal(1,3), 2) + pow(TransformLocal(2,3), 2) < pow(scanRange*value,2))
        return true;
    else
        return false;
}

//检测当前位置是否处于其他已经建立的submap区域中，如果处于，则返回ture和submap的id号
bool checkIfInHistorySubMapRange(Eigen::Matrix4f TransformMap, float value, unsigned int &id)
{
    for(unsigned int i=0; i<allSubMap.size(); i++)
    {
        if(pow(TransformMap(0,3)-allSubMapPose[i](0,3), 2) +
           pow(TransformMap(1,3)-allSubMapPose[i](1,3), 2) +
           pow(TransformMap(2,3)-allSubMapPose[i](2,3), 2) <
                pow(scanRange*value,2)
                )
        {
           id = i;
           return true;
        }
    }

    return false;
}

void show_pose()
{
    static int counter = 0;
    counter++;
    if(counter<20)
        return;
    counter = 0;
    Eigen::Matrix3f r_TransformAllmatrix = TransformAll.block<3,3>(0,0);
    Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(r_(0), r_(1), r_(2));
    std::cout << "x y th " << TransformAll(0, 3) << " " << TransformAll(1, 3) << " " << r_(2) <<std::endl;
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

  TransformAll = Eigen::Affine3f::Identity().matrix();
  TransformLast = Eigen::Affine3f::Identity().matrix();

  ndt.setTransformationEpsilon (0.005);//设置迭代结束的条件
  ndt.setStepSize (0.1);//0.1改0.2没影响
  ndt.setResolution (0.2);//0.2在停车场只有10cm柱子的时候比较好，0.5会出现匹配问题
  ndt.setMaximumIterations (30);//30改成5 没影响,耗时不变，都是提前跳出的

  bool reflash_map = false;


  ros::Rate rate(100);
  bool status = ros::ok();



  while (status)
  {
    ros::spinOnce();
    if(!newLaserFlag)
        continue;

    newLaserFlag = false;
    if(!SystemInit)
    {
        subMap.push_back(newScan);//这里面存的scan是没有坐标变换
        subMapPose.push_back(Eigen::Affine3f::Identity().matrix());//这代表每一帧scan相对于submap坐标系原点的位置，注意这里的匹配是越后面加进来的帧越回合更多的帧匹配。
        OdomMethod = 1;
        reflash_map = addMatchedMap(true);

        allSubMapPose.push_back(Eigen::Affine3f::Identity().matrix());//注意全局地图的第一个pose先被初始化，但是地图还不存在。

        historySubMapId = 0;
        inSubMapId = 0;
        SystemInit = true;
        cout << "init" << endl;
        continue;
    }

    //新数据来了以后先和subMap匹配，获得全局位置。
    PCLPointCloud::Ptr newScan_ptr(new PCLPointCloud(newScan));
    PCLPointCloud::Ptr filtered_cloud (new PCLPointCloud);
    downsampleCloud(newScan_ptr,filtered_cloud, 0.2);
    ndt.setInputSource (filtered_cloud);

    if(reflash_map)
    {
        PointMatched.clear();
        for(unsigned int i=0; i<subMap.size(); i++)//这一步之后再优化，因为没必要每一帧scan都更新。
        {
            PCLPointCloud temp;
            pcl::transformPointCloud (subMap[i], temp, subMapPose[i]);
            PointMatched += temp;
        }
    }

    PCLPointCloud::Ptr PointMatched_ptr (new PCLPointCloud(PointMatched));//这个部分不耗时
    ndt.setInputTarget (PointMatched_ptr);//这部分耗费0.2s！内部有降采样的部分，ndt.setResolution

    // Calculating required rigid transform to align the input cloud to the target cloud.
    PCLPointCloud::Ptr output_cloud (new PCLPointCloud);
    ndt.align (*output_cloud, TransformAll*TransformLast);//必须加上预测，否则迭代会很慢，里程计的帧数跟不上激光发布的速度，直接导致建图效果很差

    TransformLast = TransformAll.inverse() * ndt.getFinalTransformation();
    TransformAll = ndt.getFinalTransformation();

    show_pose();
    //std::cout << TransformAll << std::endl;//在submap下全局坐标

    //先进行submap范围检测
    if(checkIfInSubMapRange(TransformAll, 0.5))
    {
        if(OdomMethod == 1)//里程计模式下需要更新关键帧
        {
            //保证当前帧在submap中，再进行subMap关键帧更新检测
            reflash_map = addMatchedMap(false);//检测是否更新关键帧，这个更新条件可以有很多变种体，后续需要测试
            if(reflash_map)
            {
                subMap.push_back(newScan);
                subMapPose.push_back(TransformAll);//相对于submap第一帧的位姿态。
                std::cout << "add a new key scan." << std::endl;
            }
        }
    }
    else//重头戏来了
    {
        if(OdomMethod == 1)
        {
            //当超过某个正在建立的submap范围后,表示之前的submap建立完毕
            //先对当前submap进行图优化调节每个关键scan的rt
            //void ceresGraphAdjust();

            //再进行所有关键scan求和，并且之后不再改变。注意如果不进行ceres调优时，这个部分的结果和PointMatched一样。
            subMapSum.clear();
            for(unsigned int i=0; i<subMap.size(); i++)
            {
                PCLPointCloud temp;
                pcl::transformPointCloud (subMap[i], temp, subMapPose[i]);
                subMapSum += temp;
            }
            allSubMap.push_back(subMapSum);
            std::cout << "add a new global submap." << std::endl;

            //清除所有局部地图信息，之后判断是新建submap还是保持重定位，等待建立新的submap
            subMap.clear();
            subMapPose.clear();
            subMapSum.clear();
            TransformAllMap = allSubMapPose[inSubMapId] * TransformAll;//计算当前的全局位置

            //之后再检测当前位置获得的scan是否进入到了历史中存在的submap0.5倍范围内
            if(checkIfInHistorySubMapRange(TransformAllMap, 0.5, historySubMapId))
            {
                //进入一个已有的submap，切换到当前submap中
                subMap.push_back(allSubMap[historySubMapId]);
                subMapPose.push_back(Eigen::Affine3f::Identity().matrix());

                inSubMapId = historySubMapId;
                OdomMethod = 2;//之后只需要重定位
                TransformAll = allSubMapPose[inSubMapId].inverse() * TransformAllMap;//计算出在当前地图中的位置
                TransformLast = Eigen::Affine3f::Identity().matrix(); //其实可以基于旋转进行变换，但是给0方便一些。
                reflash_map = addMatchedMap(true);
                std::cout << "last method is 1. in new map use method 2." << std::endl;

            }
            else
            {
                //创建新的submap
                subMap.push_back(newScan);
                subMapPose.push_back(Eigen::Affine3f::Identity().matrix());

                OdomMethod = 1;
                TransformAll = Eigen::Affine3f::Identity().matrix();
                TransformLast = Eigen::Affine3f::Identity().matrix(); //其实可以基于旋转进行变换，但是给0方便一些。
                allSubMapPose.push_back(TransformAllMap);//非常重要！当前的scan是下一个submap的初始位置，是下一个submap的初始帧。
                inSubMapId = allSubMapPose.size()-1;
                reflash_map = addMatchedMap(true);
                std::cout << "last method is 1. in new map use method 1." << std::endl;
            }
        }
        else//当之前是重定位模式
        {
            subMap.clear();
            subMapPose.clear();
            subMapSum.clear();
            TransformAllMap = allSubMapPose[inSubMapId] * TransformAll;//计算当前的全局位置

            if(checkIfInHistorySubMapRange(TransformAllMap, 0.5, historySubMapId))
            {
                subMap.push_back(allSubMap[historySubMapId]);
                subMapPose.push_back(Eigen::Affine3f::Identity().matrix());
                inSubMapId = historySubMapId;
                //OdomMethod = 2;//之后还是重定位
                TransformAll = allSubMapPose[inSubMapId].inverse() * TransformAllMap;//计算出在当前地图中的位置
                TransformLast = Eigen::Affine3f::Identity().matrix(); //其实可以基于旋转进行变换，但是给0方便一些。
                reflash_map = addMatchedMap(true);
                std::cout << "last method is 2. in new map use method 2." << std::endl;
            }
            else
            {
                //创建新的submap
                subMap.push_back(newScan);
                subMapPose.push_back(Eigen::Affine3f::Identity().matrix());

                OdomMethod = 1;
                TransformAll = Eigen::Affine3f::Identity().matrix();
                TransformLast = Eigen::Affine3f::Identity().matrix(); //其实可以基于旋转进行变换，但是给0方便一些。
                allSubMapPose.push_back(TransformAllMap);//非常重要！当前的scan是下一个submap的初始位置，是下一个submap的初始帧。
                inSubMapId = allSubMapPose.size()-1;
                reflash_map = addMatchedMap(true);
                std::cout << "last method is 2. in new map use method 2." << std::endl;
            }
        }
    }


{
    Eigen::Matrix4f TransformAllMap;//全局地图中位置
    TransformAllMap = allSubMapPose[inSubMapId] * TransformAll;
    PCLPointCloud temp;
    pcl::transformPointCloud (newScan, temp, TransformAllMap);

    sensor_msgs::PointCloud2 PonitMap2;
    pcl::toROSMsg(temp, PonitMap2);

    PonitMap2.header.stamp = ros::Time::now();
    PonitMap2.header.frame_id = "/odom";
    pubLaserCloud.publish(PonitMap2);
}

    {
        PCLPointCloud temp1, temp2;
        for(unsigned int i=0 ;i<allSubMap.size(); i++)
        {
            pcl::transformPointCloud (allSubMap[i], temp1, allSubMapPose[i]);
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
        Eigen::Matrix4f TransformAllMap;//全局地图中位置
        TransformAllMap = allSubMapPose[inSubMapId] * TransformAll;
        Eigen::Matrix3f r_TransformAllmatrix = TransformAllMap.block<3,3>(0,0);
        Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(r_(0), r_(1), r_(2));
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = TransformAllMap(0,3);
        odom_trans.transform.translation.y = TransformAllMap(1,3);
        odom_trans.transform.translation.z = TransformAllMap(2,3);
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster->sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = TransformAllMap(0,3);
        odom.pose.pose.position.y = TransformAllMap(1,3);
        odom.pose.pose.position.z = TransformAllMap(2,3);
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
  return 0;
}

