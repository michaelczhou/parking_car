#ifndef _MAP_BUILDER_HPP_
#define _MAP_BUILDER_HPP_

#include "ceres_pose2d.hpp"
#include "3d_party_libraries.h"
#include <vector>

class MapEdge
{
public:
    MapEdge(int _idFirst, int _idSecond, Eigen::Matrix4f _transformT)
    {
        idFirst = _idFirst;
        idSecond = _idSecond;
        transformT = _transformT;
    }

    int idFirst, idSecond;
    Eigen::Matrix4f transformT;
};

//SubMap 就是MapVertex
class SubMap
{
public:
    SubMap(int _id, VPointCloud _initKeyScan,  Eigen::Matrix4f _subMapGlobalPose, double _keyScanPoseInBuildingrRange = 15.0)
    {
        id = _id;
        keyScanPoseInBuildingrRange = _keyScanPoseInBuildingrRange;
        completeFlag = false;

        keyScan.push_back(_initKeyScan);
        keyScanAfterTransform.push_back(_initKeyScan);
        keyScanLocalPose.push_back(Eigen::Isometry3f::Identity().matrix());

        subMapGlobalPose = _subMapGlobalPose;

        newScanDeltaLocalPose = Eigen::Isometry3f::Identity().matrix();
        newScanLocalPose = Eigen::Isometry3f::Identity().matrix();
        newScanGlobalPose = _subMapGlobalPose;

        updateSumOfAllKeyScan();//更新被匹配的关键帧
        addMatchedMap(true);

        ndt.setTransformationEpsilon (0.01);//设置迭代结束的条件
        ndt.setStepSize (0.1);//0.1改0.2没影响
        ndt.setResolution (0.5);//0.2在停车场只有10cm柱子的时候比较好，0.5会出现匹配问题
        ndt.setMaximumIterations (30);//30改成5 没影响,耗时不变，都是提前跳出的
        ndt.setOulierRatio(0.45);

        VPointCloud::Ptr PointMatched_ptr (new VPointCloud(sumOfAllKeyScan));//这个部分不耗时
        ndt.setInputTarget (PointMatched_ptr);//这部分耗费，内部有降采样的部分，ndt.setResolution
    }

    //输入采集到的_newScan输出当前全局位置
    Eigen::Matrix4f getPose(VPointCloud _newScan)
    {
        newScan = _newScan;//备份
        VPointCloud::Ptr newScan_ptr(new VPointCloud(_newScan));
        VPointCloud::Ptr filtered_cloud (new VPointCloud);
        downsampleCloud(newScan_ptr, filtered_cloud, 0.5);
        ndt.setInputSource (filtered_cloud);

        // Calculating required rigid transform to align the input cloud to the target cloud.
        VPointCloud::Ptr output_cloud (new VPointCloud);
        ndt.align (*output_cloud, newScanLocalPose * newScanDeltaLocalPose);//必须加上预测，否则迭代会很慢，里程计的帧数跟不上激光发布的速度，直接导致建图效果很差

        newScanDeltaLocalPose = newScanLocalPose.inverse() * ndt.getFinalTransformation();
        newScanLocalPose = ndt.getFinalTransformation();
        newScanGlobalPose = subMapGlobalPose * newScanLocalPose;

        showLocalPose();
        return newScanGlobalPose;
    }

    //当调用过getPose,并确认在submap范围内可以调用这个，判断刚才的数据是否需要被添加关键帧.若要添加，则要更新sum等
    bool reflash()
    {
        if(addMatchedMap(false))
        {
            VPointCloud temp;
            pcl::transformPointCloud (newScan, temp, newScanLocalPose);//在需要被添加时再计算这个变换

            keyScan.push_back(newScan);
            keyScanAfterTransform.push_back(temp);
            keyScanLocalPose.push_back(newScanLocalPose);
            updateSumOfAllKeyScan();

            VPointCloud::Ptr PointMatched_ptr (new VPointCloud(sumOfAllKeyScan));//这个部分不耗时
            //VPointCloud::Ptr PointMatched_ptr (new VPointCloud(keyScanAfterTransform.back()));//这个部分不耗时
            ndt.setInputTarget (PointMatched_ptr);//这部分耗费，内部有降采样的部分，ndt.setResolution
            return true;
        }
        return false;
    }

    bool checkIfInSubMapRange()
    {
        if(pow(newScanLocalPose(0,3), 2) + pow(newScanLocalPose(1,3), 2) + pow(newScanLocalPose(2,3), 2) < pow(keyScanPoseInBuildingrRange,2))
            return true;
        else
            return false;
    }

    void setLocalPose(Eigen::Matrix4f _newScanLocalPose)
    {
        newScanLocalPose = _newScanLocalPose;
        newScanDeltaLocalPose = Eigen::Isometry3f::Identity().matrix();
        newScanGlobalPose = subMapGlobalPose * _newScanLocalPose;
    }

    //结束构建，需要进行节点优化，并且重新返回当前scan基于优化后的map的全局pose
    Eigen::Matrix4f finish()
    {
        //这里要局部优化

        completeFlag = true;
        return newScanGlobalPose;
    }

    int id;//感觉这个变量没意义
    double keyScanPoseInBuildingrRange;
    bool completeFlag;
    std::vector<VPointCloud> keyScan;
    std::vector<VPointCloud> keyScanAfterTransform;
    std::vector<Eigen::Matrix4f> keyScanLocalPose;

    Eigen::Matrix4f newScanDeltaLocalPose;
    Eigen::Matrix4f newScanLocalPose;
    Eigen::Matrix4f newScanGlobalPose;

    VPointCloud newScan;
    VPointCloud sumOfAllKeyScan;

    Eigen::Matrix4f subMapGlobalPose;

    pcl::NormalDistributionsTransform<VPoint, VPoint> ndt;


private:
    inline void updateSumOfAllKeyScan()
    {
       sumOfAllKeyScan +=  keyScanAfterTransform.back();
    }

    double limitYaw(double value)
    {
      if(value>3.1415926)
        value -= 3.1415926;
      if(value < -3.1415926)
        value += 3.1415926;
      return value;
    }
    //需要添加角度方向的策略
    //这里也可以添加概率地图，以及射线跟踪。
    bool addMatchedMap(bool bForceUpdate=false)//强制更新标志,当强制更新时，会重置内部计数器。
    {
      //use length
      static Eigen::Matrix4f transformBefore = Eigen::Isometry3f::Identity().matrix();
      static double yaw = 0;
      static int couter = 0;

      Eigen::Quaternionf ndt_Q(Eigen::Matrix3f(newScanLocalPose.block(0,0,3,3)));
      double ndt_roll, ndt_pitch, ndt_yaw;
      tf::Quaternion ndt_orientation(ndt_Q.x(), ndt_Q.y(), ndt_Q.z(), ndt_Q.w());
      tf::Matrix3x3(ndt_orientation).getRPY(ndt_roll, ndt_pitch, ndt_yaw);//注意 这个rpy时绕固定轴！

      if(bForceUpdate)
      {
          couter = 0;
          transformBefore = newScanLocalPose;
          yaw = ndt_yaw;
          return true;
      }
      if(couter >= 20)//最多10个关键帧，这个后期要修改
          return false;
    //因为这个是360度激光，所以不用yaw角的策略
      if(pow(newScanLocalPose(0,3) - transformBefore(0,3), 2) +
         pow(newScanLocalPose(1,3) - transformBefore(1,3), 2) +
         pow(newScanLocalPose(2,3) - transformBefore(2,3), 2) > pow(keyScanPoseInBuildingrRange/10, 2)
         || fabs(limitYaw(yaw-ndt_yaw))>3.1415926/6
         )
      {
        transformBefore = newScanLocalPose;
        yaw = ndt_yaw;
        couter++;
        return true;
      }
      return false;
    }



    void showLocalPose()
    {
        static int counter = 0;
        counter++;
        if(counter<20)
            return;
        counter = 0;
        Eigen::Matrix3f r_TransformAllmatrix = newScanLocalPose.block<3,3>(0,0);
        Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序

        Eigen::Matrix3f r_TransformAllmatrixGlobal = newScanGlobalPose.block<3,3>(0,0);
        Eigen::Vector3f r_Global = r_TransformAllmatrixGlobal.eulerAngles(0,1,2);//roll pitch yaw 顺序


        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Local/Global Pose (x,y,th): " << "(" <<\
                     newScanLocalPose(0, 3) << " " << newScanLocalPose(1, 3) << " "<< r_(2) << ") (" <<\
                     newScanGlobalPose(0, 3) << " " << newScanGlobalPose(1, 3) << " "<< r_Global(2) << ")" << std::endl;
        std::cout.unsetf( std::ios::fixed );
    }
};

class MapBuilder
{
public:
    MapBuilder()
    {
        inWhichSubMapIdFront = 0;
        inWhichSubMapId = 0;
        systemInitFlag = false;
        locationWay = 1;
        newScanGlobalPose = Eigen::Isometry3f::Identity().matrix();
        ndt.setTransformationEpsilon (0.01);//设置迭代结束的条件
        ndt.setStepSize (0.1);//0.1改0.2没影响
        ndt.setResolution (0.5);//0.2在停车场只有10cm柱子的时候比较好，0.5会出现匹配问题
        ndt.setMaximumIterations (30);//30改成5 没影响,耗时不变，都是提前跳出的
        ndt.setOulierRatio(0.45);
    }

    //V0.1
    void addScan(VPointCloud _newScan, Eigen::Matrix4f &_globalPosition)//输入一帧当前数据，输出机器人全局位置。
    {
        if(!systemInitFlag)
        {
            PoseInOtherSubMapRange = 2.0;
            //创建新的submap
            int id = subMap.size();
            subMap.push_back(SubMap(id, _newScan, Eigen::Isometry3f::Identity().matrix(), PoseInOtherSubMapRange));
            inWhichSubMapIdFront = 0;
            inWhichSubMapId = 0;
            _globalPosition = newScanGlobalPose;
            systemInitFlag = true;
            std::cout << "init a new submap. id = " << inWhichSubMapId << std::endl;
            return;
        }

        newScanGlobalPose = subMap[inWhichSubMapId].getPose(_newScan);//输出当前位置，--无论是重定位还是里程计
        _globalPosition = newScanGlobalPose;

        if(subMap[inWhichSubMapId].checkIfInSubMapRange())
        {
            if(locationWay == 1)
            {
                if(subMap[inWhichSubMapId].reflash())
                {
                    std::cout << "add a keyScan" << std::endl;
                }
            }
        }
        else
        {
            if(locationWay == 1)
            {
                //当超过某个正在建立的submap范围后,表示之前的submap建立完毕
                //先对当前submap进行图优化调节每个关键scan的rt,再进行所有关键scan求和，并且之后不再改变。
                newScanGlobalPose = subMap[inWhichSubMapId].finish();//还没实现submap内部闭环
                //_globalPosition = newScanGlobalPose;
                std::cout << "complete a new submap. id = " << inWhichSubMapId << std::endl;

                //之后基于当前建立的submap的总地图和上一个submap总地图进行匹配，修正当前submap的初始全局pose，
                //因为当前submap初始全局pose是由第一帧scan和上一个submap匹配获得的，数据量太少。
                VPointCloud::Ptr PointMatched_ptr (new VPointCloud(subMap[inWhichSubMapIdFront].sumOfAllKeyScan));
                ndt.setInputTarget (PointMatched_ptr);//这部分耗费，内部有降采样的部分，ndt.setResolution

                VPointCloud::Ptr PointSource_ptr(new VPointCloud(subMap[inWhichSubMapId].sumOfAllKeyScan));
                VPointCloud::Ptr filtered_cloud (new VPointCloud);
                downsampleCloud(PointSource_ptr, filtered_cloud, 0.5);
                ndt.setInputSource (filtered_cloud);

                VPointCloud::Ptr output_cloud (new VPointCloud);
                ndt.align (*output_cloud, subMap[inWhichSubMapIdFront].subMapGlobalPose.inverse() *
                           subMap[inWhichSubMapId].subMapGlobalPose);//必须加上预测，否则迭代会很慢，里程计的帧数跟不上激光发布的速度，直接导致建图效果很差

                subMap[inWhichSubMapId].subMapGlobalPose = subMap[inWhichSubMapIdFront].subMapGlobalPose * ndt.getFinalTransformation();
                newScanGlobalPose = subMap[inWhichSubMapId].subMapGlobalPose * subMap[inWhichSubMapId].newScanLocalPose;
                _globalPosition = newScanGlobalPose;
            }

            //之后再检测当前位置获得的scan是否进入到了历史中存在的submap范围内
            unsigned int historySubMapId;
            bool flag1 = checkIfInHistorySubMapRange(historySubMapId);
            if(flag1 && historySubMapId != inWhichSubMapId)//重要！因为前面增加了submap构建完毕胡修改全局位置，会导致当前的scan又回到了当前的submap中，从而产生错误的闭环！所以增加一个判断条件
            {
              //当进入到某一个历史帧当中 就需要进行一次闭环优化

              if(locationWay == 1)//上一次是新建的地图
              {
                ceresPoses.clear();
                ceresConstraints.clear();
                for(unsigned int i=0 ;i<subMap.size(); i++)
                {
                  Eigen::Vector3f r_ = subMap[i].subMapGlobalPose.block<3,3>(0,0).eulerAngles(0,1,2);//roll pitch yaw 顺序
                  Pose2d temp;
                  temp.x = subMap[i].subMapGlobalPose(0,3);
                  temp.y = subMap[i].subMapGlobalPose(1,3);
                  temp.yaw_radians = r_(2);
                  ceresPoses.insert(std::make_pair(i, temp));
                }

                {
                Eigen::Vector3f r_ = newScanGlobalPose.block<3,3>(0,0).eulerAngles(0,1,2);//roll pitch yaw 顺序
                Pose2d temp;
                temp.x = newScanGlobalPose(0,3);
                temp.y = newScanGlobalPose(1,3);
                temp.yaw_radians = r_(2);
                ceresPoses.insert(std::make_pair(subMap.size(), temp));//增加当前节点位闭环虚拟节点。
                }

                for(unsigned int i=0; i<mapEdge.size(); i++)
                {
                  Eigen::Vector3f r_ = mapEdge[i].transformT.block<3,3>(0,0).eulerAngles(0,1,2);

                  Constraint2d temp;
                  temp.id_begin = mapEdge[i].idFirst;
                  temp.id_end = mapEdge[i].idSecond;
                  temp.information << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
                  temp.x = mapEdge[i].transformT(0, 3);
                  temp.y = mapEdge[i].transformT(1, 3);
                  temp.yaw_radians = r_(2);
                  ceresConstraints.push_back(temp);
                }
                //虚拟边
                {
                  Eigen::Matrix4f newScanLocalPose = subMap[inWhichSubMapId].subMapGlobalPose.inverse() * newScanGlobalPose;
                  Eigen::Vector3f r_ = newScanLocalPose.block<3,3>(0,0).eulerAngles(0,1,2);

                  Constraint2d temp;
                  temp.id_begin = inWhichSubMapId;
                  temp.id_end = subMap.size();//指向当前虚拟节点
                  temp.information << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
                  temp.x = newScanLocalPose(0, 3);
                  temp.y = newScanLocalPose(1, 3);
                  temp.yaw_radians = r_(2);
                  ceresConstraints.push_back(temp);
                }
                //虚拟闭环边
                {
                  subMap[historySubMapId].setLocalPose(subMap[historySubMapId].subMapGlobalPose.inverse() * newScanGlobalPose);
                  Eigen::Matrix4f newScanGlobalPoseInHistory = subMap[historySubMapId].getPose(_newScan);

                  Eigen::Matrix4f newScanLocalPoseInHistory = subMap[historySubMapId].subMapGlobalPose.inverse() * newScanGlobalPoseInHistory;
                  Eigen::Vector3f r_ = newScanLocalPoseInHistory.block<3,3>(0,0).eulerAngles(0,1,2);

                  Constraint2d temp;
                  temp.id_begin = historySubMapId;
                  temp.id_end = subMap.size();//指向当前虚拟节点
                  temp.information << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
                  temp.x = newScanLocalPoseInHistory(0, 3);
                  temp.y = newScanLocalPoseInHistory(1, 3);
                  temp.yaw_radians = r_(2);
                  ceresConstraints.push_back(temp);
                }
                //至此数据准备完毕
                Optimization();
                //优化后将数据重新赋值节点
                for(unsigned int i=0 ;i<subMap.size(); i++)
                {
                  Pose2d temp = ceresPoses.find(i)->second;
                  //2d 转 3d
                  Eigen::AngleAxisf rotationVector(temp.yaw_radians, Eigen::Vector3f(0,0,1));
                  Eigen::Matrix4f result = Eigen::Isometry3f::Identity().matrix();
                  result.block<3,3>(0,0) = rotationVector.matrix();
                  result(0,3) = temp.x;
                  result(1,3) = temp.y;
                  subMap[i].subMapGlobalPose = result;
                }
                //把当前帧的全局pose修正
                {
                  Pose2d temp = ceresPoses.find(subMap.size())->second;
                  //2d 转 3d
                  Eigen::AngleAxisf rotationVector(temp.yaw_radians, Eigen::Vector3f(0,0,1));
                  Eigen::Matrix4f result = Eigen::Isometry3f::Identity().matrix();
                  result.block<3,3>(0,0) = rotationVector.matrix();
                  result(0,3) = temp.x;
                  result(1,3) = temp.y;
                  newScanGlobalPose = result;
                  _globalPosition = newScanGlobalPose;
                }
                //之后更新edge



              }
              else//上一次是重定位
              {

              }


                std::cout << "last method is 1. in new map use method 2." << std::endl;
                std::cout << "last subMap id is " << inWhichSubMapId << "  now subMap id is " << historySubMapId << std::endl;
                //进入一个已有的submap，切换到当前submap中
                inWhichSubMapIdFront = inWhichSubMapId;
                inWhichSubMapId = historySubMapId;
                subMap[inWhichSubMapId].setLocalPose(subMap[inWhichSubMapId].subMapGlobalPose.inverse() * newScanGlobalPose);
                locationWay = 2;

            }
            else
            {
              //创建新的submap
                int id = subMap.size();
              //创建边
                mapEdge.push_back(MapEdge(inWhichSubMapId, id, subMap[inWhichSubMapId].subMapGlobalPose.inverse() * newScanGlobalPose));
                subMap.push_back(SubMap(id, _newScan, newScanGlobalPose, PoseInOtherSubMapRange));
                inWhichSubMapIdFront = inWhichSubMapId;
                inWhichSubMapId = id;
                locationWay = 1;
                std::cout << "init a new submap. id = " << inWhichSubMapId << std::endl;
            }
        }
    }

    //基于addScanV0.1， 在ceres做大回环优化时不使用闭环帧，直接使用产生关联的两帧submap
    void addScan2(VPointCloud _newScan, Eigen::Matrix4f &_globalPosition)//输入一帧当前数据，输出机器人全局位置。
    {
        if(!systemInitFlag)
        {
            PoseInOtherSubMapRange = 2.0;
            //创建新的submap
            int id = subMap.size();
            subMap.push_back(SubMap(id, _newScan, Eigen::Isometry3f::Identity().matrix(), PoseInOtherSubMapRange));
            inWhichSubMapIdFront = 0;
            inWhichSubMapId = 0;
            _globalPosition = newScanGlobalPose;
            systemInitFlag = true;
            std::cout << "init a new submap. id = " << inWhichSubMapId << std::endl;
            return;
        }

        newScanGlobalPose = subMap[inWhichSubMapId].getPose(_newScan);//输出当前位置，--无论是重定位还是里程计
        _globalPosition = newScanGlobalPose;

        if(subMap[inWhichSubMapId].checkIfInSubMapRange())
        {
            if(locationWay == 1)
            {
                if(subMap[inWhichSubMapId].reflash())
                {
                    std::cout << "add a keyScan" << std::endl;
                }
            }
        }
        else
        {
            if(locationWay == 1)
            {
                //当超过某个正在建立的submap范围后,表示之前的submap建立完毕
                //先对当前submap进行图优化调节每个关键scan的rt,再进行所有关键scan求和，并且之后不再改变。
                newScanGlobalPose = subMap[inWhichSubMapId].finish();//还没实现submap内部闭环
                //_globalPosition = newScanGlobalPose;
                std::cout << "complete a new submap. id = " << inWhichSubMapId << std::endl;

                //之后基于当前建立的submap的总地图和上一个submap总地图进行匹配，修正当前submap的初始全局pose，
                //因为当前submap初始全局pose是由第一帧scan和上一个submap匹配获得的，数据量太少。
                VPointCloud::Ptr PointMatched_ptr (new VPointCloud(subMap[inWhichSubMapIdFront].sumOfAllKeyScan));
                ndt.setInputTarget (PointMatched_ptr);//这部分耗费，内部有降采样的部分，ndt.setResolution

                VPointCloud::Ptr PointSource_ptr(new VPointCloud(subMap[inWhichSubMapId].sumOfAllKeyScan));
                VPointCloud::Ptr filtered_cloud (new VPointCloud);
                downsampleCloud(PointSource_ptr, filtered_cloud, 0.5);
                ndt.setInputSource (filtered_cloud);

                VPointCloud::Ptr output_cloud (new VPointCloud);
                ndt.align (*output_cloud, subMap[inWhichSubMapIdFront].subMapGlobalPose.inverse() *
                           subMap[inWhichSubMapId].subMapGlobalPose);//必须加上预测，否则迭代会很慢，里程计的帧数跟不上激光发布的速度，直接导致建图效果很差

                subMap[inWhichSubMapId].subMapGlobalPose = subMap[inWhichSubMapIdFront].subMapGlobalPose * ndt.getFinalTransformation();
                newScanGlobalPose = subMap[inWhichSubMapId].subMapGlobalPose * subMap[inWhichSubMapId].newScanLocalPose;
                _globalPosition = newScanGlobalPose;
            }

            //之后再检测当前位置获得的scan是否进入到了历史中存在的submap范围内
            unsigned int historySubMapId;
            bool flag1 = checkIfInHistorySubMapRange(historySubMapId);
            if(flag1 && historySubMapId != inWhichSubMapId)//重要！因为前面增加了submap构建完毕胡修改全局位置，会导致当前的scan又回到了当前的submap中，从而产生错误的闭环！所以增加一个判断条件
            {
              //当进入到某一个历史帧当中 就需要进行一次闭环优化

              if(locationWay == 1)//上一次是新建的地图
              {
                ceresPoses.clear();
                ceresConstraints.clear();
                for(unsigned int i=0 ;i<subMap.size(); i++)
                {
                  Eigen::Vector3f r_ = subMap[i].subMapGlobalPose.block<3,3>(0,0).eulerAngles(0,1,2);//roll pitch yaw 顺序
                  Pose2d temp;
                  temp.x = subMap[i].subMapGlobalPose(0,3);
                  temp.y = subMap[i].subMapGlobalPose(1,3);
                  temp.yaw_radians = r_(2);
                  ceresPoses.insert(std::make_pair(i, temp));
                }

                for(unsigned int i=0; i<mapEdge.size(); i++)
                {
                  Eigen::Vector3f r_ = mapEdge[i].transformT.block<3,3>(0,0).eulerAngles(0,1,2);

                  Constraint2d temp;
                  temp.id_begin = mapEdge[i].idFirst;
                  temp.id_end = mapEdge[i].idSecond;
                  temp.information << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
                  temp.x = mapEdge[i].transformT(0, 3);
                  temp.y = mapEdge[i].transformT(1, 3);
                  temp.yaw_radians = r_(2);
                  ceresConstraints.push_back(temp);
                }

                //虚拟闭环边
                {
                  subMap[historySubMapId].setLocalPose(subMap[historySubMapId].subMapGlobalPose.inverse() * subMap[inWhichSubMapId].subMapGlobalPose);// 给一个初始迭代的位置
                  Eigen::Matrix4f newScanGlobalPoseInHistory = subMap[historySubMapId].getPose(subMap[inWhichSubMapId].sumOfAllKeyScan);//这个变量的名字就不改了，下同。

                  Eigen::Matrix4f newScanLocalPoseInHistory = subMap[historySubMapId].subMapGlobalPose.inverse() * newScanGlobalPoseInHistory;
                  Eigen::Vector3f r_ = newScanLocalPoseInHistory.block<3,3>(0,0).eulerAngles(0,1,2);

                  Constraint2d temp;
                  temp.id_begin = historySubMapId;
                  temp.id_end = inWhichSubMapId;
                  temp.information << 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1;
                  temp.x = newScanLocalPoseInHistory(0, 3);
                  temp.y = newScanLocalPoseInHistory(1, 3);
                  temp.yaw_radians = r_(2);
                  ceresConstraints.push_back(temp);
                }
                //至此数据准备完毕
                Optimization();
                //优化后将数据重新赋值节点
                for(unsigned int i=0 ;i<subMap.size(); i++)
                {
                  Pose2d temp = ceresPoses.find(i)->second;
                  //2d 转 3d
                  Eigen::AngleAxisf rotationVector(temp.yaw_radians, Eigen::Vector3f(0,0,1));
                  Eigen::Matrix4f result = Eigen::Isometry3f::Identity().matrix();
                  result.block<3,3>(0,0) = rotationVector.matrix();
                  result(0,3) = temp.x;
                  result(1,3) = temp.y;
                  subMap[i].subMapGlobalPose = result;
                }
                //把当前帧的全局pose修正
                {
                  newScanGlobalPose = subMap[inWhichSubMapId].subMapGlobalPose * subMap[inWhichSubMapId].newScanLocalPose;
                  _globalPosition = newScanGlobalPose;
                }
                //之后更新edge



              }
              else//上一次是重定位
              {

              }


                std::cout << "last method is 1. in new map use method 2." << std::endl;
                std::cout << "last subMap id is " << inWhichSubMapId << "  now subMap id is " << historySubMapId << std::endl;
                //进入一个已有的submap，切换到当前submap中
                inWhichSubMapIdFront = inWhichSubMapId;
                inWhichSubMapId = historySubMapId;
                subMap[inWhichSubMapId].setLocalPose(subMap[inWhichSubMapId].subMapGlobalPose.inverse() * newScanGlobalPose);
                locationWay = 2;

            }
            else
            {
              //创建新的submap
                int id = subMap.size();
              //创建边
                mapEdge.push_back(MapEdge(inWhichSubMapId, id, subMap[inWhichSubMapId].subMapGlobalPose.inverse() * newScanGlobalPose));
                subMap.push_back(SubMap(id, _newScan, newScanGlobalPose, PoseInOtherSubMapRange));
                inWhichSubMapIdFront = inWhichSubMapId;
                inWhichSubMapId = id;
                locationWay = 1;
                std::cout << "init a new submap. id = " << inWhichSubMapId << std::endl;
            }
        }
    }
    std::vector<SubMap> subMap;
    std::vector<MapEdge> mapEdge;

    unsigned int inWhichSubMapIdFront;//上一个submap
    unsigned int inWhichSubMapId;
    double PoseInOtherSubMapRange;
    bool systemInitFlag;
    int locationWay;//1 里程计，2重定位。

    Eigen::Matrix4f newScanGlobalPose;

    //用于优化
    std::map<int, Pose2d> ceresPoses;//最后一个节点是矛盾节点，是虚拟的，不代表地图
    std::vector<Constraint2d> ceresConstraints;
    pcl::NormalDistributionsTransform<VPoint, VPoint> ndt;
private:
    //检测当前位置是否处于其他已经建立的submap区域中，如果处于，则返回ture和submap的id号
    bool checkIfInHistorySubMapRange(unsigned int &id)
    {
        for(unsigned int i=0; i<subMap.size(); i++)
        {
            if(pow(newScanGlobalPose(0,3)-subMap[i].subMapGlobalPose(0,3), 2) +
               pow(newScanGlobalPose(1,3)-subMap[i].subMapGlobalPose(1,3), 2) +
               pow(newScanGlobalPose(2,3)-subMap[i].subMapGlobalPose(2,3), 2) <
                    pow(PoseInOtherSubMapRange,2)
                    )
            {
               id = i;
               return true;
            }
        }
        return false;
    }

    void showGlobalPose()
    {
//        static int counter = 0;
//        counter++;
//        if(counter<20)
//            return;
//        counter = 0;
        Eigen::Matrix3f r_TransformAllmatrix = newScanGlobalPose.block<3,3>(0,0);
        Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "newScanGlobalPose (x,y,th): " << "(" <<\
                     newScanGlobalPose(0, 3) << " " << newScanGlobalPose(1, 3) << " "<< r_(2) << ")" << std::endl;
        std::cout.unsetf( std::ios::fixed );

    }

    void Optimization()
    {
      std::cout << "Number of poses: " << ceresPoses.size() << '\n';
      std::cout << "Number of constraints: " << ceresConstraints.size() << '\n';

      if (!OutputPoses("/home/xjh/poses_original.txt", ceresPoses)) {
      }

      ceres::Problem problem;
      BuildOptimizationProblem(ceresConstraints, &ceresPoses, &problem);

      if (!SolveOptimizationProblem(&problem)) {
        std::cout << "The solve was not successful, exiting.\n";
      }

      if (!OutputPoses("/home/xjh/poses_optimized.txt", ceresPoses)) {
      }

    }
};



#endif
