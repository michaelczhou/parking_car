#ifndef _3D_PARTY_LIBRARIES_H_
#define _3D_PARTY_LIBRARIES_H_

#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


#include <eigen3/Eigen/Core>

#include <tf/tf.h>

//数据类型不影响ndt
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

void downsampleCloud(const VPointCloud::Ptr in_cloud_ptr, \
                     VPointCloud::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::VoxelGrid<VPoint> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}


#endif
