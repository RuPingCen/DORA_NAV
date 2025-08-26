#ifndef HDL_LOCALIZATION
#define HDL_LOCALIZATION


#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pclomp/ndt_omp.h>


#include "pose_estimator.hpp"
#include "delta_estimater.hpp"
#include "imu_msg.hpp"
#include "getYaw.hpp"
#include "slam_pose.hpp"


using namespace std;

class Hdl_Localization
{
public:
    bool init_param(double point_downsample_resolution);
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr create_registration();
    canslam::slampose compute_odometry(const Eigen::Matrix4f& pose);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);



public:
    std::atomic_bool relocalizing;
    std::unique_ptr<hdl_localization::DeltaEstimater> delta_estimater;
    std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
    pcl::Filter<pcl::PointXYZI>::Ptr downsample_filter;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr last_scan;
    std::vector<canslam::imu_msg_h> imu_data;
};


pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr Hdl_Localization::create_registration()
{
  std::string reg_method = "NDT_OMP";
  std::string ndt_neighbor_search_method = "DIRECT7";
  double ndt_neighbor_search_radius = 2.0;
  double ndt_resolution = 1.0;

  if(reg_method == "NDT_OMP") 
  {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if (ndt_neighbor_search_method == "DIRECT1") 
    {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    }
    else if (ndt_neighbor_search_method == "DIRECT7")
    {
      std::cout << "The method is : " << ndt_neighbor_search_method << std::endl;
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } 
    return ndt;
  } 
  return nullptr;
}

bool Hdl_Localization::init_param(double point_downsample_resolution)
{
    double downsample_resolution = point_downsample_resolution;
    boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    registration = create_registration();

    relocalizing = false;
    delta_estimater.reset(new hdl_localization::DeltaEstimater(create_registration()));

    pose_estimator.reset(
      new hdl_localization::PoseEstimator(
        registration,
        Eigen::Vector3f(0.0, 0.0, 0.0),
        Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0),
        2
      )
    );
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Hdl_Localization::downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  if (!downsample_filter) {
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  downsample_filter->setInputCloud(cloud);
  downsample_filter->filter(*filtered);
  // filtered->header = cloud->header;

  return filtered;
}

canslam::slampose Hdl_Localization::compute_odometry(const Eigen::Matrix4f& pose)
{
    canslam::slampose slam_pose;

    Eigen::Isometry3d isometry = Eigen::Isometry3d(pose.cast<double>());
    // 提取平移部分
    slam_pose.x = isometry.translation().x();
    slam_pose.y = isometry.translation().y();

    // // 获取偏航角（yaw）并转换为度数
    Eigen::Matrix3d rotationMatrix = pose.block<3, 3>(0, 0).cast<double>(); 
    auto yaw = getYaw(rotationMatrix);
    slam_pose.theta = yaw * 180.0 / M_PI;

    // // TODO: theta
    slam_pose.theta = slam_pose.theta < 0.0 ? ( -slam_pose.theta) : (360.0 -slam_pose.theta);    //slam坐标旋转角度180.... -180转换到0到360；
    slam_pose.theta = (slam_pose.theta >= 0.0 && slam_pose.theta<90.0) ? (90.0 - slam_pose.theta) : (450.0 - slam_pose.theta);   

    return slam_pose;
}


#endif