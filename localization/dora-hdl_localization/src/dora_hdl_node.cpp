#include <asm-generic/errno.h>
#include <type_traits>
extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <fstream>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <thread>  
#include <chrono>  

#include "hdl_localization.hpp"

#define imu_dt 0.05



pcl::PointCloud<pcl::PointXYZI>::Ptr init_map(double map_downsample_resolution, std::string map_pcd_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap;
    globalmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile(map_pcd_path, *globalmap) == -1) 
    { 
        cerr << "[ERROR] Could not read file: " << map_pcd_path << endl;
        return nullptr;
    }
    double downsample_resolution = map_downsample_resolution;
    boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid->filter(*filtered);

    return filtered;
}

//*********************************test downsample*********************************************************

// pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) 
// {
//   boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid_1(new pcl::VoxelGrid<pcl::PointXYZI>());
//   voxelgrid_1->setLeafSize(0.1, 0.1, 0.1);
//   voxelgrid_1->setInputCloud(cloud);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
//   voxelgrid_1->filter(*filtered);
// //-------------------------------------------------------------------------------------------------------------------------
//     // pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
//     // voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
//     // voxelgrid.setInputCloud(cloud);
//     //  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());

//     // voxelgrid.filter(*filtered);
//   return filtered;
// }

//************************************************************************************************************


pcl::PointCloud<pcl::PointXYZI>::Ptr bytes2cloud(const char *bytes, int32_t size)
{
    if (size <= 0)
    {
        std::cerr << "Error: Point cloud size <= 0!" << std::endl;
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr row_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    row_cloud->header.seq = *(std::uint32_t *)bytes;
    row_cloud->header.stamp = *(std::uint64_t *)(bytes + 8);
    // std::cout << "row_cloud->header.stamp: " << row_cloud->header.stamp << std::endl;
    row_cloud->header.frame_id = "rslidar";
    row_cloud->width = size;
    row_cloud->height = 1;
    row_cloud->is_dense = false;
    for (size_t i = 0; i < size; i++) 
    {
        pcl::PointXYZI tem_point;
        tem_point.x = *(float *)(bytes + 16 + 16 * i);
        tem_point.y = *(float *)(bytes + 16 + 4 + 16 * i);
        tem_point.z = *(float *)(bytes + 16 + 8 + 16 * i);
        tem_point.intensity = *(float *)(bytes + 16 + 12 + 16 * i);
        row_cloud->points.push_back(tem_point);
    }

    return row_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr rslidar2baselink(const pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //TODO 90°
    transform(0, 1) = -1; 
    transform(1, 0) = 1; 
    transform(0, 0) = 0; 
    transform(1, 1) = 0; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*points, *trans_cloud, transform);

    return trans_cloud;
}


bool run_once(Hdl_Localization& hdl_loc, const char *data, int32_t point_len, void* dora_context, std::ofstream& points_xy, bool use_imu, bool get_imu)
{
    auto clouds = bytes2cloud(data, point_len);
    if (clouds == nullptr)
    {
        std::cerr << "Error: Failed to rec point cloud!" << std::endl;
    }

    // pcl::io::savePCDFileASCII("clouds.pcd", *clouds);
    //***************************过滤天花板********************************

    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud(clouds);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-1.0, 2.5);  
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // pass.filter(*cloud_filtered);

    //********************************************************************

    double stamp = (double)clouds->header.stamp*1.0*1e-6;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*clouds, *cloud_no_nan, indices);



    auto filtered = hdl_loc.downsample(cloud_no_nan);
    // pcl::io::savePCDFileASCII("output.pcd", *filtered);
    auto trans_clouds = rslidar2baselink(filtered);
    // pcl::io::savePCDFileASCII("trans_clouds.pcd", *trans_clouds);

    hdl_loc.last_scan = trans_clouds;

    if(use_imu)
    {
        if(!get_imu)
        {
            std::cerr << "imu data is not ready" << std::endl;
            return false;
        }
        else 
        {
            auto imu_iter = hdl_loc.imu_data.begin();
            for (imu_iter; imu_iter != hdl_loc.imu_data.end(); imu_iter++) 
            {
                // TODO: check stamp
                if (imu_iter->stamp > stamp)
                {
                    break;
                }
                if(imu_iter->stamp + imu_dt < stamp)//获取最近距离雷达点云最近一帧的imu数据
                {
                    continue;
                }
                const auto& acc = imu_iter->linear_acceleration;
                const auto& gyro = imu_iter->angular_velocity;

                if (std::isnan(acc.x) || std::isnan(acc.y) || std::isnan(acc.z) ||
                    std::isnan(gyro.x) || std::isnan(gyro.y) || std::isnan(gyro.z)) 
                {
                    continue;
                }
                double acc_sign = 1.0;
                double gyro_sign = 1.0;

                hdl_loc.pose_estimator->predict(
                    imu_iter->stamp,
                    acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
                    gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z)
                );
            }
            hdl_loc.imu_data.erase(hdl_loc.imu_data.begin(),imu_iter);
        }
    }
    else
    {
        hdl_loc.pose_estimator->predict(stamp); //不使用imu
    }

    auto aligned = hdl_loc.pose_estimator->correct(stamp, trans_clouds);
    auto cur_pose = hdl_loc.compute_odometry(hdl_loc.pose_estimator->matrix());
    points_xy << cur_pose.x << " " << cur_pose.y << std::endl;

    std::string out_id = "cur_pose";
    canslam::slampose *cur_pose_ptr = &cur_pose;
    char *output_data = (char*)cur_pose_ptr;
    size_t output_data_len = sizeof(canslam::slampose);
    int result_pose = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
    if(result_pose != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    return true;
}

int run(void *dora_context, Hdl_Localization& hdl_loc, std::ofstream& points_xy, bool use_imu)
{
    bool get_imu = false;
    while (true)
    {
        void * event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            int32_t point_len = (data_len - 16) / 16;         
            if (strncmp("pointcloud", data_id, 10) == 0)
            {
                // static int count = 0;
                // struct timeval tv;
                // gettimeofday(&tv, NULL);//获取时间
                // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
                // std::cout << std::fixed << std::setprecision(6)<< "start : " << start << " The count: " << count++ << std::endl;
                // struct timeval tv_1;
                // gettimeofday(&tv_1, NULL);//获取时间
                // auto end = tv_1.tv_sec + tv_1.tv_usec * 1e-6;
                // auto all_time = end - start;
                // std::cout << "Time: " << all_time << std::endl;

                //--------------------------------------------------------------------------------------------------------------

                bool once_slam = run_once(hdl_loc, data, point_len, dora_context, points_xy, use_imu, get_imu);
                if(!once_slam)
                {
                    std::cerr << "failed to run slam once" << std::endl;
                    return -1;
                }
                // std::cout << "pointcloud" << std::endl;
            }
            if (strncmp("imu_msg", data_id, 7) == 0)
            {
                //TODO add later 
                canslam::imu_msg_h *imu_msg = reinterpret_cast<canslam::imu_msg_h *>(data);
                hdl_loc.imu_data.push_back(*imu_msg);
                get_imu = true;
            }

        }

        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}


int main()
{
    std::cout << "hdl_slam_localization" << std::endl;
    void* dora_context = init_dora_context_from_env();

    double map_downsample_resolution = std::getenv("map_downsample_resolution") ? std::stod(std::getenv("map_downsample_resolution")) : 0.1;
    double point_downsample_resolution = std::getenv("point_downsample_resolution") ? std::stod(std::getenv("point_downsample_resolution")) : 0.1;
    std::cout << "map_downsample_resolution : " << map_downsample_resolution << " point_downsample_resolution: " << point_downsample_resolution << std::endl;
    
    const char* use_imu_env = std::getenv("use_imu");
    bool use_imu = (use_imu_env && std::string(use_imu_env) == "1");
    if(use_imu)
    {
        std::cout << "use imu!!!" << std::endl;
    }
    else std::cout << "do not use imu!!!" << std::endl;

    
    const char *env_pcd_path = getenv("MAP_PCD");
    std::string map_pcd_path;
    if (env_pcd_path == nullptr) {
        map_pcd_path = "./data/map.pcd";
        std::cout << "MAP_PCD not set , use default path: " << map_pcd_path << std::endl;
    }
    else
    {
        map_pcd_path = env_pcd_path;
        std::cout << "PCD path is : " << map_pcd_path << std::endl;
    }


    const char *env_path = getenv("way_points");
    std::string way_points_path;
    if (env_path == nullptr) {
        way_points_path = "./data/path/trajectory.txt";
        std::cout << "way_points not set, using default path: " << way_points_path << std::endl;
    }
    else
    {
        way_points_path = env_path;
        std::cout << "way_points path is : " << map_pcd_path << std::endl;
    }

    std::ofstream points_xy(way_points_path, std::ios::trunc);
    if (!points_xy) 
    {
        std::cerr << "Fail to open the file!!" << std::endl;
        return -1;
    }

    Hdl_Localization hdl_loc;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map = init_map(map_downsample_resolution, map_pcd_path);
    // std::cout << "downsample globalmap : \n" << *pcd_map << std::endl;
    bool localization_param = hdl_loc.init_param(point_downsample_resolution);
    if(!localization_param)
    {
        std::cerr << "Fail to init hdl_loc!!! " << std::endl;
        return -1;
    }

    hdl_loc.registration->setInputTarget(pcd_map);

    // std::this_thread::sleep_for(std::chrono::seconds(5));   
    int ret = run(dora_context, hdl_loc, points_xy, use_imu);
    
    points_xy.close();
    free_dora_context(dora_context);
    std::cout << "END hdl_slam_localization" << std::endl;

    return ret;
}