extern "C"
{
#include "node_api.h"   
#include "operator_api.h"
#include "operator_types.h"
}
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <chrono>
#include <thread>
#include <fstream>

#include "dora_ndt_mapper.h"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <vector>

typedef struct pose
{
    double x;
    double y;
    double yaw;
}Pose_t;

Pose_t cur_pose__;

void Send_PoseData(void *dora_context, ndt_mapping::Pose& cur_pose)
{
    cur_pose__.x   = cur_pose.x;
    cur_pose__.y   = cur_pose.y;
    cur_pose__.yaw = cur_pose.yaw;
    /* 先发送估计位姿 */
    std::string output_id = "pose_data";
    size_t id_len = output_id.length();
    char *output_data = reinterpret_cast<char*>(&cur_pose__);
    size_t output_data_len = sizeof(cur_pose__);
    dora_send_output(dora_context, &output_id[0], id_len, output_data, output_data_len);
}

void Send_MappingData(void *dora_context, pcl::PointCloud<pcl::PointXYZI> &map)
{


    /* 再发送点云数据 */
    std::string output_id = "mapping_data";
    size_t id_len = output_id.length();

    std::vector<float> coordinates;
    coordinates.reserve(map.size() * 3);
    for ( const auto& point : map.points)
    {
        coordinates.push_back(point.x);
        coordinates.push_back(point.y);
        coordinates.push_back(point.z);
    }

    dora_send_output(dora_context, &output_id[0], id_len, reinterpret_cast<char*>(coordinates.data()),
                     coordinates.size() * sizeof(float));
}


int max_save = 100;

int mapping(void *dora_context, ndt_mapping::NDTMapper &mapper){
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    std::chrono::time_point<std::chrono::steady_clock>  last_data_time,current_time;

    while(true){
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);

            if ( strncmp(data_id,"pointcloud",data_id_len) == 0)
            {
                // std::cout << "This is a dora ndt mapping node!" << std::endl;
                int32_t point_len = (data_len-16)/4 ;

                pcl::PointCloud<pcl::PointXYZI> pointcloud;
                pointcloud.header.seq = *(std::uint32_t*)data;
                pointcloud.header.stamp = *(std::uint64_t*)(data+8);
                pointcloud.header.frame_id = "lidar";
                pointcloud.width = point_len;
                pointcloud.height = 1;
                pointcloud.is_dense = true;

                for(int i = 0; i < (data_len-16)/16; i++){
                    pcl::PointXYZI tem_point;
                    
                    tem_point.x = *(float*)(data + 16 + 16 * i);
                    tem_point.y = *(float*)(data + 16 + 4 + 16 * i);
                    tem_point.z = *(float*)(data + 16 + 8 + 16 * i);
                    tem_point.intensity = *(float*)(data + 16 + 12 + 16 * i);
                    pointcloud.points.push_back(tem_point);
                }

                pointcloud_ptr = pointcloud.makeShared();
                mapper.points_callback(pointcloud_ptr);
                pointcloud.clear();
                pointcloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
                // std::cout << "完成计算一次" << std::endl;

                // std::cout << "Current Pose:  x = " << mapper.current_pose.x << " y = " << mapper.current_pose.y
                // << " yaw = " << mapper.current_pose.yaw << std::endl;
                /********************* 在这里发送dora数据 *********************/
                Send_MappingData(dora_context,mapper.map);
                Send_PoseData(dora_context,mapper.current_pose);

            }
            else if ( strncmp(data_id,"tick",data_id_len) == 0)
            {
                std::cout << "This is a test topic tick!!!!!!!!!!!!!" << std::endl;
            }


            last_data_time = std::chrono::steady_clock::now();

        }
        else if (ty == DoraEventType_Stop)
        {
            // mapper.saveMap();
            printf("[c node] received stop event\n");
        }
        else
        {
            // mapper.saveMap();
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);

        // // 超过3秒无新数据 保存地图
        // current_time = std::chrono::steady_clock::now();
        // auto time_since_last_data = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_data_time).count();    
        // if (time_since_last_data >= 3) 
        // { 
        //     mapper.saveMap();
        //     break;
        // }
    }

}


int main(int argc, char* argv[])
{
    printf("dora ndt mapping node\n");

    void *dora_context = init_dora_context_from_env();


    ndt_mapping::NDTMapper mapper;
    mapping(dora_context, mapper);


    printf("[c node] received 10 events\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}