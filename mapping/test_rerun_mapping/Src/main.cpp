extern "C" {
    #include "node_api.h"
}

#include <iostream>
#include <string.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include <thread>
#include <chrono>
#include <mutex>

typedef struct pose
{
    double x;
    double y;
    double yaw;
}Pose_t;

Pose_t current_pose;

std::vector<float> global_map_data;
std::mutex mtx;

const int SAVE_INTERVAL_SEC = 1;
const std::string MAP_FILENAME = "mapping.pcd";


void save_map()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(SAVE_INTERVAL_SEC));
        std::lock_guard<std::mutex> lock(mtx);
        if ( global_map_data.empty()) continue;
        pcl::PointCloud<pcl::PointXYZI> map_cloud;
        size_t num_points = global_map_data.size() / 3;
        map_cloud.reserve(num_points);
        for ( size_t i = 0 ; i < num_points ; i ++)
        {
            float x = global_map_data[i * 3];
            float y = global_map_data[i * 3 + 1];
            float z = global_map_data[i * 3 + 2];
            map_cloud.push_back(pcl::PointXYZI(x,y,z,0.0f));
        }
        if ( pcl::io::savePCDFileBinary(MAP_FILENAME,map_cloud) == 0)
        {
            // std::cout << "Successfully saved LATEST map to" << MAP_FILENAME << " with " << num_points << " points. " << std::endl;
        }
        else
        {
            std::cerr << "Error: Failed to save map!!!!!!!!!!!!!!!!!" << std::endl;
        }
    }
    
}

void PointCloud_Process(char *data, size_t data_len, rerun::RecordingStream& rec)
{
    std::vector<rerun::Position3D> rerun_points_PointCloud;
    if ( data_len == 0 || data_len % ( 3 * sizeof(float)) != 0)
    {
        std::cout << "PCL Data is Error!!!!!!!!!!!!!!!!" << std::endl;
    }
    else
    {
        // std::cout << "Recv PCL Data Successfully!!!!!!!!!!!!" << std::endl;
        float *data_ = reinterpret_cast<float*>(data);
        size_t num_points = data_len / (3 * sizeof(float));
        // std::cout << "recv num_points = " << num_points << std::endl;
        rerun_points_PointCloud.reserve(num_points);

        std::lock_guard<std::mutex> lock(mtx);
        global_map_data.assign(data_, data_ + num_points * 3);

        pcl::PointXYZ tem_point;
        for (size_t i = 0 ; i < num_points ; i ++)
        {
            tem_point.x = data_[i * 3];
            tem_point.y = data_[i * 3 + 1];
            tem_point.z = data_[i * 3 + 2];
            rerun_points_PointCloud.emplace_back(tem_point.x,tem_point.y,tem_point.z);
        }
        rec.log("PointCloud", rerun::Points3D(rerun_points_PointCloud).with_colors(0x00FF00FF).with_radii({0.02f}));
    }
}

void CurPose_Process(char *data, size_t data_len, rerun::RecordingStream& rec)
{
    if ( data_len != sizeof(Pose_t))
    {
        std::cout << "Pose Data Error!!!!!!!!!!!!!!!" << std::endl;
        return;
    }
    memcpy(&current_pose,data,sizeof(Pose_t));
    // std::cout << "current pose: x = " << current_pose.x << " y = " << current_pose.y << " yaw =  " << current_pose.yaw << std::endl;

    std::vector<rerun::Position3D> origins;
    std::vector<rerun::Vector3D> vectors;

    float theta = current_pose.yaw;  
    float length = 0.8f;  
 
    origins.push_back({current_pose.x, current_pose.y, 0.0});
    vectors.push_back({length * cosf(theta), length * sinf(theta), 0.0});

    rec.log(
        "arrows",
        rerun::Arrows3D::from_vectors(vectors).with_origins(origins).with_colors(0xFF00FFFF)
    );
}

int run(void *dora_context, rerun::RecordingStream& rec)
{
    while (true)
    {
        void *event = dora_next_event(dora_context);
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

            if ( strncmp(data_id,"pose_data",data_id_len) == 0)
            {
                CurPose_Process(data, data_len, rec);
            }
            else if ( strncmp(data_id,"mapping_data",data_id_len) == 0)
            {
                PointCloud_Process(data,data_len, rec);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            // mapper.saveMap();
            printf("[c node] received stop event\n");
            free_dora_event(event);
        }
        else
        {
            // mapper.saveMap();
            printf("[c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
        }
    }
}

int main()
{
    std::cout << "Hello World" << std::endl;

    rerun::RecordingStream rec("pointcloud_viewer");
    rec.spawn().exit_on_failure();

    std::thread save_thread(save_map);
    save_thread.detach();

    void *dora_context = init_dora_context_from_env();

    run(dora_context, rec);

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");
}