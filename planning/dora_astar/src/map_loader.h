#pragma once

#include <dora/node.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "grid_map.h"

class MapLoaderNode {
private:
    dora::Node node_;
    std::string map_type_;       // "grid" or "pointcloud"
    std::string map_path_;       // 地图文件路径，不含扩展名
    GridMap grid_map_;           // 栅格地图数据
    
public:
    MapLoaderNode();
    void run();
    
private:
    // 加载ROS格式的栅格地图（图片+YAML）
    bool load_grid_map();
    
    // 加载点云地图
    bool load_pointcloud_map();
    
    // 将栅格地图转换为Arrow格式
    std::vector<uint8_t> serialize_grid_map_to_arrow() const;
    
    // 将点云地图转换为Arrow格式
    std::vector<uint8_t> serialize_pointcloud_to_arrow() const;
    
    // 处理节点事件
    void handle_event(const dora::Event& event);
};