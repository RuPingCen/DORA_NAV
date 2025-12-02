#pragma once

#include <vector>
#include <cstdint>
#include <string>

struct GridMap {
    std::vector<int8_t> data;    // -1=未知, 0=空闲, 100=障碍
    size_t width;
    size_t height;
    float resolution;            // 米/格
    float origin[3];             // [x, y, z]
    std::string frame_id;
    
    // 检查坐标是否在地图范围内
    bool in_bounds(int x, int y) const {
        return x >= 0 && x < static_cast<int>(width) && 
               y >= 0 && y < static_cast<int>(height);
    }
    
    // 获取指定位置的占用值
    int8_t get_value(int x, int y) const {
        if (!in_bounds(x, y)) return -1;
        return data[y * width + x];
    }
    
    // 世界坐标转网格坐标
    bool world_to_grid(float world_x, float world_y, int& grid_x, int& grid_y) const {
        float local_x = world_x - origin[0];
        float local_y = world_y - origin[1];
        
        grid_x = static_cast<int>(local_x / resolution);
        grid_y = static_cast<int>(local_y / resolution);
        
        return in_bounds(grid_x, grid_y);
    }
    
    // 网格坐标转世界坐标
    void grid_to_world(int grid_x, int grid_y, float& world_x, float& world_y) const {
        world_x = origin[0] + (grid_x + 0.5f) * resolution;
        world_y = origin[1] + (grid_y + 0.5f) * resolution;
    }
};