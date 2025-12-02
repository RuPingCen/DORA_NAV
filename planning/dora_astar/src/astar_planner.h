#pragma once

#include <dora/node.h>
#include "grid_map.h"
#include "path_planning.h"
#include <arrow/ipc/reader.h>
#include <arrow/io/memory.h>

class AStarPlannerNode {
private:
    dora::Node node_;
    GridMap grid_map_;
    AStarPlanner* planner_;
    float inflation_radius_;
    float heuristic_weight_;
    bool allow_diagonal_;
    
public:
    AStarPlannerNode();
    ~AStarPlannerNode();
    void run();
    
private:
    // 从Arrow数据反序列化栅格地图
    bool deserialize_grid_map_from_arrow(const std::vector<uint8_t>& data);
    
    // 处理节点事件
    void handle_event(const dora::Event& event);
    
    // 处理路径规划请求
    void handle_path_request(const dora::DoraEvent& event);
    
    // 将路径序列化为Arrow格式
    std::vector<uint8_t> serialize_path_to_arrow(const Path& path) const;
};