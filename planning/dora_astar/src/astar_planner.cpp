#include "astar_planner.h"
#include <arrow/ipc/reader.h>
#include <arrow/io/memory.h>
#include <arrow/table.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <json/json.h>

AStarPlannerNode::AStarPlannerNode() {
    node_ = dora::Node::init().value();
    
    // 从配置中获取A*参数
    auto config = node_.config().value();
    inflation_radius_ = config["inflation_radius"].as_float().value_or(0.3f);
    heuristic_weight_ = config["heuristic_weight"].as_float().value_or(1.0f);
    allow_diagonal_ = config["allow_diagonal"].as_bool().value_or(true);
    
    planner_ = nullptr;
}

AStarPlannerNode::~AStarPlannerNode() {
    delete planner_;
}

bool AStarPlannerNode::deserialize_grid_map_from_arrow(const std::vector<uint8_t>& data) {
    try {
        // 1. 创建内存输入流
        auto buffer = std::make_shared<arrow::Buffer>(data.data(), data.size());
        auto input = std::make_shared<arrow::io::BufferReader>(buffer);
        
        // 2. 读取Arrow文件
        std::shared_ptr<arrow::ipc::RecordBatchFileReader> file_reader;
        ARROW_CHECK_OK(arrow::ipc::RecordBatchFileReader::Open(input, &file_reader));
        
        // 3. 读取元数据
        auto schema = file_reader->schema();
        auto metadata = schema->metadata();
        
        // 4. 提取地图参数
        grid_map_.width = std::stoi(metadata->value(metadata->FindKey("width")));
        grid_map_.height = std::stoi(metadata->value(metadata->FindKey("height")));
        grid_map_.resolution = std::stof(metadata->value(metadata->FindKey("resolution")));
        grid_map_.origin[0] = std::stof(metadata->value(metadata->FindKey("origin_x")));
        grid_map_.origin[1] = std::stof(metadata->value(metadata->FindKey("origin_y")));
        grid_map_.origin[2] = std::stof(metadata->value(metadata->FindKey("origin_z")));
        grid_map_.frame_id = metadata->value(metadata->FindKey("frame_id"));
        
        // 5. 读取地图数据
        std::shared_ptr<arrow::Table> table;
        ARROW_CHECK_OK(file_reader->ReadAll(&table));
        auto data_array = std::static_pointer_cast<arrow::Int8Array>(table->column(0)->chunk(0));
        
        // 6. 转换为GridMap数据
        grid_map_.data.resize(data_array->length());
        for (int64_t i = 0; i < data_array->length(); i++) {
            grid_map_.data[i] = data_array->Value(i);
        }
        
        std::cout << "成功反序列化栅格地图: " << grid_map_.width << "x" << grid_map_.height << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "反序列化栅格地图时出错: " << e.what() << std::endl;
        return false;
    }
}

std::vector<uint8_t> AStarPlannerNode::serialize_path_to_arrow(const Path& path) const {
    // 1. 创建坐标数组
    arrow::FloatBuilder x_builder, y_builder, z_builder;
    
    for (const auto& point : path.poses) {
        ARROW_CHECK_OK(x_builder.Append(point.x));
        ARROW_CHECK_OK(y_builder.Append(point.y));
        ARROW_CHECK_OK(z_builder.Append(point.z));
    }
    
    std::shared_ptr<arrow::Array> x_array, y_array, z_array;
    ARROW_CHECK_OK(x_builder.Finish(&x_array));
    ARROW_CHECK_OK(y_builder.Finish(&y_array));
    ARROW_CHECK_OK(z_builder.Finish(&z_array));
    
    // 2. 创建元数据
    auto metadata = std::make_shared<arrow::KeyValueMetadata>();
    metadata->Append("frame_id", path.frame_id);
    metadata->Append("count", std::to_string(path.poses.size()));
    
    // 3. 创建Schema
    auto schema = arrow::schema({
        arrow::field("x", arrow::float32(), false),
        arrow::field("y", arrow::float32(), false),
        arrow::field("z", arrow::float32(), false)
    })->WithMetadata(metadata);
    
    // 4. 创建Table
    auto table = arrow::Table::Make(schema, {x_array, y_array, z_array});
    
    // 5. 序列化
    arrow::io::FixedSizeBufferWriter stream(arrow::AllocateBuffer(1024).ValueOrDie());
    arrow::ipc::IpcWriteOptions options;
    
    std::shared_ptr<arrow::ipc::RecordBatchWriter> writer;
    ARROW_CHECK_OK(arrow::ipc::MakeFileWriter(&stream, schema, options, &writer));
    ARROW_CHECK_OK(writer->WriteTable(*table));
    ARROW_CHECK_OK(writer->Close());
    
    std::shared_ptr<arrow::Buffer> buffer;
    ARROW_CHECK_OK(stream.Finish(&buffer));
    
    std::vector<uint8_t> result(buffer->data(), buffer->data() + buffer->size());
    return result;
}

void AStarPlannerNode::handle_path_request(const dora::DoraEvent& event) {
    if (!planner_) {
        std::cerr << "没有可用的地图，无法规划路径" << std::endl;
        return;
    }
    
    // 1. 解析JSON请求
    Json::Value json_request;
    Json::CharReaderBuilder reader_builder;
    std::string errors;
    std::istringstream iss(std::string(event.data.begin(), event.data.end()));
    
    if (!Json::parseFromStream(reader_builder, iss, &json_request, &errors)) {
        std::cerr << "解析路径请求JSON失败: " << errors << std::endl;
        return;
    }
    
    // 2. 提取起点和终点
    float start_x = json_request["start"]["x"].asFloat();
    float start_y = json_request["start"]["y"].asFloat();
    float goal_x = json_request["goal"]["x"].asFloat();
    float goal_y = json_request["goal"]["y"].asFloat();
    
    std::cout << "规划路径: (" << start_x << ", " << start_y << ") -> (" 
              << goal_x << ", " << goal_y << ")" << std::endl;
    
    // 3. 规划路径
    Path path = planner_->plan_path(start_x, start_y, goal_x, goal_y);
    
    if (path.poses.empty()) {
        std::cerr << "A*无法找到有效路径" << std::endl;
        return;
    }
    
    // 4. 序列化并发送路径
    auto serialized_path = serialize_path_to_arrow(path);
    node_.send_output("path", std::move(serialized_path));
    std::cout << "已找到路径，包含 " << path.poses.size() << " 个点" << std::endl;
}

void AStarPlannerNode::handle_event(const dora::Event& event) {
    if (event.as_input_event()) {
        auto input = event.as_input_event().value();
        
        if (input.id == "grid_map") {
            // 接收并处理栅格地图
            if (deserialize_grid_map_from_arrow(input.data)) {
                // 创建A*规划器（先膨胀障碍物）
                delete planner_;  // 释放之前的规划器
                planner_ = new AStarPlanner(grid_map_, inflation_radius_, heuristic_weight_, allow_diagonal_);
                planner_->inflate_obstacles();
                std::cout << "A*规划器已初始化" << std::endl;
            }
        }
        else if (input.id == "path_request") {
            // 处理路径规划请求
            handle_path_request(input);
        }
    }
}

void AStarPlannerNode::run() {
    while (true) {
        auto event = node_.next();
        if (event) {
            handle_event(event.value());
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}