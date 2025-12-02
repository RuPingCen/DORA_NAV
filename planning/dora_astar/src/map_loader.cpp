#include "map_loader.h"
#include <arrow/ipc/writer.h>
#include <arrow/io/memory.h>
#include <arrow/table.h>
#include <arrow/array.h>
#include <thread>
#include <chrono>

MapLoaderNode::MapLoaderNode() {
    node_ = dora::Node::init().value();
    
    // 从配置中获取参数
    auto config = node_.config().value();
    map_type_ = config["map_type"].as_string().value();
    map_path_ = config["map_path"].as_string().value();
}

bool MapLoaderNode::load_grid_map() {
    try {
        // 1. 加载YAML元数据
        YAML::Node yaml_file = YAML::LoadFile(map_path_ + ".yaml");
        float resolution = yaml_file["resolution"].as<float>();
        std::vector<float> origin = yaml_file["origin"].as<std::vector<float>>();
        int negate = yaml_file["negate"].as<int>();
        float occupied_thresh = yaml_file["occupied_thresh"].as<float>();
        float free_thresh = yaml_file["free_thresh"].as<float>();
        
        // 2. 加载地图图片
        cv::Mat map_image = cv::imread(map_path_ + ".png", cv::IMREAD_GRAYSCALE);
        if (map_image.empty()) {
            std::cerr << "无法加载地图图片: " << map_path_ + ".png" << std::endl;
            return false;
        }
        
        // 3. 转换图像到占用值
        grid_map_.data.resize(map_image.rows * map_image.cols);
        grid_map_.width = map_image.cols;
        grid_map_.height = map_image.rows;
        grid_map_.resolution = resolution;
        
        // 设置原点
        if (origin.size() >= 3) {
            grid_map_.origin[0] = origin[0];
            grid_map_.origin[1] = origin[1];
            grid_map_.origin[2] = origin[2];
        } else {
            grid_map_.origin[0] = 0.0f;
            grid_map_.origin[1] = 0.0f;
            grid_map_.origin[2] = 0.0f;
        }
        
        grid_map_.frame_id = "map";
        
        // 转换像素值到ROS占用值
        for (int y = 0; y < map_image.rows; y++) {
            for (int x = 0; x < map_image.cols; x++) {
                uint8_t pixel_value = map_image.at<uint8_t>(y, x);
                float normalized = static_cast<float>(pixel_value) / 255.0f;
                
                // 应用negate设置
                float occupancy = negate == 0 ? (1.0f - normalized) : normalized;
                
                // 转换为ROS占用值
                int8_t occupancy_value;
                if (occupancy > occupied_thresh) {
                    occupancy_value = 100;  // 障碍
                } else if (occupancy < free_thresh) {
                    occupancy_value = 0;    // 空闲
                } else {
                    occupancy_value = -1;   // 未知
                }
                
                // 存储在y-major顺序中
                grid_map_.data[y * map_image.cols + x] = occupancy_value;
            }
        }
        
        std::cout << "成功加载栅格地图: " << map_image.cols << "x" << map_image.rows << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "加载栅格地图时出错: " << e.what() << std::endl;
        return false;
    }
}

/**********************************************************

pointcloud:
  resolution: 0.05          # 栅格分辨率(米/格)
  min_x: -10.0              # 地图X范围最小值
  max_x: 10.0               # 地图X范围最大值
  min_y: -10.0              # 地图Y范围最小值
  max_y: 10.0               # 地图Y范围最大值
  min_height: 0.1           # 考虑的最小高度(过滤地面点)
  max_height: 1.8           # 考虑的最大高度(过滤天花板点)
  min_points_per_cell: 3    # 判定为障碍物所需的最小点数
  occupancy_threshold: 0.65 # 占用阈值(0-1)
  free_threshold: 0.196     # 空闲阈值(0-1)

**********************************************************/

bool MapLoaderNode::load_pointcloud_map() {
    try {
        // 1. 加载PCD文件
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_ + ".pcd", *cloud) == -1) {
            std::cerr << "无法加载PCD文件: " << map_path_ + ".pcd" << std::endl;
            return false;
        }
        
        std::cout << "成功加载点云地图，包含 " << cloud->size() << " 个点" << std::endl;
        
        // 2. 从配置获取投影参数
        auto config = node_.config().value();
        float min_height = config["pointcloud"]["min_height"].as_float().value_or(0.0f);
        float max_height = config["pointcloud"]["max_height"].as_float().value_or(2.0f);
        float resolution = config["pointcloud"]["resolution"].as_float().value_or(0.05f);
        float min_x = config["pointcloud"]["min_x"].as_float().value_or(-10.0f);
        float max_x = config["pointcloud"]["max_x"].as_float().value_or(10.0f);
        float min_y = config["pointcloud"]["min_y"].as_float().value_or(-10.0f);
        float max_y = config["pointcloud"]["max_y"].as_float().value_or(10.0f);
        int min_points_per_cell = config["pointcloud"]["min_points_per_cell"].as_int().value_or(3);
        
        // 3. 创建栅格地图参数
        grid_map_.resolution = resolution;
        grid_map_.width = static_cast<size_t>((max_x - min_x) / resolution);
        grid_map_.height = static_cast<size_t>((max_y - min_y) / resolution);
        grid_map_.origin[0] = min_x;
        grid_map_.origin[1] = min_y;
        grid_map_.origin[2] = 0.0f;
        grid_map_.frame_id = "map";
        grid_map_.data.resize(grid_map_.width * grid_map_.height, 0); // 初始化为空闲
        
        // 4. 创建二维占用计数器
        std::vector<std::vector<int>> occupancy_count(
            grid_map_.height, 
            std::vector<int>(grid_map_.width, 0)
        );
        
        // 5. 遍历点云，投影到2D平面
        for (const auto& point : cloud->points) {
            // 检查NaN值
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
                
            // 高度过滤：只考虑特定高度范围内的点（例如，机器人可能碰撞的高度）
            if (point.z < min_height || point.z > max_height)
                continue;
                
            // 检查是否在XY范围内
            if (point.x < min_x || point.x > max_x || 
                point.y < min_y || point.y > max_y)
                continue;
                
            // 计算栅格坐标
            int grid_x = static_cast<int>((point.x - min_x) / resolution);
            int grid_y = static_cast<int>((point.y - min_y) / resolution);
            
            // 边界检查
            if (grid_x < 0 || grid_x >= static_cast<int>(grid_map_.width) ||
                grid_y < 0 || grid_y >= static_cast<int>(grid_map_.height))
                continue;
                
            // 增加该栅格的占用计数
            occupancy_count[grid_y][grid_x]++;
        }
        
        // 6. 根据占用计数确定栅格状态
        for (size_t y = 0; y < grid_map_.height; y++) {
            for (size_t x = 0; x < grid_map_.width; x++) {
                if (occupancy_count[y][x] >= min_points_per_cell) {
                    // 足够多的点表示障碍物
                    grid_map_.data[y * grid_map_.width + x] = 100;
                } else if (occupancy_count[y][x] > 0) {
                    // 少量点表示未知区域
                    grid_map_.data[y * grid_map_.width + x] = -1;
                } else {
                    // 没有点表示空闲空间
                    grid_map_.data[y * grid_map_.width + x] = 0;
                }
            }
        }
        
        std::cout << "点云已成功转换为 " << grid_map_.width << "x" << grid_map_.height 
                  << " 栅格地图" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "加载点云地图时出错: " << e.what() << std::endl;
        return false;
    }
}

std::vector<uint8_t> MapLoaderNode::serialize_grid_map_to_arrow() const {
    // 1. 创建Arrow数组
    arrow::Int8Builder data_builder;
    ARROW_CHECK_OK(data_builder.AppendValues(
        reinterpret_cast<const int8_t*>(grid_map_.data.data()), 
        grid_map_.data.size()));
    
    std::shared_ptr<arrow::Array> data_array;
    ARROW_CHECK_OK(data_builder.Finish(&data_array));
    
    // 2. 创建元数据
    auto metadata = std::make_shared<arrow::KeyValueMetadata>();
    metadata->Append("width", std::to_string(grid_map_.width));
    metadata->Append("height", std::to_string(grid_map_.height));
    metadata->Append("resolution", std::to_string(grid_map_.resolution));
    metadata->Append("origin_x", std::to_string(grid_map_.origin[0]));
    metadata->Append("origin_y", std::to_string(grid_map_.origin[1]));
    metadata->Append("origin_z", std::to_string(grid_map_.origin[2]));
    metadata->Append("frame_id", grid_map_.frame_id);
    
    // 3. 创建Schema
    auto schema = arrow::schema({
        arrow::field("data", arrow::int8(), false)
    })->WithMetadata(metadata);
    
    // 4. 创建Table
    auto table = arrow::Table::Make(schema, {data_array});
    
    // 5. 序列化到缓冲区
    arrow::io::FixedSizeBufferWriter stream(arrow::AllocateBuffer(1024*1024).ValueOrDie());
    arrow::ipc::IpcWriteOptions options;
    options.emit_dictionary_deltas = true;
    
    std::shared_ptr<arrow::ipc::RecordBatchWriter> writer;
    ARROW_CHECK_OK(arrow::ipc::MakeFileWriter(&stream, schema, options, &writer));
    ARROW_CHECK_OK(writer->WriteTable(*table));
    ARROW_CHECK_OK(writer->Close());
    
    // 6. 获取序列化数据
    std::shared_ptr<arrow::Buffer> buffer;
    ARROW_CHECK_OK(stream.Finish(&buffer));
    
    // 转换为std::vector
    std::vector<uint8_t> result(buffer->data(), buffer->data() + buffer->size());
    return result;
}

void MapLoaderNode::handle_event(const dora::Event& event) {
    if (event.as_input_event()) {
        auto input = event.as_input_event().value();
        if (input.id == "tick") {
            if (map_type_ == "grid") {
                if (load_grid_map()) {
                    auto serialized_map = serialize_grid_map_to_arrow();
                    node_.send_output("grid_map", std::move(serialized_map));
                    std::cout << "已发送栅格地图数据" << std::endl;
                }
            } else if (map_type_ == "pointcloud") {
                if (load_pointcloud_map()) {
                    // 点云序列化实现类似，此处略
                    // auto serialized_cloud = serialize_pointcloud_to_arrow();
                    // node_.send_output("pointcloud_map", std::move(serialized_cloud));
                    std::cout << "点云地图加载完成（序列化逻辑需实现）" << std::endl;
                }
            }
        }
    }
}

void MapLoaderNode::run() {
    while (true) {
        auto event = node_.next();
        if (event) {
            handle_event(event.value());
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}