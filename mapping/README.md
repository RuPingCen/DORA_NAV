#  1 NDT Mapping
修改于 source: https://github.com/Kin-Zhang/simple_ndt_slam
## 1.1 ndt_mapping 文件清单
**dora_node.cc**  ndt建图dora节点
**dora_ndt_mapper.cc**   ndt建图类
**ndt_cpu**   ndt算法实现
**ndt_mapping_config.yml**   ndt建图参数
**dataflow_pcap.yml**   Dora数据流文件

## 1.2 依赖
```bash
# pcl库
sudo apt-get installl libpcl-dev
# eigen库
sudo apt install libeigen3-dev
```
yaml-cpp: https://github.com/jbeder/yaml-cpp

## 1.3 编译
需要先编译ndt_cpu
```bash
cd ndt_cpu
mkdir build && cd build
cmake ..
cmake --build .
```
编译dora ndt建图节点
```bash
cd ../..
# 在编译之前需要先修改CMakeLists.txt，修改
# include_directories()
# target_link_libraries()
# 中的有关dora的路径
mkdir build && cd build
cmake ..
cmake --build .
```

## 1.4 建图
```bash
# 在ndt_mapping路径下
dora up
# 在启动前，需要把dataflow_pcap.yml中，节点的source
dora start dataflow_pcap.yml --name test
```
建好的地图会保存在 **ndt_mapping_config.yml** map_save_path这一路径下

 