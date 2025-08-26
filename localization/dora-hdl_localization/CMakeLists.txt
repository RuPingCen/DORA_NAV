cmake_minimum_required(VERSION 2.8.3)

project(hdl_localization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(NDT_OMP_LIB_PATH ${CMAKE_SOURCE_DIR}/3rdparty/hdl_ndt_omp/build/devel/lib)
link_directories(${NDT_OMP_LIB_PATH})


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator
  ./include
  ./3rdparty/hdl_ndt_omp/include
)


add_executable(${PROJECT_NAME} src/dora_hdl_node.cpp src/pose_estimator.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  ndt_omp
  rt
)
