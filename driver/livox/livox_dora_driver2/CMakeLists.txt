# cmake_minimum_required(VERSION 2.8.3)
# if(POLICY CMP0048)
#   cmake_policy(SET CMP0048 NEW)
# endif()

project(livox_dora_driver VERSION 1.0.0)

# message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})
# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

# set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# # NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
# set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
# # set(LIVOX_LIDAR_SDK_LIBRARY "/home/nvidia/SSD/indoorslam/0326_yewai/livox/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_static.a")
# message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")

find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
  )
  
# find_package(catkin REQUIRED COMPONENTS)
find_package(PkgConfig)

pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)


#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## make sure the livox_lidar_sdk_static library is installed
find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)


#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_node
  ""
)

#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_node
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------


#---------------------------------------------------------------------------------------
# source file
#---------------------------------------------------------------------------------------
target_sources(${PROJECT_NAME}_node
  PRIVATE
  src/lds.cpp
  src/lds_lidar.cpp
  src/lddc_dora.cpp
  src/livox_dora_driver2.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Livox-SDK2/include)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  # ${CMAKE_CURRENT_SOURCE_DIR}/../Livox-SDK2/include
  3rdparty
  src
)

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
  pcap
)
