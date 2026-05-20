project(livox_dora_driver)

add_definitions(-DBUILDING_DORA)

find_library(LIVOX_SDK_LIBRARY
    NAMES livox_lidar_sdk_static
    PATHS /usr/local/lib /usr/lib
    REQUIRED
)

link_directories(/usr/local/lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    ${DORA_INCLUDE_DIR}
    /usr/local/include
)

set(DORA_DRIVER_SOURCES
    src/livox_dora_driver.cpp
    src/dora_node.cpp
    src/lddc_dora.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/comm/cache_index.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/pub_handler.cpp
    src/comm/semaphore.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
)

add_executable(${PROJECT_NAME} ${DORA_DRIVER_SOURCES})

target_link_libraries(${PROJECT_NAME}
    ${LIVOX_SDK_LIBRARY}
    nlohmann_json::nlohmann_json
    Threads::Threads
    ${DORA_NODE_API_LIB}
    liblivox_lidar_sdk_static.a
    pthread
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config FILES_MATCHING PATTERN "*.json")
