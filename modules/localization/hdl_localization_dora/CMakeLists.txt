project(hdl_localization_dora)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()

include_directories(
    include
    src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${DORA_INCLUDE_DIR}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(SOURCES
    src/hdl_localization_dora_node.cpp
    src/json_serialization.cpp
    src/config_manager.cpp
    src/pose_estimator.cpp
)

add_executable(hdl_localization_dora_node ${SOURCES})

target_link_libraries(hdl_localization_dora_node
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    OpenMP::OpenMP_CXX
    pthread
    ${DORA_NODE_API_LIB}
)

# ndt_omp（可选）
find_library(NDT_OMP_LIBRARY
    NAMES ndt_omp
    PATHS /usr/local/lib /usr/lib
    NO_DEFAULT_PATH
)
if(NDT_OMP_LIBRARY)
    message(STATUS "Found ndt_omp: ${NDT_OMP_LIBRARY}")
    target_link_libraries(hdl_localization_dora_node ${NDT_OMP_LIBRARY})
    find_path(NDT_OMP_INCLUDE_DIR NAMES pclomp/ndt_omp.h PATHS /usr/local/include /usr/include NO_DEFAULT_PATH)
    if(NDT_OMP_INCLUDE_DIR)
        include_directories(${NDT_OMP_INCLUDE_DIR})
    endif()
else()
    message(WARNING "ndt_omp library not found. Please install it or specify the path.")
endif()

# fast_gicp（可选）
find_library(FAST_GICP_LIBRARY
    NAMES fast_gicp
    PATHS /usr/local/lib /usr/lib
    NO_DEFAULT_PATH
)
if(FAST_GICP_LIBRARY)
    message(STATUS "Found fast_gicp: ${FAST_GICP_LIBRARY}")
    target_link_libraries(hdl_localization_dora_node ${FAST_GICP_LIBRARY})
    find_path(FAST_GICP_INCLUDE_DIR NAMES fast_gicp/gicp/fast_gicp.hpp PATHS /usr/local/include /usr/include NO_DEFAULT_PATH)
    if(FAST_GICP_INCLUDE_DIR)
        include_directories(${FAST_GICP_INCLUDE_DIR})
    endif()
else()
    message(STATUS "fast_gicp library not found (optional)")
endif()

install(TARGETS hdl_localization_dora_node RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config FILES_MATCHING PATTERN "*.json")
