project(dora_mickrobot)

set(SERIAL_INCLUDE_DIR "/usr/local/include")
set(SERIAL_LIB_DIR "/usr/local/lib")

include_directories(
    ${SERIAL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    ${DORA_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${SERIAL_LIB_DIR})

add_executable(mickrobotx4 src/mickx4_bringup.cc)

target_link_libraries(mickrobotx4
    ${EIGEN_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    ${DORA_NODE_API_LIB}
    serial
    m rt dl pthread
)
