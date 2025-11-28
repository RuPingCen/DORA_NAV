pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ndt_cpu/include
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/ndt_cpu/build)

add_executable(ndt_mapping  dora_node.cc
                            dora_ndt_mapper.cc
)

target_link_libraries(ndt_mapping
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${DORA_NODE_API_LIB}
  ndt_cpu
  rt
)

