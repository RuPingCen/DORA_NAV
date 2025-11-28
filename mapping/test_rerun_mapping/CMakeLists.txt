link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
)

add_executable(mapping_test Src/main.cpp)
target_link_libraries(mapping_test PRIVATE 
  ${PCL_LIBRARIES}
  ${DORA_NODE_API_LIB}
  rerun_sdk
  m
  rt
  dl
)
