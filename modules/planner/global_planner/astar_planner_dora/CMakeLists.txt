project(astar_planner_dora)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

include_directories(
    include
    src
    ${DORA_INCLUDE_DIR}
)

set(SOURCES
    src/astar_planner_dora_node.cpp
    src/astar_algorithm.cpp
    src/map_loader.cpp
    src/coordinate_transform.cpp
    src/json_serialization.cpp
    src/path_smoother.cpp
)

add_executable(astar_planner_dora_node ${SOURCES})

target_link_libraries(astar_planner_dora_node
    pthread
    nlohmann_json::nlohmann_json
    ${DORA_NODE_API_LIB}
)

install(TARGETS astar_planner_dora_node RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config FILES_MATCHING PATTERN "*.json")
