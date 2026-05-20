project(goal_publisher)

include_directories(
    src
    ${DORA_INCLUDE_DIR}
)

set(SOURCES
    src/goal_publisher_node.cpp
    src/udp_receiver.cpp
)

add_executable(goal_publisher_node ${SOURCES})

target_link_libraries(goal_publisher_node
    pthread
    ${DORA_NODE_API_LIB}
)

install(TARGETS goal_publisher_node RUNTIME DESTINATION bin)
