project(rerun_visualizer)

# Find Rerun SDK
find_package(rerun_sdk REQUIRED)

# Find PCL (for reading PCD map files)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${DORA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

set(VISUALIZER_SOURCES
    src/rerun_visualizer.cpp
)

add_executable(${PROJECT_NAME} ${VISUALIZER_SOURCES})

target_link_libraries(${PROJECT_NAME}
    nlohmann_json::nlohmann_json
    rerun_sdk
    ${PCL_LIBRARIES}
    Threads::Threads
    ${DORA_NODE_API_LIB}
    pthread
)

# Both rerun_sdk and dora_node_api_c are Rust static libraries that each bundle
# their own copy of the Rust std (rust_eh_personality, etc.).
# --allow-multiple-definition tells the linker to keep the first definition
# and silently ignore duplicates, resolving the symbol conflict.
target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,--allow-multiple-definition
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
