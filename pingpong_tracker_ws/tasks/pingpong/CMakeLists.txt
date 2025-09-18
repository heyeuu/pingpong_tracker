cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# add_subdirectory(planner/tinympc)

add_library(pingpong OBJECT 
    # multithread/commandgener.cpp
    # multithread/mt_detector.cpp
    # planner/planner.cpp
)

target_link_libraries(pingpong  io openvino::runtime tinympcstatic)