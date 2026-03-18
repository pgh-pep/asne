cmake_minimum_required(VERSION 3.8)
project(asne_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

# Python Nodes:
install(PROGRAMS
    ${PROJECT_NAME}/rc_decoder_node.py
    ${PROJECT_NAME}/waypoint_manager_node.py
    ${PROJECT_NAME}/serial_comm_node.py
    ${PROJECT_NAME}/system_controller.py
    ${PROJECT_NAME}/gps_failsafe_node.py
    ${PROJECT_NAME}/servo_control_node.py
    ${PROJECT_NAME}/manual_control_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# Cpp Nodes
# ament_auto_add_executable(
# cpp_exe_name
# ${PROJECT_NAME}/cpp_node.cpp
# ${PROJECT_NAME}/other_cpp_file.cpp
# ...
# )

# target_link_libraries(cpp_exe_name non_ROS_dependencies)
install(TARGETS

    # cpp_exe_name
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE launch config rviz
)