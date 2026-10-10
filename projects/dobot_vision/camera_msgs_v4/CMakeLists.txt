cmake_minimum_required(VERSION 3.16)
project(camera_msgs_v4)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
        "msg/InternalParameters.msg"
        "msg/ArucoPose.msg"
        "msg/Result.msg"
        "msg/YoloPose.msg"
        )

set(srv_files
       "srv/GetAngle.srv"
       "srv/GetPose.srv"
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files} ${srv_files}
        DEPENDENCIES
        builtin_interfaces
        std_msgs
        )

ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
