cmake_minimum_required(VERSION 3.8)
project(agv_app_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agv_app_msgs REQUIRED)
find_package(agv_service REQUIRED)


# ---------------------------------------------------------
# 1. Build the Shared Library (Core Logic)
# ---------------------------------------------------------
# colcon build --cmake-args -DBUILD_SHARED_LIBS=ON
add_library(${PROJECT_NAME}_lib SHARED
  src/agv_app_server.cpp
  src/instant_action.cpp
)

# 关键：添加头文件搜索路径
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 关键：使用 ament_target_dependencies 来处理 ROS 依赖
# 这会自动处理包括 fmt 在内的所有传递依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  agv_app_msgs
  agv_service
)

# ---------------------------------------------------------
# 2. Build the Executable Node
# ---------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/main.cpp)

# 链接核心库
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
    fmt::fmt
)

# ---------------------------------------------------------
# 3. Install Rules
# ---------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  DESTINATION /home/byd/node/agv_app_server/bin/)

# 把源码目录中 include/ 文件夹下的所有内容（通常是 .hpp 头文件），递归地复制到安装目录的 include/ 路径下（即 ${CMAKE_INSTALL_PREFIX}/include）
install(DIRECTORY include/
  DESTINATION include
)

ament_package()