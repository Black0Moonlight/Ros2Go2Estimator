cmake_minimum_required(VERSION 3.5)
project(fusion_estimator)

set(CMAKE_BUILD_TYPE Debug)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  unitree_legged_msgs
  urdf
)

# 生成消息接口
add_message_files(
  FILES
  FusionEstimatorTest.msg
)

# 生成服务接口（如果有）
# add_service_files(
#   FILES
#   FusionEstimatorService.srv
# )

# 生成依赖的消息代码
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 获取 src/Estimator 目录下的所有 .cpp 文件
file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# 创建可执行文件
add_executable(fusion_estimator_node ${ESTIMATOR_SOURCES})

# 包含目录
target_include_directories(fusion_estimator_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接库
target_link_libraries(fusion_estimator_node
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS fusion_estimator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 声明依赖
catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs 
  nav_msgs 
  sensor_msgs 
  tf
  unitree_legged_msgs
)


