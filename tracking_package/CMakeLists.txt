cmake_minimum_required(VERSION 2.8.2)
project(tracking_package)

# 找到catkin宏和包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
)
find_package(Eigen3)

# 包含从find_package()调用返回的catkin宏
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  # 定义包的依赖项
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
  DEPENDS system_lib
)

# 定义源文件变量
set(SOURCE_FILES src/tracker.cpp)

# 将可执行文件添加到项目中
add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

# 链接catkin库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(pure_pursuit src/pure_pursuit.cpp)

target_link_libraries(pure_pursuit
  ${catkin_LIBRARIES}
)

# 附加依赖项，例如OpenCV
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 如果你有自定义的库，例如THUTracker，也需要在这里链接
# 添加 THUTracker 库
add_library(THUTracker
  src/thutracker.cpp
  src/fhog.cpp
)
target_link_libraries(${PROJECT_NAME}_node THUTracker ${PCL_LIBRARIES})

# 附加安装规则，例如可执行文件和launch文件
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
