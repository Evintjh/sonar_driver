cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)
project(FD_sonar_driver)

# Set C++ standard
add_compile_options(-std=c++17)
# Define USE_LINUX
add_definitions(-DUSE_LINUX)

# Include BEEX_CMAKE.txt from bx_msgs
find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

# Find Catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  nav_msgs
  bx_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)
find_package(OpenCV REQUIRED)

# Find GeographicLib
# find_package(GeographicLib REQUIRED)

# # Find LhForwardSDK library
# find_library(LHFORWARDSDK_LIBRARY
#   NAMES LhForwardSDK
#   PATHS ${PROJECT_SOURCE_DIR}/bin/
#   NO_DEFAULT_PATH
# )

find_library(LHFORWARDSDK_LIBRARY
  NAMES LhForwardSDK LhForwardSDK_X64 LhForwardSDK_X64.lib
  PATHS ${PROJECT_SOURCE_DIR}/bin
  NO_DEFAULT_PATH
)
if (NOT LHFORWARDSDK_LIBRARY)
  message(FATAL_ERROR "LhForwardSDK library not found in ${PROJECT_SOURCE_DIR}/bin")
endif()

# Define the catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rosbag
    std_msgs
    nav_msgs
    bx_msgs
    sensor_msgs
    geometry_msgs
#   DEPENDS GeographicLib
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#   ${GeographicLib_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/common
  ${PROJECT_SOURCE_DIR}/src/net
)

# Link directories
link_directories(
  ${PROJECT_SOURCE_DIR}/bin
)

# Define the executable
add_executable(sonar_processor
  src/main.cpp
  src/SonarNode.cpp
  src/Processing.cpp
  src/net/UdpReceiver.cpp
  src/common/CircleBuffer.cpp  # Add if source file exists
#   src/common/WhileThread.h  # Add if source file exists
#   src/common/sonarCommon.h  # Add if source file exists
)

# Link libraries to the executable
target_link_libraries(sonar_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#   ${GeographicLib_LIBRARIES}
  ${LHFORWARDSDK_LIBRARY}  # Link LhForwardSDK library
)

# Install targets
install(TARGETS sonar_processor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Optionally install config and launch files
# install_my_config_and_launch_files()