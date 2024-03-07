# Copyright (c) 2023，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.0.2)
project(hobot_vio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  tf2
  tf2_ros
)

# x3|j5|x86
set(PREFIX_PATH x3)
set(SYS_ROOT ${CMAKE_SYSROOT})


message("build platform X3")
add_definitions(-DPLATFORM_X3)
set(PREFIX_PATH x3)

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    tf2
    tf2_ros
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
        opencv_world
        ${catkin_LIBRARIES}
        pthread
        dl
        ${PROJECT_SOURCE_DIR}/lib/libhorizon_vio.so
        )

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/
        DESTINATION lib/
        )

install(DIRECTORY ${PROJECT_SOURCE_DIR}/config ${PROJECT_SOURCE_DIR}/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)