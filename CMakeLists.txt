cmake_minimum_required(VERSION 3.0.2)
project(panthera_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    realtime_tools
    pluginlib
    panthera_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
    src/panthera_controller.cpp
    src/speed_limiter.cpp
)
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES four_wheel_steering_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)