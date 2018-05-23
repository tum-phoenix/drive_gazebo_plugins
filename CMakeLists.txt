cmake_minimum_required(VERSION 2.8.3)
project(drive_gazebo_plugins)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge gazebo_plugins gazebo_ros)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

find_package(jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES drive_camera_label_plugin
    CATKIN_DEPENDS roscpp cv_bridge gazebo_plugins gazebo_ros
    DEPENDS gazebo OpenCV
)

###########
## Build ##
###########

add_library(drive_sign_label_plugin src/sign_label_plugin.cpp)
target_link_libraries(drive_sign_label_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} CameraPlugin jsoncpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS drive_sign_label_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
    PATTERN "*.svn" EXCLUDE)

install(DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.world" PATTERN "*.sdf"
    PATTERN "*.svn" PATTERN "materials" EXCLUDE)

install(DIRECTORY worlds/materials
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.png" PATTERN "*.material"
    PATTERN "*.svn" EXCLUDE)

install(FILES
    worlds/car-cc2017.dae
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/draw_bounding_boxes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

