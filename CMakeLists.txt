cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_planning_srv)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_msgs
  roscpp
  urdf
  tf_conversions
)

set(THIS_PACKAGE_INCLUDE_DIRS
    srv_kinematics_plugin/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
		    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(srv_kinematics_plugin)

install(
  FILES
    srv_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
