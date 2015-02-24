# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(estimator_node)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  puppeteer_msgs
  tf
  tf_conversions
  geometry_msgs
  nav_msgs
  angles
  cmake_modules)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )



# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package()
    # DEPENDS roscpp puppeteer_msgs tf tf_conversions geometry_msgs nav_msgs angles
    # CATKIN_DEPENDS # TODO
    # INCLUDE_DIRS # TODO include
    # LIBRARIES # TODO
#)


## add eigen functionality
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
# add logging functionality
find_library(LOG4CXX_LIBRARY log4cxx)

add_executable(estimator_node src/estimator_node.cpp)
add_executable(robot_estimator_3d src/robot_estimator_3d.cpp)
add_executable(robot_estimator_kinect_3d src/robot_estimator_kinect_3d.cpp)
add_executable(kalman_estimator src/kalman_estimator.cpp)


add_dependencies(estimator_node ${catkin_EXPORTED_TARGETS}})
add_dependencies(robot_estimator_3d ${catkin_EXPORTED_TARGETS}})
add_dependencies(robot_estimator_kinect_3d ${catkin_EXPORTED_TARGETS}})
add_dependencies(kalman_estimator ${catkin_EXPORTED_TARGETS}})


target_link_libraries(estimator_node ${catkin_LIBRARIES})
target_link_libraries(robot_estimator_3d ${catkin_LIBRARIES})
target_link_libraries(robot_estimator_kinect_3d ${catkin_LIBRARIES}) 
target_link_libraries(kalman_estimator ${catkin_LIBRARIES})


