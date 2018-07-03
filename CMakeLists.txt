cmake_minimum_required(VERSION 2.8.3)
project(skimap_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## OPTIONS
option(BUILD_SLAMDUNK "Build Slamdunk Library" OFF)
option(EXPERIMENTAL "Experimental features" OFF)
option(BUILD_TUTORIALS "Build Tutorials Nodes" ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
    cv_bridge
    image_transport
    pcl_ros
    nav_msgs
    std_msgs
    geometry_msgs
    message_generation
    tf
    tf_conversions
    eigen_conversions
    )

include_directories(${catkin_INCLUDE_DIRS})

#Services
add_service_files(
  FILES
  SkimapIntegrationService.srv
  SemanticService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# useful macros
include(CMakeDependentOption)

# Find modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules ${CMAKE_MODULE_PATH})


# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
else(OPENMP_FOUND)
  message (STATUS "Not found OpenMP")
endif(OPENMP_FOUND)

#Eigen3
find_package(Eigen3 REQUIRED)

#OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc flann highgui )
if(OpenCV_INCLUDE_DIRS)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif(OpenCV_INCLUDE_DIRS)

#BOOST
find_package(Boost 1.50.0 REQUIRED COMPONENTS random timer system python)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR}) # cached
endif(Boost_FOUND)

#CNPY
set(CNPY_LIBRARY /home/daniele/work/workspace_cpp/cnpy/build/libcnpy.a)



################################################################################
################################################################################
################################################################################

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/skimap_ros_void_library.cpp
)


catkin_package(
   INCLUDE_DIRS include/
   LIBRARIES ${PROJECT_NAME}
)




#########################
# SKIMAP LIBRARY
#########################
include_directories(
  include/
)

  

#########################
# NODES
#########################



add_executable(skimap_live src/nodes/skimap_live.cpp)
target_link_libraries(skimap_live ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(skimap_map_service src/nodes/skimap_map_service.cpp)
target_link_libraries(skimap_map_service ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(skimap_map_service skimap_ros_generate_messages_cpp)

add_executable(skimap_semantic_map_service src/nodes/skimap_semantic_map_service.cpp)
target_link_libraries(skimap_semantic_map_service ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(skimap_semantic_map_service skimap_ros_generate_messages_cpp)

add_executable(skimap_semantic_map_service_playground src/nodes/skimap_semantic_map_service_playground.cpp)
target_link_libraries(skimap_semantic_map_service_playground ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(skimap_semantic_map_service_playground skimap_ros_generate_messages_cpp)




if(BUILD_TUTORIALS)
    #INTEGRATION OF RANDOM POINTS
    add_executable(integration_of_random_points  src/nodes/tutorials/integration_of_random_points.cpp)
    target_link_libraries(integration_of_random_points ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
endif(BUILD_TUTORIALS)


#########################
# EXPERIMENTS
#########################

# include_directories(/home/daniele/work/workspace_cpp/ann_1.1.2/include)
# ADD_LIBRARY(ann STATIC IMPORTED)
# SET_TARGET_PROPERTIES(ann PROPERTIES IMPORTED_LOCATION /home/daniele/work/workspace_cpp/ann_1.1.2/lib/libANN.a)

add_executable(kd_test src/nodes/experiments/kd_test.cpp)
# target_link_libraries(kd_test ann)
target_link_libraries(kd_test ${OpenCV_LIBRARIES} boost_system)

# add_dependencies(kd_test skimap_ros_generate_messages_cpp)

# include_directories(/home/daniele/work/workspace_cpp/octomap/octomap/include)
# ADD_LIBRARY(octomap SHARED IMPORTED)
# SET_TARGET_PROPERTIES(octomap PROPERTIES IMPORTED_LOCATION /home/daniele/work/workspace_cpp/octomap/octomap/lib/liboctomap.so )
# ADD_LIBRARY(octomath SHARED IMPORTED)
# SET_TARGET_PROPERTIES(octomath PROPERTIES IMPORTED_LOCATION /home/daniele/work/workspace_cpp/octomap/octomap/lib/liboctomath.so )


# add_executable(comparison_test src/nodes/experiments/comparisons/comparison_test.cpp)
# target_link_libraries(comparison_test octomap octomath ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})



################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
#
# SITECO EXPERIMENTS
#
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################


include_directories(/home/daniele/work/workspace_cpp/SitecoLadybugTools/SitecoLadybugTools/include)
add_executable(raycasting_test src/nodes/experiments/raycasting/raycasting_test.cpp)
target_link_libraries(raycasting_test ${CNPY_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(map_visualizer src/nodes/experiments/raycasting/map_visualizer.cpp)
target_link_libraries(map_visualizer ${CNPY_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )


# add_executable(raycasting_test_click src/nodes/experiments/raycasting/raycasting_test_click.cpp)
# target_link_libraries(raycasting_test_click ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

###############################################################################
# INSTALLATION
###############################################################################

install(DIRECTORY include/skimap
  DESTINATION include
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

