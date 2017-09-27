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
    )

include_directories(${catkin_INCLUDE_DIRS})

#Services
add_service_files(
  FILES
  SkimapIntegrationService.srv
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
find_package(OpenCV REQUIRED)
if(OpenCV_INCLUDE_DIRS)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif(OpenCV_INCLUDE_DIRS)

#BOOST
find_package(Boost 1.50.0 REQUIRED COMPONENTS random timer system)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR}) # cached
endif(Boost_FOUND)


if(BUILD_SLAMDUNK)
  #KDL
  find_package(orocos_kdl REQUIRED)

  #G2O 
  find_package(G2O)
  if(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIR})
      MESSAGE(${G2O_INCLUDE_DIR})
  else(G2O_FOUND)
    set(G2O_ROOT "" CACHE PATH "Installation root for G2O")
    MESSAGE(FATAL_ERROR "G2O NOT FOUND")
  endif(G2O_FOUND)

  #FLANN
  find_package(FLANN REQUIRED)
  if(FLANN_FOUND)
    include_directories(${FLANN_INCLUDE_DIRS})
  else(FLANN_FOUND)
    set(FLANN_ROOT "" CACHE PATH "Installation root for FLANN")
  endif(FLANN_FOUND)
  
  #PCL
  find_package(PCL REQUIRED )
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  
endif(BUILD_SLAMDUNK)


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
# SLAMDUNK LIBRARY
#########################


if(BUILD_SLAMDUNK)
    include_directories(
      include/
    )
    add_library(slamdunk
           src/slamdunk/data.cpp
           src/slamdunk/edge_se3_xyzpair.cpp
           src/slamdunk/feature_tracker.cpp
           src/slamdunk/graph_backend.cpp
           src/slamdunk/graph_utils.cpp
           src/slamdunk/feature_matcher.cpp
           src/slamdunk/slam_dunk.cpp
           src/slamdunk/transformation_estimation.cpp
           src/slamdunk/internal_timers.hpp
           src/slamdunk_extension/SlamDunkScene.cpp
           src/slamdunk_extension/Utility.cpp
    )
    target_link_libraries (slamdunk 
        ${OpenCV_LIBRARIES}
        ${Boost_TIMER_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${FLANN_LIBRARIES}
        ${G2O_STUFF_LIBRARY}
        ${G2O_CORE_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_TYPES_SLAM3D}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES} 
        ${orocos_kdl_LIBRARIES}
    )
endif(BUILD_SLAMDUNK)


#########################
# SKIMAP LIBRARY
#########################
include_directories(
  include/
)

#########################
# GRID EXPERIMENTS
#########################

include_directories(/home/daniele/Downloads/ann_1.1.2/include)
add_executable(grid2d src/nodes/experiments/grid2d.cpp)
target_link_libraries(grid2d /home/daniele/Downloads/ann_1.1.2/lib/libANN.a ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


#########################
# NODES
#########################



add_executable(skimap_live src/nodes/skimap_live.cpp)
target_link_libraries(skimap_live ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(skimap_map_service src/nodes/skimap_map_service.cpp)
target_link_libraries(skimap_map_service ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(skimap_map_service skimap_ros_generate_messages_cpp)

if(BUILD_SLAMDUNK)
    #SLAM TRACKER
    add_executable(slamdunk_tracker  src/nodes/slamdunk_tracker.cpp)
    target_link_libraries(slamdunk_tracker ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} slamdunk   ${PCL_LIBRARIES})

    #SLAM TRACKER
    add_executable(testing  src/nodes/test.cpp)
    target_link_libraries(testing ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} slamdunk   ${PCL_LIBRARIES})
endif(BUILD_SLAMDUNK)

if(EXPERIMENTAL)
    #SLAM TRACKER
    add_executable(skimap_pose_filtering  src/nodes/skimap_pose_filtering.cpp)
    target_link_libraries(skimap_pose_filtering ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

endif(EXPERIMENTAL)

if(BUILD_TUTORIALS)
    #INTEGRATION OF RANDOM POINTS
    add_executable(integration_of_random_points  src/nodes/tutorials/integration_of_random_points.cpp)
    target_link_libraries(integration_of_random_points ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
endif(BUILD_TUTORIALS)



###############################################################################
# INSTALLATION
###############################################################################

if(BUILD_SLAMDUNK)
    ## Mark executables and/or libraries for installation
    install(TARGETS 
      slamdunk
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/slamdunk DESTINATION include/ FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN ".svn" EXCLUDE)
    install(DIRECTORY include/slamdunk_extension DESTINATION include/ FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN ".svn" EXCLUDE)
endif(BUILD_SLAMDUNK)

install(DIRECTORY include/skimap
  DESTINATION include
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

