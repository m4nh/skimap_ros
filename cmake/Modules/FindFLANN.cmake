## Adapted from PCL Library pointclouds.org

###############################################################################
# Find FLANN
#
# This sets the following variables:
# FLANN_FOUND - True if FLANN was found.
# FLANN_INCLUDE_DIRS - Directories containing the FLANN include files.
# FLANN_LIBRARIES - Library needed to use FLANN.
# If FLANN_USE_STATIC is specified and then look for static libraries ONLY else
# look for shared ones

if(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp_s)
  set(FLANN_DEBUG_NAME flann_cpp_s-gd)
else(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp)
  set(FLANN_DEBUG_NAME flann_cpp-gd)
endif(FLANN_USE_STATIC)

find_package(PkgConfig QUIET)
if (FLANN_FIND_VERSION)
    pkg_check_modules(PC_FLANN flann>=${FLANN_FIND_VERSION})
else(FLANN_FIND_VERSION)
    pkg_check_modules(PC_FLANN flann)
endif(FLANN_FIND_VERSION)

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
          HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" ${FLANN_ROOT}/src/cpp
          PATH_SUFFIXES include)

find_library(FLANN_LIBRARY_RELEASE
             NAMES ${FLANN_RELEASE_NAME}
             HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" ${FLANN_ROOT}/build/lib
             PATH_SUFFIXES lib)

find_library(FLANN_LIBRARY_DEBUG
             NAMES ${FLANN_DEBUG_NAME} ${FLANN_RELEASE_NAME}
             HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" ${FLANN_ROOT}/build/lib
             PATH_SUFFIXES lib)

if(NOT FLANN_LIBRARY_DEBUG)
  set(FLANN_LIBRARY_DEBUG ${FLANN_LIBRARY_RELEASE})
endif(NOT FLANN_LIBRARY_DEBUG)

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})

set(FLANN_LIBRARIES
  debug ${FLANN_LIBRARY_DEBUG}
  optimized ${FLANN_LIBRARY_RELEASE}
)

mark_as_advanced(FLANN_LIBRARY_DEBUG FLANN_LIBRARY_RELEASE FLANN_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FLANN DEFAULT_MSG FLANN_LIBRARIES FLANN_INCLUDE_DIRS)

IF(FLANN_FOUND)
  if(FLANN_USE_STATIC)
    add_definitions(-DFLANN_STATIC)
  endif(FLANN_USE_STATIC)
ENDIF(FLANN_FOUND)
