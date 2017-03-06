
include(${PROJECT_SOURCE_DIR}/cmake/CMakeParseArguments.cmake)

macro(CVLAB_COMMON_CACHED_FLAGS)
  set(CVLAB_C_FLAGS_N)
  set(CVLAB_C_FLAGS_RELEASE_N)
  set(CVLAB_C_FLAGS_DEBUG_N)

  set(CVLAB_CXX_FLAGS_N)
  set(CVLAB_CXX_FLAGS_RELEASE_N)
  set(CVLAB_CXX_FLAGS_DEBUG_N)

  set(CVLAB_EXE_LINKER_FLAGS_N)
  set(CVLAB_EXE_LINKER_FLAGS_RELEASE_N)
  set(CVLAB_EXE_LINKER_FLAGS_DEBUG_N)

  set(CVLAB_SHARED_LINKER_FLAGS_N)
  set(CVLAB_SHARED_LINKER_FLAGS_RELEASE_N)
  set(CVLAB_SHARED_LINKER_FLAGS_DEBUG_N)

  if(CMAKE_COMPILER_IS_GNUCC)
    set(CVLAB_C_FLAGS_N "-Wall -Wextra")
    set(CVLAB_CXX_FLAGS_N "-fexceptions -Wall -Wextra -std=c++11")
    set(CVLAB_EXE_LINKER_FLAGS_N "-Wl,--error-unresolved-symbols -Wl,--as-needed")
    set(CVLAB_SHARED_LINKER_FLAGS_N "-Wl,--error-unresolved-symbols -Wl,--as-needed")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CVLAB_CXX_FLAGS_N "-fexceptions -Wall -Wextra -std=c++11")
    set(CVLAB_C_FLAGS_N "-Wall -Wextra")
  elseif(MSVC)
    add_definitions("-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX")
    #SET(CVLAB_CXX_FLAGS_N "/bigobj /EHsc /fp:fast /wd4800")
    SET(CVLAB_CXX_FLAGS_N "/EHsc /fp:fast /wd4800 /W4")
    SET(CVLAB_CXX_FLAGS_RELEASE_N "/GL /wd4800 /W4")
    SET(CVLAB_SHARED_LINKER_FLAGS_RELEASE_N "/LTCG")
    SET(CVLAB_EXE_LINKER_FLAGS_RELEASE_N "/LTCG")
    if(MSVC90 OR MSVC10)
      SET(CVLAB_CXX_FLAGS_N "/MP")
    endif(MSVC90 OR MSVC10)
  endif(CMAKE_COMPILER_IS_GNUCC)

  set(CVLAB_C_FLAGS "${CVLAB_C_FLAGS_N}" CACHE STRING "${PROJECT_NAME} c options")
  set(CVLAB_C_FLAGS_RELEASE "${CVLAB_C_FLAGS_RELEASE_N}" CACHE STRING "${PROJECT_NAME} c release options")
  set(CVLAB_C_FLAGS_DEBUG "${CVLAB_C_FLAGS_DEBUG_N}" CACHE STRING "${PROJECT_NAME} c debug options")

  set(CVLAB_CXX_FLAGS "${CVLAB_CXX_FLAGS_N}" CACHE STRING "${PROJECT_NAME} cxx options")
  set(CVLAB_CXX_FLAGS_RELEASE "${CVLAB_CXX_FLAGS_RELEASE_N}" CACHE STRING "${PROJECT_NAME} cxx release options")
  set(CVLAB_CXX_FLAGS_DEBUG "${CVLAB_CXX_FLAGS_DEBUG_N}" CACHE STRING "${PROJECT_NAME} cxx debug options")

  set(CVLAB_EXE_LINKER_FLAGS "${CVLAB_EXE_LINKER_FLAGS_N}" CACHE STRING "${PROJECT_NAME} exe linking options")
  set(CVLAB_EXE_LINKER_FLAGS_RELEASE "${CVLAB_EXE_LINKER_FLAGS_RELEASE_N}" CACHE STRING "${PROJECT_NAME} exe release linking options")
  set(CVLAB_EXE_LINKER_FLAGS_DEBUG "${CVLAB_EXE_LINKER_FLAGS_DEBUG_N}" CACHE STRING "${PROJECT_NAME} exe debug linking options")

  set(CVLAB_SHARED_LINKER_FLAGS "${CVLAB_SHARED_LINKER_FLAGS_N}" CACHE STRING "${PROJECT_NAME} shared lib linking options")
  set(CVLAB_SHARED_LINKER_FLAGS_RELEASE "${CVLAB_SHARED_LINKER_FLAGS_RELEASE_N}" CACHE STRING "${PROJECT_NAME} shared release lib linking options")
  set(CVLAB_SHARED_LINKER_FLAGS_DEBUG "${CVLAB_SHARED_LINKER_FLAGS_DEBUG_N}" CACHE STRING "${PROJECT_NAME} shared debug lib linking options")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CVLAB_C_FLAGS}")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CVLAB_C_FLAGS_RELEASE}")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CVLAB_C_FLAGS_DEBUG}")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CVLAB_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CVLAB_CXX_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CVLAB_CXX_FLAGS_DEBUG}")

  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CVLAB_EXE_LINKER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${CVLAB_EXE_LINKER_FLAGS_RELEASE}")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${CVLAB_EXE_LINKER_FLAGS_DEBUG}")

  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${CVLAB_SHARED_LINKER_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${CVLAB_SHARED_LINKER_FLAGS_RELEASE}")
  set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} ${CVLAB_SHARED_LINKER_FLAGS_DEBUG}")
endmacro(CVLAB_COMMON_CACHED_FLAGS)

macro(CVLAB_PRINT_FLAGS)
  message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
  message("CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
  message("CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")

  message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
  message("CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  message("CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")

  message("CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
  message("CMAKE_EXE_LINKER_FLAGS_RELEASE: ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
  message("CMAKE_EXE_LINKER_FLAGS_DEBUG: ${CMAKE_EXE_LINKER_FLAGS_DEBUG}")

  message("CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
  message("CMAKE_SHARED_LINKER_FLAGS_RELEASE: ${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
  message("CMAKE_SHARED_LINKER_FLAGS_DEBUG: ${CMAKE_SHARED_LINKER_FLAGS_DEBUG}")
endmacro(CVLAB_PRINT_FLAGS)

macro(CVLAB_SET_OUTPUT_DIR)
  set(CVLAB_COMPILER_DIR)

  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION)
    string(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONENTS ${GCC_VERSION})
    list(GET GCC_VERSION_COMPONENTS 0 GCC_MAJOR)
    list(GET GCC_VERSION_COMPONENTS 1 GCC_MINOR)

    set(CVLAB_COMPILER_DIR "gcc${GCC_MAJOR}_${GCC_MINOR}")
  endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

  if(MSVC60) # Good luck!
    set(CVLAB_COMPILER_DIR "vc6") # yes, this is correct
  elseif(MSVC70) # Good luck!
    set(CVLAB_COMPILER_DIR "vc7") # yes, this is correct
  elseif(MSVC71)
    set(CVLAB_COMPILER_DIR "vc71")
  elseif(MSVC80)
    set(CVLAB_COMPILER_DIR "vc8")
  elseif(MSVC90)
    set(CVLAB_COMPILER_DIR "vc9")
  elseif(MSVC10)
    set(CVLAB_COMPILER_DIR "vc10")
  elseif(MSVC)
    set(CVLAB_COMPILER_DIR "vc") # ??
  endif(MSVC60)

  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CVLAB_COMPILER_DIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/${CVLAB_COMPILER_DIR})
  IF(WIN32)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/${CVLAB_COMPILER_DIR})
  ELSE(WIN32)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CVLAB_COMPILER_DIR})
  ENDIF(WIN32)
endmacro(CVLAB_SET_OUTPUT_DIR)

macro(CVLAB_CHECK_FOR_SSE)
    set(CVLAB_SSE_FLAGS)

    if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION)
        if(GCC_VERSION VERSION_GREATER 4.2)
          set(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} -march=native")
          message(STATUS "Using CPU native flags for SSE optimization: ${CVLAB_SSE_FLAGS}")
        endif()
    endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

    if(NOT CVLAB_SSE_FLAGS)
      include(CheckCXXSourceRuns)
      set(CMAKE_REQUIRED_FLAGS)

      check_cxx_source_runs("
          #include <mm_malloc.h>
          int main()
          {
            void* mem = _mm_malloc (100, 16);
            return 0;
          }"
          HAVE_MM_MALLOC)

      check_cxx_source_runs("
          #include <stdlib.h>
          int main()
          {
            void* mem;
            return posix_memalign (&mem, 16, 100);
          }"
          HAVE_POSIX_MEMALIGN)

      if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
          set(CMAKE_REQUIRED_FLAGS "-msse4.1")
      endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

      check_cxx_source_runs("
          #include <smmintrin.h>
          int main()
          {
            __m128 a, b;
            float vals[4] = {1, 2, 3, 4};
            const int mask = 123;
            a = _mm_loadu_ps(vals);
            b = a;
            b = _mm_dp_ps (a, a, mask);
            _mm_storeu_ps(vals,b);
            return 0;
          }"
          HAVE_SSE4_1_EXTENSIONS)

      if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
          set(CMAKE_REQUIRED_FLAGS "-msse3")
      endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

      check_cxx_source_runs("
          #include <pmmintrin.h>
          int main()
          {
              __m128d a, b;
              double vals[2] = {0};
              a = _mm_loadu_pd(vals);
              b = _mm_hadd_pd(a,a);
              _mm_storeu_pd(vals, b);
              return 0;
          }"
          HAVE_SSE3_EXTENSIONS)

      if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
          set(CMAKE_REQUIRED_FLAGS "-msse2")
      elseif(MSVC AND NOT CMAKE_CL_64)
          set(CMAKE_REQUIRED_FLAGS "/arch:SSE2")
      endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

      check_cxx_source_runs("
          #include <emmintrin.h>
          int main()
          {
              __m128d a, b;
              double vals[2] = {0};
              a = _mm_loadu_pd(vals);
              b = _mm_add_pd(a,a);
              _mm_storeu_pd(vals,b);
              return 0;
          }"
          HAVE_SSE2_EXTENSIONS)

      if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
          set(CMAKE_REQUIRED_FLAGS "-msse")
      elseif(MSVC AND NOT CMAKE_CL_64)
          set(CMAKE_REQUIRED_FLAGS "/arch:SSE")
      endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

      check_cxx_source_runs("
          #include <xmmintrin.h>
          int main()
          {
              __m128 a, b;
              float vals[4] = {0};
              a = _mm_loadu_ps(vals);
              b = a;
              b = _mm_add_ps(a,b);
              _mm_storeu_ps(vals,b);
              return 0;
          }"
          HAVE_SSE_EXTENSIONS)

      set(CMAKE_REQUIRED_FLAGS)

      if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
          if (HAVE_SSE4_1_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} -msse4.1 -mfpmath=sse")
              message(STATUS "Found SSE4.1 extensions, using flags: ${CVLAB_SSE_FLAGS}")
          elseif(HAVE_SSE3_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} -msse3 -mfpmath=sse")
              message(STATUS "Found SSE3 extensions, using flags: ${CVLAB_SSE_FLAGS}")
          elseif(HAVE_SSE2_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} -msse2 -mfpmath=sse")
              message(STATUS "Found SSE2 extensions, using flags: ${CVLAB_SSE_FLAGS}")
          elseif(HAVE_SSE_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} -msse -mfpmath=sse")
              message(STATUS "Found SSE extensions, using flags: ${CVLAB_SSE_FLAGS}")
          else (HAVE_SSE4_1_EXTENSIONS)
              message(STATUS "No SSE extensions found")
          endif(HAVE_SSE4_1_EXTENSIONS)
      elseif (MSVC)
          if(HAVE_SSE2_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} /arch:SSE2")
              message(STATUS "Found SSE2 extensions, using flags: ${CVLAB_SSE_FLAGS}")
          elseif(HAVE_SSE_EXTENSIONS)
              SET(CVLAB_SSE_FLAGS "${CVLAB_SSE_FLAGS} /arch:SSE")
              message(STATUS "Found SSE extensions, using flags: ${CVLAB_SSE_FLAGS}")
          endif(HAVE_SSE2_EXTENSIONS)
      endif ()

    endif()

    set(CVLAB_CXX_FLAGS_RELEASE "${CVLAB_CXX_FLAGS_RELEASE} ${CVLAB_SSE_FLAGS}")
endmacro(CVLAB_CHECK_FOR_SSE)


# Add a test target
# _name the test name
# _exename the executable name
# FILES _file_names the source files
# ARGUMENTS _cmd_line the exec args
# LINK_WITH _lib_names which libs must be linked
macro(CVLAB_ADD_TEST _name _exename)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs FILES ARGUMENTS LINK_WITH)
  cmake_parse_arguments(CVLAB_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  add_executable(${_exename} ${CVLAB_ADD_TEST_FILES})
  if(NOT WIN32)
    set_target_properties(${_exename} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif(NOT WIN32)
  target_link_libraries(${_exename} ${Boost_SYSTEM_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${CVLAB_ADD_TEST_LINK_WITH})

  if(${CMAKE_VERSION} VERSION_LESS 2.8.4)
    add_test(${_name} ${_exename} ${CVLAB_ADD_TEST_ARGUMENTS})
  else(${CMAKE_VERSION} VERSION_LESS 2.8.4)
    add_test(NAME ${_name} COMMAND ${_exename} ${CVLAB_ADD_TEST_ARGUMENTS})
  endif(${CMAKE_VERSION} VERSION_LESS 2.8.4)
endmacro(CVLAB_ADD_TEST _name _exename)


# Add a library target
# _name The library name
# FILES _filenames the source files
# LINK_WITH _lib_names which libs must be linked
# DEFINITIONS _definitions
macro(CVLAB_ADD_LIBRARY _name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs FILES LINK_WITH DEFINITIONS)
  cmake_parse_arguments(CVLAB_ADD_LIBRARY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  add_library(${_name} ${CVLAB_LIB_TYPE} ${CVLAB_ADD_LIBRARY_FILES})
  target_link_libraries(${_name} ${CVLAB_ADD_LIBRARY_LINK_WITH})
  set_property(TARGET ${_name} APPEND PROPERTY COMPILE_DEFINITIONS ${CVLAB_ADD_LIBRARY_DEFINITIONS})

  #if(USE_PROJECT_FOLDERS)
    #set_target_properties(${_name} PROPERTIES FOLDER "Libs")
  #endif(USE_PROJECT_FOLDERS)
endmacro(CVLAB_ADD_LIBRARY)


# Add an executable target
# _name The executable name
# FILES _filenames the source files
# LINK_WITH _lib_names which libs must be linked
macro(CVLAB_ADD_EXECUTABLE _name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs FILES LINK_WITH)
  cmake_parse_arguments(CVLAB_ADD_EXECUTABLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  if(APPLE)
    #SET(EXECUTABLE_FLAG MACOSX_BUNDLE)
    add_executable(${_name} MACOSX_BUNDLE ${CVLAB_ADD_EXECUTABLE_FILES})
  else(APPLE)
    add_executable(${_name} ${CVLAB_ADD_EXECUTABLE_FILES})
  endif(APPLE)
  set_target_properties(${_name} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
                                            RELEASE_POSTFIX "${CMAKE_RELEASE_POSTFIX}"
                                            RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}"
                                            MINSIZEREL_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
  target_link_libraries(${_name} ${CVLAB_ADD_EXECUTABLE_LINK_WITH})

  #if(USE_PROJECT_FOLDERS)
    #set_target_properties(${_name} PROPERTIES FOLDER "Bin")
  #endif(USE_PROJECT_FOLDERS)
endmacro(CVLAB_ADD_EXECUTABLE)

macro(CVLAB_CUDA_COMPILE _out_cuda_objs)
  if(UNIX OR APPLE)
    set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
  endif(UNIX OR APPLE)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

  CUDA_COMPILE(${_out_cuda_objs} ${ARGN})
endmacro(CVLAB_CUDA_COMPILE)

macro(CVLAB_PROJECT_DEPS)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs)
  cmake_parse_arguments(CVLAB_PROJECT_DEPS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  foreach(dep ${CVLAB_PROJECT_DEPS_UNPARSED_ARGUMENTS})
    include_directories(${PROJECT_SOURCE_DIR}/${dep}/include)
  endforeach(dep ${CVLAB_PROJECT_DEPS_UNPARSED_ARGUMENTS})
endmacro(CVLAB_PROJECT_DEPS)
