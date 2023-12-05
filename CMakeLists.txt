project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

include(FindPackageHandleStandardArgs)
  set(GLOG_ROOT_DIR "" CACHE PATH "Folder contains Google glog")
  if(NOT WIN32)
      find_path(GLOG_INCLUDE_DIR glog/logging.h
          PATHS ${GLOG_ROOT_DIR})
  endif()

  find_library(GLOG_LIBRARY glog
      PATHS ${GLOG_ROOT_DIR}
      PATH_SUFFIXES lib lib64)

  find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

  if(GLOG_FOUND)
    message(STATUS
        "Caffe2: Found glog (include: ${GLOG_INCLUDE_DIR}, "
        "library: ${GLOG_LIBRARY})")
    add_library(glog::glog UNKNOWN IMPORTED)
    set_property(
        TARGET glog::glog PROPERTY IMPORTED_LOCATION ${GLOG_LIBRARY})
    set_property(
        TARGET glog::glog PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        ${GLOG_INCLUDE_DIR})
  endif()

# After above, we should have the glog::glog target now.
if(NOT TARGET glog::glog)
  message(WARNING
      "glog cannot be found!")
endif()

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

# include(src/Planner_lib.h)
# include_directories(/home/steve/git/CarND-Path-Planning-Project/src)
# add_library(planner_lib src/Planner_lib.cpp)


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS glog)
