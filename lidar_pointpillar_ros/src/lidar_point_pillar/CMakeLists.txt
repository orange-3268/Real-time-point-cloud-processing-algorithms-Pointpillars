cmake_minimum_required(VERSION 2.8.3)
project(lidar_point_pillar)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)

find_package(CUDA)
message("CUDA is available!")
message("CUDA Libs: ${CUDA_LIBRARIES}")
message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
set(CUDA_AVAIL ON)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSERS NAMES nvonnxparser)
if(NVINFER AND NVPARSERS AND NVONNXPARSERS)
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  message("NVONNXPARSERS: ${NVONNXPARSERS}")
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library."
)

if(CUDNN_LIBRARY)
  message("CUDNN is available!")
  message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()


if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    pcl_ros
    autoware_msgs
  )
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

  catkin_package(
    CATKIN_DEPENDS
      roslib
      pcl_ros
      autoware_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
  set(SOURCE_FILES
    src/node.cpp
    src/point_pillars_ros.cpp
  )
  add_executable(lidar_point_pillar
    ${SOURCE_FILES}
  )
  add_dependencies(lidar_point_pillar
    ${catkin_EXPORTED_TARGETS}
  )
  cuda_add_library(gpu_point_pillars_lib
    src/pillarScatterKernels.cu
    src/postprocess_kernels.cu
    src/preprocess_kernels.cu
  )
  target_link_libraries(gpu_point_pillars_lib
    ${CUDA_LIBRARIES}
  )
  add_library(point_pillars_lib
    src/pillarScatter.cpp
    src/point_pillar.cpp
    src/postprocess.cpp
    src/preprocess.cpp
  )
  target_link_libraries(point_pillars_lib
    ${NVINFER}
    ${NVONNXPARSERS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    gpu_point_pillars_lib
  )
  target_link_libraries(lidar_point_pillar
    ${catkin_LIBRARIES}
    point_pillars_lib
  )
else()
  find_package(catkin REQUIRED)
  catkin_package()
  message("PointPillars won't be built, CUDA and/or TensorRT were not found.")
endif()
  


