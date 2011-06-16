#########################################################
### CONFIG ##############################################
#########################################################
# 1, if SiftGPU should be used. For additional information
# see README
set(USE_SIFT_GPU 	0)			
set(USE_GICP_BIN	0)
set(USE_GICP_CODE	0)
#########################################################
#########################################################
#########################################################


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

#is used for compiling the libraries 
set(USE_GICP 0)
IF (${USE_GICP_BIN} OR ${USE_GICP_CODE})
	set(USE_GICP 1)
ENDIF (${USE_GICP_BIN} OR ${USE_GICP_CODE})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library) 
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
##############################################################################
# Required 3rd party
##############################################################################
# http://qtnode.net/wiki/Qt4_with_cmake

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})
# This is necessary as all ui files etc will get dumped in the bottom of the
# binary directory. 
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR} gicp/ann_1.1.1/include/)

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})
SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})
#SET(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE TRUE)
#SiftGPU
IF (${USE_SIFT_GPU})
	include_directories(${CMAKE_CURRENT_BINARY_DIR} external/siftgpu/src)
	EXEC_PROGRAM(make ../external/siftgpu/linux ARGS all)
	FILE(COPY external/siftgpu/linux/bin/libsiftgpu.so DESTINATION ../lib)
ENDIF (${USE_SIFT_GPU})

#GICP
IF (${USE_GICP})
	EXEC_PROGRAM(make ../gicp/ann_1.1.1/ ARGS linux-g++)
	FILE(COPY gicp/ann_1.1.1/lib/libANN.a DESTINATION ../lib)
ENDIF (${USE_GICP})

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/openni_listener.h src/qtcv.h src/graph_manager.h src/glviewer.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})  

##############################################################################
# Sources
##############################################################################
SET(ADDITIONAL_SOURCES src/gicp-fallback.cpp src/main.cpp src/qtros.cpp  src/openni_listener.cpp src/qtcv.cpp src/flow.cpp src/node.cpp src/graph_manager.cpp src/glviewer.cpp src/parameter_server.cpp)

IF (${USE_SIFT_GPU})
 	SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/sift_gpu_feature_detector.cpp)
ENDIF (${USE_SIFT_GPU})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS ${ADDITIONAL_SOURCES})

##############################################################################
# Binaries
##############################################################################

IF (${USE_SIFT_GPU})
add_definitions(-DUSE_SIFT_GPU)
ENDIF (${USE_SIFT_GPU})

IF (${USE_GICP_BIN})
add_definitions(-DUSE_ICP_BIN)
ENDIF (${USE_GICP_BIN})

IF (${USE_GICP_CODE})
add_definitions(-DUSE_ICP_CODE) 
ENDIF (${USE_GICP_CODE})

rosbuild_add_executable(rgbdslam ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})


SET(LIBS_LINK rgbdslam GL GLU ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
IF (${USE_SIFT_GPU})
 	SET(LIBS_LINK ${LIBS_LINK} siftgpu)
ENDIF (${USE_SIFT_GPU})

IF (${USE_GICP})
 	SET(LIBS_LINK ${LIBS_LINK} gicp ANN gsl gslcblas)
ENDIF (${USE_GICP})
 
IF (${USE_GICP})
# use this lines for gicp
  rosbuild_add_library(gicp gicp/bfgs_funcs.cpp gicp/gicp.cpp gicp/optimize.cpp gicp/scan.cpp gicp/transform.cpp)
ENDIF (${USE_GICP})
#rosbuild_add_library(ann gicp/ann_1.1.1/*cpp)

target_link_libraries(${LIBS_LINK})

#rosbuild_add_executable(pcl_manager src/pcl_manager/pcl_main.cpp)

