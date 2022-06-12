cmake_minimum_required(VERSION 3.18.1)
project("test")
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
add_definitions(-w)
#后面添加的，暂未知
ADD_DEFINITIONS(-DNOMINMAX)
SET(EIGEN3_INCLUDE_DIR "3rdparty/eigen3/eigen3")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "3rdparty/eigen3/eigen3")
include_directories( ${EIGEN3_INCLUDE_DIR} )
set(lib_path "${CMAKE_SOURCE_DIR}/../jniLibs")
set(OpenCV_DIR E:/OpenCV-android-sdk/sdk/native/jni/abi-arm64-v8a)
find_package(OpenCV 3 REQUIRED)
##所以find_package之后，主要是生成了几个cmake变量(xxx_FOUND, xxx_INCLUDE_DIRS, xxx_LIBRARIES).你需要判断是否找到，如果找到，需要加上include_directories和target_link_libraries。
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
SET(REQUIRED_LIBRARIES ${OpenCV_LIBS} )
include_directories(include)
include_directories(utility)
include_directories(utility/draw_result)
include_directories(utility/example)
add_library(realsense2 SHARED IMPORTED)
set_target_properties(realsense2 PROPERTIES IMPORTED_LOCATION ${lib_path}/librealsense2.so)
#add_library(opencv_java SHARED IMPORTED)
#set_target_properties(opencv_java PROPERTIES IMPORTED_LOCATION ${lib_path}/libopencv_java3.so)
set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/../jniLibs/libboost")
file(GLOB Boost_LIBRARIES ${BOOST_ROOT}/lib/libboost*.a)
include_directories( ${BOOST_ROOT}/include/boost-1_69)

#set(OPENCV_ROOT "${CMAKE_SOURCE_DIR}/../jniLibs/libopencv")
#add_library(opencv_calib3d STATIC IMPORTED)
#set_target_properties(opencv_calib3d PROPERTIES IMPORTED_LOCATION ${OPENCV_ROOT}/libopencv_calib3d.a)
#add_library(opencv_features2d STATIC IMPORTED)
#set_target_properties(opencv_features2d PROPERTIES IMPORTED_LOCATION ${OPENCV_ROOT}/libopencv_features2d.a)
#add_library(opencv_ml STATIC IMPORTED)
#set_target_properties(opencv_ml PROPERTIES IMPORTED_LOCATION ${OPENCV_ROOT}/libopencv_ml.a)
#add_library(opencv_imgproc STATIC IMPORTED)
#set_target_properties(opencv_imgproc PROPERTIES IMPORTED_LOCATION ${OPENCV_ROOT}/libopencv_imgproc.a)
#add_library(opencv_core STATIC IMPORTED)
#set_target_properties(opencv_core PROPERTIES IMPORTED_LOCATION ${OPENCV_ROOT}/libopencv_core.a)
include_directories(3rdparty/g2o/)
ADD_SUBDIRECTORY(3rdparty/g2o/g2o)
include_directories(3rdparty/apriltag)
ADD_SUBDIRECTORY(3rdparty/apriltag)
add_definitions(-DXFLANN_OPENCV)
add_definitions(-DXFEATURES2D)
ADD_SUBDIRECTORY(3rdparty/xflann/xflann)
include_directories(3rdparty/xflann)
SET(G2O_LIBS    g2o_stuff  g2o_core    g2o_types_sba)
ADD_SUBDIRECTORY(3rdparty/aruco/aruco) #目录里需要包含CMAKELISTS.txt
include_directories(3rdparty/aruco)
#SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} aruco xflann ${G2O_LIBS})
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} apriltag aruco xflann ${G2O_LIBS})
message("REQUIRED_LIBRARIES:${REQUIRED_LIBRARIES}")
add_subdirectory(src)
INCLUDE_DIRECTORIES(src)
link_libraries(${PROJECT_NAME})
FILE(GLOB hdrs_base "src/*.h" )
FILE(GLOB srcs_base "src/*.cpp")

SET( opt_src   src/optimization/globaloptimizer.cpp     src/optimization/ippe.cpp     src/optimization/loopclosure.cpp      src/optimization/solvepnp.cpp)
SET( opt_hdr   src/optimization/globaloptimizer.h src/optimization/levmarq.h         src/optimization/ippe.h   src/optimization/sparselevmarq.h  src/optimization/proj_info.h src/optimization/loopclosure.h   src/optimization/solvepnp.h)

SET( opt_src ${opt_src} src/optimization/globaloptimizer_g2o.cpp src/optimization/solvepnp_g2o.cpp src/optimization/solvepnp_ucoslam.cpp)
SET( opt_hdr ${opt_hdr} src/optimization/globaloptimizer_g2o.h)

SET(basictypes_hdr src/basic_types/frame.h  src/basic_types/mappoint.h  src/basic_types/marker.h)
SET(basictypes_src src/basic_types/frame.cpp  src/basic_types/mappoint.cpp  src/basic_types/marker.cpp)
SET(viewers_src mapviewer.cpp )
SET(viewers_hdr mapviewer.h  viewers/slam_sglcvviewer.h  viewers/sgl.h)


SET(stuff_src src/stuff/debug.cpp  src/stuff/ferns.cpp  src/stuff/covisgraph.cpp  src/stuff/io_utils.cpp  src/stuff/utils.cpp src/stuff/matcher.cpp
        src/stuff/frameextractor.cpp src/stuff/framedatabase.cpp src/stuff/mapinitializer.cpp src/stuff/loopdetector.cpp)
SET(stuff_hdr src/stuff/debug.h                src/stuff/ferns.h   src/stuff/heap.h      src/stuff/picoflann.h       src/stuff/se3.h     src/stuff/utils.h
        src/stuff/expansiblecontainer.h  src/stuff/covisgraph.h  src/stuff/io_utils.h  src/stuff/reusablecontainer.h  src/stuff/timers.h src/stuff/matcher.h src/stuff/se3transform.h
        src/stuff/tsqueue.h src/stuff/frameextractor.h src/stuff/framedatabase.h src/stuff/mapinitializer.h src/stuff/minmaxbags.h src/stuff/hash.h   src/stuff/loopdetector.h)

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs ${srcs_base} )
FILE(GLOB_RECURSE spmsrcs ${srcs} ${hdrs} ${opt_src} ${opt_hdr}
        ${viewers_src} ${viewers_hdr} ${stuff_src}
        ${stuff_hdr} ${basictypes_hdr}
        ${basictypes_src}
        )
set_property(SOURCE ${spmsrcs} APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 -fomit-frame-pointer -ffast-math")


include_directories(${CMAKE_SOURCE_DIR}/src)
#link_libraries(${PROJECT_NAME})

INCLUDE_DIRECTORIES(utility)
INCLUDE_DIRECTORIES(utility/draw_result/)
INCLUDE_DIRECTORIES(utility/example/)
FILE(GLOB_RECURSE utilitys
        utility/*.h
        utility/*.cpp
        utility/draw_result/*.h
        utility/draw_result/*.cpp
        utility/example/*.h
        utility/example/*.cpp

        )

set_property(SOURCE ${utilitys} APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 -fomit-frame-pointer -ffast-math")



#add_subdirectory(utility)
#include_directories(${CMAKE_SOURCE_DIR}/utility)
#file(GLOB_RECURSE slam_src
#        ${CMAKE_SOURCE_DIR}/src/*.h
#        ${CMAKE_SOURCE_DIR}/src/*.cpp
#        ${CMAKE_SOURCE_DIR}/src/basic_types/*.cpp
#        ${CMAKE_SOURCE_DIR}/src/basic_types/*.h
#        ${CMAKE_SOURCE_DIR}/src/optimization/*.h
#        ${CMAKE_SOURCE_DIR}/src/optimization/*.cpp
#        ${CMAKE_SOURCE_DIR}/src/stuff/*.h
#        ${CMAKE_SOURCE_DIR}/src/stuff/*.cpp
#        ${CMAKE_SOURCE_DIR}/src/viewer/*.h
#        )

#set_property(SOURCE ${slam_src} APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 -fomit-frame-pointer -ffast-math")


#file(GLOB_RECURSE srcs
#        ${CMAKE_SOURCE_DIR}/utility/*.cpp
#        ${CMAKE_SOURCE_DIR}/utility/draw_result/*.cpp
#        ${CMAKE_SOURCE_DIR}/utility/example/*.cpp
#        )
add_library(slam SHARED
#        ${spmsrcs}
        native-lib.cpp
        ${utilitys}
        )
find_library(log-lib log)
find_library( android-lib android )
target_link_libraries(slam
        ${OpenCV_LIBS}
#        ${Boost_LIBRARIES}
        realsense2
        ${android-lib}
        ${log-lib}
#        ${REQUIRED_LIBRARIES}
#        test
        )


###########################print
message("CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
message(WARNING "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")
#添加日志打印出来
#输出的位置：app\build\intermediates\cxx\Debug\6m2v3216\meta\arm64-v8a\metadata_generation_stderr.txt
message("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA>>>")
message("当前CMake的路径是：${CMAKE_SOURCE_DIR}")
