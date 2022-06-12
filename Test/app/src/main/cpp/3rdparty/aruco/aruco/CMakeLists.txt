include_directories(.)
include_directories(../../eigen3/eigen3/)
set(sources
        cameraparameters.cpp
        cvdrawingutils.cpp
        dictionary.cpp
        ippe.cpp
        marker.cpp
        markerdetector.cpp
        markerlabeler.cpp
        markermap.cpp
        posetracker.cpp
        markerlabelers/dictionary_based.cpp
        debug.cpp
        markerlabelers/svmmarkers.cpp
        )

set(headers
        aruco_export.h
        cameraparameters.h
        cvdrawingutils.h
        dictionary.h
        ippe.h
        marker.h
        markerdetector.h
        markerlabeler.h
        markermap.h
        posetracker.h
        markerlabelers/dictionary_based.h
        timers.h
        debug.h
        aruco.h
        markerlabelers/svmmarkers.h
        )

add_library(aruco SHARED ${sources} ${headers})
#target_link_libraries(aruco opencv_calib3d opencv_features2d opencv_ml opencv_imgproc opencv_core ${openCV_LIBS} -lz)
#IF( COMPILE_IN_ANDROID)
#    message("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwlaaaaaaaaaaaaaaaaaaaaaaaa")
#    #target_link_libraries(aruco opencv_calib3d opencv_features2d opencv_ml opencv_imgproc opencv_core ${openCV_LIBS} -lz)
#ELSE()
#    message(STATUS "ARUCO COMPILATION")
#    message("HAHAHAHAHAHAHAHAHAHAHAHA")
#ENDIF()
target_link_libraries(aruco ${OpenCV_LIBS})
#target_link_libraries(aruco PRIVATE opencv_imgproc opencv_calib3d opencv_features2d opencv_ml -lz)