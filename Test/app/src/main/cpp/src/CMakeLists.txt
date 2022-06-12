INCLUDE_DIRECTORIES(.)
FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")

SET( opt_src   optimization/globaloptimizer.cpp     optimization/ippe.cpp     optimization/loopclosure.cpp      optimization/solvepnp.cpp)
SET( opt_hdr   optimization/globaloptimizer.h optimization/levmarq.h         optimization/ippe.h   optimization/sparselevmarq.h  optimization/proj_info.h optimization/loopclosure.h   optimization/solvepnp.h)

SET( opt_src ${opt_src} optimization/globaloptimizer_g2o.cpp optimization/solvepnp_g2o.cpp optimization/solvepnp_ucoslam.cpp)
SET( opt_hdr ${opt_hdr} optimization/globaloptimizer_g2o.h)

SET(basictypes_hdr basic_types/frame.h  basic_types/mappoint.h  basic_types/marker.h)
SET(basictypes_src basic_types/frame.cpp  basic_types/mappoint.cpp  basic_types/marker.cpp)
SET(viewers_src mapviewer.cpp )
SET(viewers_hdr mapviewer.h  viewers/slam_sglcvviewer.h  viewers/sgl.h)

 
SET(stuff_src stuff/debug.cpp  stuff/ferns.cpp  stuff/covisgraph.cpp  stuff/io_utils.cpp  stuff/utils.cpp stuff/matcher.cpp
      stuff/frameextractor.cpp stuff/framedatabase.cpp stuff/mapinitializer.cpp stuff/loopdetector.cpp)
SET(stuff_hdr stuff/debug.h                stuff/ferns.h   stuff/heap.h      stuff/picoflann.h       stuff/se3.h     stuff/utils.h
    stuff/expansiblecontainer.h  stuff/covisgraph.h  stuff/io_utils.h  stuff/reusablecontainer.h  stuff/timers.h stuff/matcher.h stuff/se3transform.h
     stuff/tsqueue.h stuff/frameextractor.h stuff/framedatabase.h stuff/mapinitializer.h stuff/minmaxbags.h stuff/hash.h   stuff/loopdetector.h)

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs ${srcs_base} )
ADD_LIBRARY(${PROJECT_NAME} SHARED ${srcs} ${hdrs} ${opt_src} ${opt_hdr}
		${viewers_src} ${viewers_hdr} ${stuff_src}
		${stuff_hdr} ${basictypes_hdr}
		${basictypes_src}
		)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )
#		apriltag::apriltag)
message("src当中的REQUIRED_LIBRARIES：${REQUIRED_LIBRARIES}")