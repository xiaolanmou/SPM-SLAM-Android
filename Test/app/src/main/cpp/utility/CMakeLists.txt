INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(./draw_result/)
INCLUDE_DIRECTORIES(./example/)
INCLUDE_DIRECTORIES(.)
FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")
FILE(GLOB_RECURSE  draw_srcs draw_result/*.h)
FILE(GLOB_RECURSE  draw_hdrs draw_result/*.cpp)
FILE(GLOB_RECURSE  example_srcs example/*.h)
FILE(GLOB_RECURSE  example_hdrs example/*.cpp)
ADD_LIBRARY(utilitys SHARED
		${hdrs_base} ${srcs_base}
		${draw_srcs} ${draw_hdrs}
		${example_srcs} ${example_hdrs}
		)
TARGET_LINK_LIBRARIES(utilitys realsense2 ${REQUIRED_LIBRARIES} test )
#		apriltag::apriltag)
message("utility当中的REQUIRED_LIBRARIES：${REQUIRED_LIBRARIES}")