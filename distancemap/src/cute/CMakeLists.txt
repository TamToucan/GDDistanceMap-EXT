
add_library(CuteDistanceMap SHARED)

target_sources(CuteDistanceMap PRIVATE
	CuteDistanceMap.cpp
)

target_include_directories(CuteDistanceMap PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(CuteDistanceMap PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../core")

target_link_libraries(CuteDistanceMap
	PRIVATE DistanceMap
    PRIVATE Random
    PRIVATE MathStuff
    PRIVATE Tracker cute
)

set_target_properties(CuteDistanceMap PROPERTIES OUTPUT_NAME CuteDistanceMap)
set_target_properties(CuteDistanceMap PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT MSVC)
	set_target_properties(CuteDistanceMap PROPERTIES PREFIX "")
endif()



