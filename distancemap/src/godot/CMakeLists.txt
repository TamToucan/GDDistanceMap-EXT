set(GDDMAP_LIB_NAME GDDistanceMap)

file(GLOB_RECURSE GDDMAP_SOURCES CONFIGURE_DEPENDS
    "*.h" "*.hpp" "*.cpp"
)

add_library(${GDDMAP_LIB_NAME} SHARED ${GDDMAP_SOURCES})
target_compile_definitions(${GDDMAP_LIB_NAME} PRIVATE GDDISTANCEMAP_EXPORTS)

target_include_directories(${GDDMAP_LIB_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../core"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Util"
)

if(NOT MSVC)
    set_target_properties(${GDDMAP_LIB_NAME} PROPERTIES PREFIX "")
endif()

# Link to the sibling 'DistanceMap' target
target_link_libraries(${GDDMAP_LIB_NAME}
    PRIVATE DistanceMap GDTracker
    PUBLIC godot::cpp
)

# Copy DLLs
set(GD_DMAP_ADDON_PATH, "D:/git/the-lott/addons/gddistancemap/bin")

add_custom_command(
    TARGET ${GDDMAP_LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DistanceMap>
	${GD_DMAP_ADDON_PATH}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:Tracker>
	${GD_DMAP_ADDON_PATH}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:GDTracker>
	${GD_DMAP_ADDON_PATH}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${GDDMAP_LIB_NAME}>
        "D:/git/the-lott/addons/gddistancemap/bin"
)
