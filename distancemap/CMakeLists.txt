project(DistanceMap-Lib)

# Core library
set(CORE_LIB_NAME distanceMap)
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp")

add_library(${CORE_LIB_NAME} SHARED ${CORE_SOURCES})

set_target_properties(${CORE_LIB_NAME} PROPERTIES OUTPUT_NAME ${CORE_LIB_NAME})
if(NOT MSVC)
    set_target_properties(${CORE_LIB_NAME} PROPERTIES PREFIX "")
endif()

target_include_directories(${CORE_LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/core")

target_link_libraries(${CORE_LIB_NAME}
    PUBLIC MathStuff
    PUBLIC Stuff
    PUBLIC Algo)

# Godot wrapper library
set(GD_LIB_NAME GDDistanceMap)
file(GLOB_RECURSE GD_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/godot/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/godot/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/godot/*.cpp")

add_library(${GD_LIB_NAME} SHARED ${GD_SOURCES})

set_target_properties(${GD_LIB_NAME} PROPERTIES OUTPUT_NAME ${GD_LIB_NAME})
if(NOT MSVC)
    set_target_properties(${GD_LIB_NAME} PROPERTIES PREFIX "")
endif()

target_include_directories(${GD_LIB_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/godot" "${CMAKE_CURRENT_SOURCE_DIR}/core")

target_link_libraries(${GD_LIB_NAME}
    PRIVATE ${CORE_LIB_NAME}
    PRIVATE TrackerLib::Tracker
    PRIVATE TrackerLib::GDTracker
    PUBLIC godot::cpp)

##############################

add_custom_target(copy_grid ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/test/GRID.txt"
        "$<TARGET_FILE_DIR:testMain>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/test/GRID.txt"
        "$<TARGET_FILE_DIR:testDistanceMap>"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/GRID.txt"
)

add_executable(testMain
    test/testMain.cpp
)

add_dependencies(testMain copy_grid)

link_directories(
    "${CMAKE_BINARY_DIR}/distancemap"
)

target_link_libraries(testMain
    PRIVATE MathStuff
    PRIVATE Stuff
    PRIVATE Algo
    PRIVATE TrackerLib::Tracker
    PRIVATE TrackerLib::GDTracker
    PRIVATE ${GD_LIB_NAME}
    PRIVATE ${CORE_LIB_NAME}
    PUBLIC godot::cpp)

target_include_directories(testMain PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/core")

# Define the DLL paths manually
IF (ALL_SUBS_BUILD_ON)
    set(GDDISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/DistanceMap-Lib/distancemap/${GD_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(DISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/DistanceMap-Lib/distancemap/${CORE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(GDTRACKER_DLL "${CMAKE_BINARY_DIR}/Tracker-Lib/tracker/GDTracker${CMAKE_SHARED_LIBRARY_SUFFIX}")
ELSE()
    set(GDDISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/distancemap/${GD_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(DISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/distancemap/${CORE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(GDTRACKER_DLL "${CMAKE_BINARY_DIR}/_deps/tracker-build/tracker/GDTracker${CMAKE_SHARED_LIBRARY_SUFFIX}")
ENDIF()

add_custom_command(TARGET testMain POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GDDISTANCEMAP_DLL}" "$<TARGET_FILE_DIR:testMain>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${DISTANCEMAP_DLL}" "$<TARGET_FILE_DIR:testMain>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${GDTRACKER_DLL}" "$<TARGET_FILE_DIR:testMain>"
)

##############################
# Test for DistanceMapNavigator

add_executable(testDistanceMap
    test/testDistanceMap.cpp
)

add_dependencies(testDistanceMap copy_grid)

target_link_libraries(testDistanceMap
    PRIVATE MathStuff
    PRIVATE Stuff
    PRIVATE Algo
    PRIVATE TrackerLib::Tracker
    PRIVATE TrackerLib::GDTracker
    PRIVATE ${GD_LIB_NAME}
    PRIVATE ${CORE_LIB_NAME}
    PUBLIC godot::cpp)

target_include_directories(testDistanceMap PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/core")

add_custom_command(TARGET testDistanceMap POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GDDISTANCEMAP_DLL}" "$<TARGET_FILE_DIR:testDistanceMap>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${DISTANCEMAP_DLL}" "$<TARGET_FILE_DIR:testDistanceMap>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${GDTRACKER_DLL}" "$<TARGET_FILE_DIR:testDistanceMap>"
)

##############################

target_compile_definitions(${CORE_LIB_NAME} PRIVATE GDDISTANCEMAP_EXPORTS)
target_compile_definitions(${GD_LIB_NAME} PRIVATE GDDISTANCEMAP_EXPORTS)

set(LOTT_ADDON_DIR "D:/git/the-lott/addons/gddistanceMap/bin")

add_dependencies(${GD_LIB_NAME} GDTracker)

add_custom_command(
    TARGET ${GD_LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${GD_LIB_NAME}>" "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${CORE_LIB_NAME}>" "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo_append "Attempting to copy GDTracker DLL... "
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GDTRACKER_DLL}" "${LOTT_ADDON_DIR}" || ${CMAKE_COMMAND} -E echo "GDTracker DLL not found, skipping."
    COMMENT "Copying GDDistanceMap and GDTracker DLLs to addon dir"
)

