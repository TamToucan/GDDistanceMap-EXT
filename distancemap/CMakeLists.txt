project(GDDistanceMap)
SET(LIB_NAME GDDistanceMap)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.C"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(${LIB_NAME} SHARED ${SOURCES})

# Include directories
target_include_directories(${LIB_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Link libraries
target_link_libraries(${LIB_NAME}
    PRIVATE MathStuff
    PRIVATE Stuff
    PRIVATE Algo
    PRIVATE GDTracker
    PUBLIC godot::cpp)

##############################

add_custom_target(copy_grid ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/test/GRID.txt"
        "$<TARGET_FILE_DIR:testMain>"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/GRID.txt"
)


add_executable(testMain
    test/testMain.cpp
)

add_dependencies(testMain copy_grid)

# Add library directories
link_directories(
    "${CMAKE_BINARY_DIR}/distancemap"
)

# Link testMain against your built libraries
target_link_libraries(testMain
    PRIVATE MathStuff
    PRIVATE Stuff
    PRIVATE Algo
    PRIVATE GDTracker
    PRIVATE GDDistanceMap
    PUBLIC godot::cpp)

target_include_directories(testMain PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Define the DLL paths manually (update these if needed)
IF (ALL_SUBS_BUILD_ON)
    set(GDDISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/GDDistanceMap-EXT/distancemap/libGDDistanceMap.dll")
    set(GDTRACKER_DLL "${CMAKE_BINARY_DIR}/GDTracker-EXT/tracker/libGDTracker.dll")
ELSE()
    set(GDDISTANCEMAP_DLL "${CMAKE_BINARY_DIR}/distancemap/libGDDistanceMap.dll")
    set(GDTRACKER_DLL "${CMAKE_BINARY_DIR}/_deps/gdtracker-ext-build/tracker/libGDTracker.dll")
ENDIF()

# Copy DLLs to same folder as testMain.exe after build
add_custom_command(TARGET testMain POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GDDISTANCEMAP_DLL}" "$<TARGET_FILE_DIR:testMain>"
)

##############################

target_compile_definitions(${LIB_NAME} PRIVATE ${PROJECT_NAME}_EXPORTS)

# Output directory and DLL copy logic
set(LOTT_ADDON_DIR "D:/git/the-lott/addons/gddistanceMap/bin")

add_dependencies(${LIB_NAME} GDTracker)


add_custom_command(
    TARGET ${LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${LIB_NAME}>" "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo_append "Attempting to copy GDTracker DLL... "
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GDTRACKER_DLL}" "${LOTT_ADDON_DIR}" || ${CMAKE_COMMAND} -E echo "GDTracker DLL not found, skipping."
    COMMENT "Copying GDDistanceMap and GDTracker DLLs to addon dir"
)

