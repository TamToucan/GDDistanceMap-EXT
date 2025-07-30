project(GDDistanceMap)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_HOST_WIN32)
    set(MINGW TRUE)
    message(STATUS "MinGW detected: ${CMAKE_CXX_COMPILER}")
endif()

# Windows-specific settings
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_compile_definitions(GDDISTANCEMAP_EXPORTS)
endif()

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG:FULL")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG:FULL")
endif()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.C"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

if(CMAKE_BUILD_TYPE MATCHES RELEASE)
    set(LIB_NAME GDDistanceMap.release)
else()
    set(LIB_NAME GDDistanceMap.debug)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

add_library(${LIB_NAME} SHARED ${SOURCES})

if(MSVC)
    target_compile_options(${LIB_NAME} PRIVATE /Zi /Od)
    target_link_options(${LIB_NAME} PRIVATE /DEBUG)
    set_target_properties(${LIB_NAME} PROPERTIES
        PDB_NAME "${LIB_NAME}"
        PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMPILE_PDB_NAME "${LIB_NAME}"
        COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
endif()

set(GDTRACKER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../GDTracker-EXT")
set(GDTRACKER_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/GDTracker/build")

add_subdirectory("${GDTRACKER_SOURCE_DIR}" "${GDTRACKER_BINARY_DIR}")

# Include directories
target_include_directories(${LIB_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/Util"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/Algo"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/Stuff"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    "${GDTRACKER_SOURCE_DIR}/tracker/src"
    -isystem ${FETCHCONTENT_BASE_DIR})

# Link libraries
target_link_libraries(${LIB_NAME}
    PRIVATE MathStuff Stuff Algo GDTracker.debug
    PUBLIC godot::cpp)

# Test executable
add_executable(testMain "${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp")
target_link_libraries(testMain PRIVATE ${LIB_NAME})
target_include_directories(testMain PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${GDTRACKER_SOURCE_DIR}/tracker/src")

if(MSVC)
    target_compile_options(testMain PRIVATE /Zi /Od)
    target_link_options(testMain PRIVATE /DEBUG)
elseif(MINGW)
    target_compile_options(testMain PRIVATE -g -O0)
endif()

# Output directory and DLL copy logic
set(LOTT_ADDON_DIR "D:/git/the-lott/addons/gddistanceMap/bin")
set(GDTRACKER_DLL_PATH "${CMAKE_CURRENT_BINARY_DIR}/GDTracker/build/tracker/libGDTracker.debug.dll")
set(COPY_SCRIPT_PATH "${CMAKE_CURRENT_BINARY_DIR}/copy_tracker_if_exists.cmake")

file(WRITE "${COPY_SCRIPT_PATH}" "
if(EXISTS \"${GDTRACKER_DLL_PATH}\")
    message(STATUS \"Copying GDTracker DLL to: ${LOTT_ADDON_DIR}\")
    file(COPY \"${GDTRACKER_DLL_PATH}\" DESTINATION \"${LOTT_ADDON_DIR}\")
else()
    message(WARNING \"libGDTracker.debug.dll not found, skipping copy.\")
endif()
")

add_dependencies(${LIB_NAME} GDTracker.debug)

add_custom_command(
    TARGET ${LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${LIB_NAME}>" "${LOTT_ADDON_DIR}"
    COMMAND ${CMAKE_COMMAND} -P "${COPY_SCRIPT_PATH}"
)

function(BUILD_LIB)
    if(${ARGC} EQUAL 0)
        get_filename_component(lib ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    else()
        set(lib "${ARGV0}")
    endif()
    file(GLOB ${lib}_SOURCES *.C *.c *.cpp)
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../inc")
    add_library(${lib} STATIC ${${lib}_SOURCES})
endfunction()

set(LIBDIRS
    lib/Util
    lib/Algo
    lib/Stuff
    lib/MathStuff)

include_directories(${LIBDIRS})

foreach(DIR ${LIBDIRS})
    add_subdirectory(${DIR})
endforeach()
