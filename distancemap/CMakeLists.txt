project(GDDistanceMap)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_HOST_WIN32)
        set(MINGW TRUE)
        message(STATUS "MinGW detected: ${CMAKE_CXX_COMPILER}")
endif()

# Windows-specific settings
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)  # Simplifies DLL exports
    add_compile_definitions(GDDISTANCEMAP_EXPORTS)  # For manual export control
endif()

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG:FULL")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG:FULL")
endif()

# Automatically pick up files added to src
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.C"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    SET(LIB_NAME GDDistanceMap.release) 
ELSE()
    SET(LIB_NAME GDDistanceMap.debug) 
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
ENDIF(CMAKE_BUILD_TYPE MATCHES RELEASE)

add_library(${LIB_NAME} SHARED ${SOURCES})

# MSVC-specific settings AFTER target creation
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

# 1. First set the paths clearly
set(GDTRACKER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../GDTracker-EXT")
set(GDTRACKER_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/GDTracker/build")

# 2. Add the subdirectory with explicit paths
add_subdirectory(
    "${GDTRACKER_SOURCE_DIR}"
    "${GDTRACKER_BINARY_DIR}"
)

target_include_directories(${LIB_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Util"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Algo"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Stuff"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib"
                                       PUBLIC "${GDTRACKER_SOURCE_DIR}/tracker/src")

# 5. Link libraries - CRITICAL FIX HERE
target_link_libraries(${LIB_NAME}
    PRIVATE
        MathStuff
        Stuff
        Algo
        GDTracker.debug
    PUBLIC
        godot::cpp
)

target_include_directories(${LIB_NAME} PRIVATE -isystem ${FETCHCONTENT_BASE_DIR})

add_executable(testMain "${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp")
target_link_libraries(testMain PRIVATE ${LIB_NAME})
target_include_directories(testMain PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(MSVC)
    target_compile_options(testMain PRIVATE /Zi /Od)
    target_link_options(testMain PRIVATE /DEBUG)
elseif(MINGW)
    target_compile_options(${LIB_NAME} PRIVATE -g -O0 -gdwarf-4 -gstrict-dwarf)
    target_link_options(${LIB_NAME} PRIVATE -g -Wl,--export-all-symbols)
endif()

add_custom_command(
        TARGET ${LIB_NAME}  POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:${LIB_NAME}>
			    "D:/git/the-lott/addons/gddistanceMap/bin")
			    
			    
			    #
# Build lib using all sources and named same as dir (if no argv1)
#
function(BUILD_LIB)
   if (${ARGC} EQUAL 0)
      get_filename_component(lib ${CMAKE_CURRENT_SOURCE_DIR} NAME)
   else (${ARGC} EQUAL 0)
      set(lib "${ARGV0}")
   endif (${ARGC} EQUAL 0)
   file(GLOB ${lib}_SOURCES *.C *.c *.cpp)
   include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../inc")
   add_library(${lib} STATIC ${${lib}_SOURCES})
endfunction(BUILD_LIB)

set(LIBDIRS
    lib/Util
    lib/Algo
    lib/Stuff
    lib/MathStuff)
    
include_directories(${LIBDIRS})

foreach(DIR ${LIBDIRS})
   add_subdirectory(${DIR})
endforeach()

