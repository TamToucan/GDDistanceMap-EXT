project(GDCave)

# Automatically pick up files added to src
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.C"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    SET(LIB_NAME GDCave.release) 
ELSE()
    SET(LIB_NAME GDCave.debug) 
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
ENDIF(CMAKE_BUILD_TYPE MATCHES RELEASE)

add_library(${LIB_NAME} SHARED ${SOURCES})

target_include_directories(${LIB_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Util"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Algo"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/PCG"
					                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib")

target_link_libraries(${LIB_NAME} 
	PRIVATE Algo
	PRIVATE PCG
	PRIVATE MathStuff
	PUBLIC godot::cpp)

add_custom_command(
        TARGET ${LIB_NAME}  POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:${LIB_NAME}>
			    "C:/Users/tam/Dropbox/godot/lott-alt/addons/gdcave/bin")
			    
			    
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
    lib/MathStuff
    lib/PCG
    lib/Algo)
    
include_directories(${LIBDIRS})

foreach(DIR ${LIBDIRS})
   add_subdirectory(${DIR})
endforeach()

