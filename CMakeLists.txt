cmake_minimum_required(VERSION 3.20)

if(DEFINED BUILD_GODOT)
    set(BUILD_GODOT_DISTANCEMAP ${BUILD_GODOT})
else()
    set(BUILD_GODOT_DISTANCEMAP ON)
endif()

if (NOT ALL_SUBS_BUILD_ON)
    project(DistaceMap-Lib)
    
    set(FETCHCONTENT_QUIET OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(CMAKE_VERBOSE_MAKEFILE ON)
    
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/_deps)
    
    include(FetchContent)
    
    if(BUILD_GODOT_DISTANCEMAP)
        FetchContent_Declare(
                GDExtension
                GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
                GIT_TAG godot-4.3-stable
        )
    endif()
    
    
    FetchContent_Declare(
    	Libs
    	GIT_REPOSITORY https://github.com/TamToucan/Libs.git
    	GIT_TAG main
    )
    if (NOT TARGET Tracker)
        # Only fetch Tracker if it’s not already part of the build
        FetchContent_Declare(
            Tracker
            GIT_REPOSITORY https://github.com/TamToucan/Tracker-LIB.git
            GIT_TAG main
        )
        FetchContent_MakeAvailable(Tracker)
    endif()
    if(BUILD_GODOT_DISTANCEMAP)
        FetchContent_MakeAvailable(GDExtension Libs)
    else()
        FetchContent_MakeAvailable(Libs)
    endif()
else()
	message(STATUS "DistanceMap-LIB is being built as part of Lott-LIB, skipping FetchContent.")
endif()


add_subdirectory(distancemap)
