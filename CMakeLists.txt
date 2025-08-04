cmake_minimum_required(VERSION 3.20)
project(gdextension-distaceMap)

set(FETCHCONTENT_QUIET OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/_deps)

include(FetchContent)

FetchContent_Declare(
        GDExtension
        GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
        GIT_TAG godot-4.3-stable
)


FetchContent_Declare(
	Libs
	GIT_REPOSITORY https://github.com/TamToucan/Libs.git
	GIT_TAG main
)

FetchContent_Declare(
	GDTracker-EXT
	GIT_REPOSITORY https://github.com/TamToucan/GDTracker-EXT.git
	GIT_TAG main
)

FetchContent_MakeAvailable(GDExtension Libs GDTracker-EXT)

add_subdirectory(distancemap)
