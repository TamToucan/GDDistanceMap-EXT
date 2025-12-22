cmake_minimum_required(VERSION 3.20)

# --- 1. STANDALONE SETUP (Only runs if not a submodule) ---
if(NOT ALL_SUBS_BUILD_ON)
    project(DistanceMap-LIB)
    
    # Upgrading to 20 to match Cave/Maze
    set(CMAKE_CXX_STANDARD 20) 
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    
    # Options
    if(DEFINED BUILD_GODOT)
        set(BUILD_GODOT_DMAP ${BUILD_GODOT})
    else()
        option(BUILD_GODOT "Build Godot support" OFF)
        set(BUILD_GODOT_DMAP ${BUILD_GODOT})
    endif()

    option(BUILD_CUTE "Build Cute Framework support" ON)

    include(FetchContent)
    set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/_deps)

    # 1. Fetch Common Libs
    FetchContent_Declare(
        Libs
        GIT_REPOSITORY https://github.com/TamToucan/Libs.git
        GIT_TAG main
    )

    # 2. Fetch Tracker (DistanceMap Specific Dependency)
    if(NOT TARGET Tracker)
        FetchContent_Declare(
            Tracker
            GIT_REPOSITORY https://github.com/TamToucan/Tracker-LIB.git
            GIT_TAG main
        )
    endif()

    # 3. Fetch Godot (Only if needed)
    if(BUILD_GODOT_DMAP)
        FetchContent_Declare(
            GDExtension
            GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
            GIT_TAG godot-4.3-stable
        )
        FetchContent_MakeAvailable(GDExtension Libs Tracker)
    else()
        FetchContent_MakeAvailable(Libs Tracker)
    endif()

    # 4. Setup Cute (Standalone Mode)
    if(BUILD_CUTE)
        set(CUTE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/cute")
        if(EXISTS "${CUTE_ROOT}/lib/libcute.a")
            set(CUTE_FOUND ON)
            add_library(cute STATIC IMPORTED GLOBAL)
            set_target_properties(cute PROPERTIES
                IMPORTED_LOCATION "${CUTE_ROOT}/lib/libcute.a"
                INTERFACE_INCLUDE_DIRECTORIES "${CUTE_ROOT}/include;${CUTE_ROOT}"
            )
            file(GLOB CUTE_DEPENDENCIES "${CUTE_ROOT}/lib/*.a")
            list(REMOVE_ITEM CUTE_DEPENDENCIES "${CUTE_ROOT}/lib/libcute.a")
            set_target_properties(cute PROPERTIES INTERFACE_LINK_LIBRARIES "${CUTE_DEPENDENCIES}")
        endif()
    endif()

else()
    message(STATUS "DistanceMap-LIB is being built as a submodule.")
endif()


# --- 2. Build Core Library ---
# Point to the new split location
add_subdirectory(distancemap/src/core)
add_library(DistanceMapLib::DistanceMap ALIAS distancemap)


# --- 3. Build Godot Adapter ---
if(BUILD_GODOT OR BUILD_GODOT_DMAP)
    add_subdirectory(distancemap/src/godot)
endif()


# --- 4. Build Cute Adapter ---
if(BUILD_CUTE AND CUTE_FOUND)
    # Ensure source file exists at 'distancemap/src/cute/cuteDistanceMap.cpp'
    add_library(CuteDistanceMap SHARED distancemap/src/cute/cuteDistanceMap.cpp)
    
    # Link Core and Cute
    target_link_libraries(CuteDistanceMap PRIVATE DistanceMapLib::DistanceMap cute)
    target_include_directories(CuteDistanceMap PRIVATE distancemap/src/cute)
    
    # Windows: Copy DLL next to CuteDistanceMap so tests can run
    if(TARGET DistanceMapLib::DistanceMap)
         add_custom_command(TARGET CuteDistanceMap POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:DistanceMapLib::DistanceMap>
            $<TARGET_FILE_DIR:CuteDistanceMap>
        )
    endif()
endif()


# --- 5. Build Tests ---
option(BUILD_DMAP_TESTS "Build DistanceMap library tests" ON)

if(BUILD_DMAP_TESTS)
    add_subdirectory(distancemap/test)
endif()
