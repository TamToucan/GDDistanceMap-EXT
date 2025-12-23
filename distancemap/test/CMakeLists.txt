# --- 1. Test Executable (Functional Tests) ---
add_executable(dmapTest testMain.cpp)

target_include_directories(distancemap_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(distancemap_test PRIVATE DistanceMapLib::DistanceMap)

# Windows: Copy DLL
if(TARGET DistanceMapLib::DistanceMap)
    add_custom_command(TARGET distancemap_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DistanceMapLib::DistanceMap>
        $<TARGET_FILE_DIR:distancemap_test>
    )
endif()


# --- 2. Time Executable (Benchmarks) ---
add_executable(dmapTime timeMain.cpp)

target_include_directories(distancemap_time PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(distancemap_time PRIVATE DistanceMapLib::DistanceMap)

# Windows: Copy DLL (Same logic for the second exe)
if(TARGET DistanceMapLib::DistanceMap)
    add_custom_command(TARGET distancemap_time POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DistanceMapLib::DistanceMap>
        $<TARGET_FILE_DIR:distancemap_time>
    )
endif()

