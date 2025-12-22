# Target Name
set(DMAP_LIB_NAME distancemap)

# Source Files
file(GLOB_RECURSE DMAP_SOURCES CONFIGURE_DEPENDS
    "*.h" "*.hpp" "*.cpp"
)

# Define Shared Library
add_library(${DMAP_LIB_NAME} SHARED ${DMAP_SOURCES})

# Include Directories
target_include_directories(${DMAP_LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"    # . (src/core)
    "${CMAKE_CURRENT_SOURCE_DIR}/.." # .. (src)
    # Common Deps
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/MathStuff"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Algo"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Util"
)

# Properties
set_target_properties(${DMAP_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT MSVC)
    set_target_properties(${DMAP_LIB_NAME} PROPERTIES PREFIX "")
endif()

# Linking
# We link Tracker here because DistanceMap depends on it
target_link_libraries(${DMAP_LIB_NAME}
    PRIVATE Algo MathStuff
    PUBLIC Tracker # Assuming Tracker is needed by headers publically
)
