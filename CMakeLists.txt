cmake_minimum_required(VERSION 3.10)
project(suction_plugin)

# Find Gazebo Harmonic dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)

# Get plugin version for later use
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Add project include directory
include_directories(${PROJECT_SOURCE_DIR}/include)

# Create suction plugin library
add_library(suction_plugin SHARED
  src/suction_plugin.cpp
)

# Set C++ standard
set_property(TARGET suction_plugin PROPERTY CXX_STANDARD 17)

# Link with Gazebo Harmonic libraries
target_link_libraries(suction_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${gz-transport13_VERSION_MAJOR}::gz-transport${gz-transport13_VERSION_MAJOR}
  gz-msgs${gz-msgs10_VERSION_MAJOR}::gz-msgs${gz-msgs10_VERSION_MAJOR}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

# Get gz-sim plugin install directory (Simplified)
set(GZ_PLUGIN_PATH "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/")

# Set RPATH to include dependency paths for plugin loading
set_target_properties(suction_plugin PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "${GZ_PLUGIN_PATH}"
)

# Install the plugin to the Gazebo plugin directory
install(TARGETS suction_plugin
  LIBRARY DESTINATION "${GZ_PLUGIN_PATH}"
)

# Also install to standard lib location for general availability
install(TARGETS suction_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Print configuration info
message(STATUS "")
message(STATUS "=== Suction Plugin Configuration ===")
message(STATUS "Plugin directory: ${GZ_PLUGIN_PATH}")
message(STATUS "Gazebo Sim version: ${GZ_SIM_VER}")
message(STATUS "Gazebo Plugin version: ${GZ_PLUGIN_VER}")
message(STATUS "C++ Standard: 17")
message(STATUS "======================================")
message(STATUS "") 