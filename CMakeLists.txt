# The name of our project is "WAM95Ctrl". CMakeLists files in this project can 
# refer to the root source directory of the project as ${WAM95Ctrl_SOURCE_DIR} and 
# to the root binary directory of the project as ${WAM95Ctrl_BINARY_DIR}. 
cmake_minimum_required (VERSION 2.6)
project (WAM95Ctrl)

# Recurse into the "lib" and "demo" subdirectories. This does not actually 
# cause another cmake executable to run. The same process will walk through 
# the project's entire directory structure. 
add_subdirectory (lib)
add_subdirectory (exp)
add_subdirectory (tap)
add_subdirectory (demo)
add_subdirectory (robot)
