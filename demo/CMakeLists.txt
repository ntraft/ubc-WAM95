## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${BARRETT_LIBRARIES})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
set (EXTRA_LIBS ${EXTRA_LIBS} ${CURSES_LIBRARIES})

set (EXTRA_LIBS ${EXTRA_LIBS} boost_system boost_filesystem)

# Make sure the compiler can find include header files from our library. 
include_directories (${WAM95Ctrl_SOURCE_DIR}/lib)
include_directories (${WAM95Ctrl_SOURCE_DIR}/exp)
include_directories (${WAM95Ctrl_SOURCE_DIR}/tap)
include_directories (${WAM95Ctrl_SOURCE_DIR}/tap)
include_directories (${WAM95Ctrl_SOURCE_DIR}/robot)
include_directories (${WAM95Ctrl_SOURCE_DIR}/robot/memory)
include_directories (${WAM95Ctrl_SOURCE_DIR}/robot/senses)
include_directories (${WAM95Ctrl_SOURCE_DIR}/robot/control)
# Make sure the linker can find the libraries once they are built. 
link_directories (${WAM95Ctrl_BINARY_DIR}/lib)
link_directories (${WAM95Ctrl_BINARY_DIR}/exp)
link_directories (${WAM95Ctrl_BINARY_DIR}/tap)
link_directories (${WAM95Ctrl_SOURCE_DIR}/robot)
link_directories (${WAM95Ctrl_SOURCE_DIR}/robot/memory)
link_directories (${WAM95Ctrl_SOURCE_DIR}/robot/senses)
link_directories (${WAM95Ctrl_SOURCE_DIR}/robot/control)

add_executable (demo main.cxx)

# Link the executable to dependent modules
# Note: Order is important! place modules to the left of their respective dependents
#target_link_libraries (demo exp control senses teach play robot mainline utils ${EXTRA_LIBS})
target_link_libraries (demo exp robot control senses memory mainline utils ${EXTRA_LIBS})
