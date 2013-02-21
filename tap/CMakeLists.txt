
cmake_minimum_required (VERSION 2.6)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${BARRETT_LIBRARIES})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
set (EXTRA_LIBS ${EXTRA_LIBS} ${CURSES_LIBRARIES})

# Make sure the compiler and linker can find include header files and objects from our library. 
include_directories (${ubc-WAM95_SOURCE_DIR}/lib)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/senses)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/memory)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/control)
include_directories (${ubc-WAM95_SOURCE_DIR}/data)
link_directories (${ubc-WAM95_SOURCE_DIR}/lib)

add_library (teach teach.cxx)
add_library (play play.cxx)
add_library (display display.cxx)
add_library (tap tap.cxx)
