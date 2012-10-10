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
include_directories (${WAM95Ctrl_SOURCE_DIR}/lib)
include_directories (${WAM95Ctrl_SOURCE_DIR}/data)
link_directories (${WAM95Ctrl_SOURCE_DIR}/lib)

add_library(exp 
	experiment.cxx
    flip.cxx
    bh.cxx
    action.cxx
)
