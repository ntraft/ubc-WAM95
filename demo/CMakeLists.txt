## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${BARRETT_LIBRARIES})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
set (EXTRA_LIBS ${EXTRA_LIBS} ${CURSES_LIBRARIES})

# Make sure the compiler can find include header files from our library. 
include_directories (${WAM95Ctrl_SOURCE_DIR}/lib)
# Make sure the linker can find the libraries once they are built. 
link_directories (${WAM95Ctrl_BINARY_DIR}/lib)
# Add executable called "helloDemo" that is built from the source files 
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found. 
add_executable (demo main.cxx)
# Link the executable to the Hello library. 
target_link_libraries (demo utils ${EXTRA_LIBS})
