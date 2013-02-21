## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
#link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
include_directories (${ubc-WAM95_SOURCE_DIR}/lib)

# Make sure the compiler can find include header files from our library. 
include_directories (${ubc-WAM95_SOURCE_DIR}/lib)
include_directories (${ubc-WAM95_SOURCE_DIR}/exp)
include_directories (${ubc-WAM95_SOURCE_DIR}/tap)
include_directories (${ubc-WAM95_SOURCE_DIR}/tap)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/memory)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/senses)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/control)
# Make sure the linker can find the libraries once they are built. 
link_directories (${ubc-WAM95_BINARY_DIR}/lib)
link_directories (${ubc-WAM95_BINARY_DIR}/exp)
link_directories (${ubc-WAM95_BINARY_DIR}/tap)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/memory)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/senses)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/control)


add_library (control control.cxx)
add_library (rtcontrol rtcontrol.cxx)
add_library (naive_bayes_system naive_bayes_system.cxx)
add_library (control_strategy control_strategy.cxx)
add_library (SaS SaS.cxx)
add_library (SaA SaA.cxx)
