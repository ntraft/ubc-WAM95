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
include_directories (${ubc-WAM95_SOURCE_DIR}/lib)
include_directories (${ubc-WAM95_SOURCE_DIR}/exp)
include_directories (${ubc-WAM95_SOURCE_DIR}/tap)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/memory)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/senses)
include_directories (${ubc-WAM95_SOURCE_DIR}/robot/control)
include_directories (${ubc-WAM95_SOURCE_DIR}/tap)
# Make sure the linker can find the libraries once they are built. 
link_directories (${ubc-WAM95_BINARY_DIR}/lib)
link_directories (${ubc-WAM95_BINARY_DIR}/exp)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/memory)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/senses)
link_directories (${ubc-WAM95_SOURCE_DIR}/robot/control)
link_directories (${ubc-WAM95_BINARY_DIR}/tap)

#add in compile flags
#set(GCC_COMPILE_FLAGS "-DEIGEN_DONT_ALIGN=1") #prevent eigen unaligned array assertion
#add_definitions(${GCC_COMPILE_FLAGS})

add_executable (demo main.cxx)
add_executable (test test2.cxx)
add_executable (unit_test unit_test.cxx)

# Link the executable to dependent modules
# Note: Order is important! place modules to the left of their respective dependents
# target_link_libraries (demo exp control senses teach play robot mainline utils ${EXTRA_LIBS})
target_link_libraries (demo 
    tap teach play 
    robot 
    memory rtmemory 
    naive_bayes_system parameter_estimator pls_system
    control_strategy SaS SaA 
    control rtcontrol 
    senses sensor_stream_system 
    mainline var_server utils ${EXTRA_LIBS})
target_link_libraries (test 
    tap teach play 
    robot 
    memory rtmemory 
    naive_bayes_system parameter_estimator pls_system
    control_strategy SaS SaA
    control rtcontrol 
    senses sensor_stream_system 
    mainline var_server utils ${EXTRA_LIBS})
target_link_libraries (unit_test 
    tap teach play 
    robot 
    memory rtmemory 
    naive_bayes_system parameter_estimator pls_system
    control_strategy SaS SaA
    control rtcontrol 
    senses sensor_stream_system 
    mainline var_server utils ${EXTRA_LIBS})
