## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
#link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


add_library (utils utils.cxx)
add_library (robot robot.cxx)
add_library (senses senses.cxx)
add_library (control control.cxx)
add_library (mainline mainline.cxx)
#target_link_libraries(control utils)
