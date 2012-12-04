cmake_minimum_required(VERSION 2.6)
project(teach_and_play)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

include_directories (../../lib)
link_directories (../../lib)

add_executable(teach teach.cpp)
target_link_libraries(teach ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})

add_library (hand_system hand_system.cxx)
target_link_libraries(hand_system ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(hand_system ${BARRETT_LIBRARIES} boost_system boost_filesystem)

add_library (memory memory.cxx)
target_link_libraries(memory ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(memory ${BARRETT_LIBRARIES} boost_system boost_filesystem)

add_library (wam_system wam_system.cxx)
target_link_libraries(wam_system ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(wam_system ${BARRETT_LIBRARIES} boost_system boost_filesystem)

add_library (data_stream data_stream.cxx)

add_library (display display.cxx)
target_link_libraries(display ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(display ${BARRETT_LIBRARIES} boost_system boost_filesystem)

add_executable(play play.cpp)
target_link_libraries(play ${BARRETT_LIBRARIES} boost_system boost_filesystem)
target_link_libraries(play ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(play data_stream display)

add_executable(loop loop.cpp)
target_link_libraries(loop ${BARRETT_LIBRARIES} boost_system boost_filesystem)
target_link_libraries(loop ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(loop data_stream display)

add_executable(rtloop rtloop.cpp)
target_link_libraries(rtloop ${BARRETT_LIBRARIES} boost_system boost_filesystem)
target_link_libraries(rtloop ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
target_link_libraries(rtloop data_stream hand_system wam_system display utils)

add_executable(bt-wam-ccg calibrate_control_gains.cpp)
target_link_libraries(bt-wam-ccg ${BARRETT_LIBRARIES})
