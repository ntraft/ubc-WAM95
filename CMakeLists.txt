cmake_minimum_required(VERSION 2.6)
project(libbarrett_examples)


# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


function(add_programs)
        foreach (prog ${ARGV})
                add_executable(${prog} ${prog}.cpp)
                target_link_libraries(${prog} ${BARRETT_LIBRARIES})
        endforeach()
endfunction()

add_programs(
	main
)

# ex04 uses the ncurses library
target_link_libraries(main ${CURSES_LIBRARIES})

