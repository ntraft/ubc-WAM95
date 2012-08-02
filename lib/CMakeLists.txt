#should we link ncurses library?
option (USE_CURSES
        "Use the ncurses library, which helps display info in the terminal" OFF)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${BARRETT_LIBRARIES})

## curses
if (USE_CURSES)
        find_package(Curses REQUIRED)
        include_directories(${CURSES_INCLUDE_DIR})
        set (EXTRA_LIBS ${EXTRA_LIBS} ${CURSES_LIBRARIES})
endif (USE_CURSES)

function(add_libraries)
        foreach (lib ${ARGV})
                add_library(${lib} ${lib}.cpp)
                target_link_libraries(${lib} ${EXTRA_LIBS})
        endforeach()
endfunction()

add_libraries(
        io
)
