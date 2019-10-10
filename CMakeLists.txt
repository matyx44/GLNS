cmake_minimum_required(VERSION 3.15.3)
project(GLNS)

set(CMAKE_CXX_STANDARD 17)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/x86_64-linux-gnu/pkgconfig")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")


include_directories("/usr/include/cairomm-1.0")

link_directories(
        ${GTKMM_LIBRARY_DIRS}
)

include_directories(
        ${GTKMM_INCLUDE_DIRS}
)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)

add_executable(GLNS main.cpp point.cpp point.h arc.cpp arc.h parser.h parser.cpp canvas.cpp canvas.h vertex.h edge.h vertex.cpp set.h set.cpp tour.h edge.cpp planner.cpp planner.h heuristic.h heuristic.cpp clipper.hpp clipper.cpp MapDrawer.cpp MapDrawer.hpp utils.cpp utils.hpp geom.cpp geom.hpp poly_maps.hpp)

target_link_libraries(GLNS
        ${GTKMM_LIBRARIES}
        )