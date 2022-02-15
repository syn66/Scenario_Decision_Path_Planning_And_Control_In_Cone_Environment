cmake_minimum_required(VERSION 3.7)
project(test_delaunay)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O2")

include_directories(.)
include_directories(External/matplotlib)
include_directories(External/Json/include)
include_directories(External/spline)
# include_directories(External/bezier_curve)

add_executable(test_delaunay
test_delaunay.cpp
                Plotting/plotting.h
                Plotting/plotting.cpp
                roadmap/load_map.h
                roadmap/load_map.cpp
                scenario_decision/drive_enviroment.cpp
                scenario_decision/drive_enviroment.h
                scenario_decision/scenario_manager.h
                scenario_decision/scenario_manager.cpp
                planning/lanekeep.h
                planning/lanekeep.cpp
                planning/delaunay_planner.h
                planning/delaunay_planner.cpp
                # planning/smoother.h
                # planning/smoother.cpp
                planning/quintic_polynomial.h
                planning/quintic_polynomial.cpp
                planning/bezier.h
                planning/bezier.cpp
                # model/dynamic_model.h
                # model/dynamic_model.cpp
                # control/LQR.h
                # control/LQR.cpp
                # control/LQR_speed.h
                # control/LQR_speed.cpp
                utils.h
                utils.cpp
                config.h
                )


find_package(Python COMPONENTS Development)
target_include_directories(test_delaunay PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(test_delaunay ${Python_LIBRARIES})


find_package(CGAL REQUIRED COMPONENTS Core)
target_link_libraries(test_delaunay CGAL::CGAL CGAL::CGAL_Core)
