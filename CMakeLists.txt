#Minimum CMake version
cmake_minimum_required(VERSION 3.20)

# C++ standard
set(CMAKE_CXX_STANDARD 23)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_CXX_FLAGS  "/O2 /w  /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS  " -w  -pthread")
endif()


#Project information
set(PROJECT_VERSION 0.1)
project(technician_routing VERSION ${PROJECT_VERSION} DESCRIPTION "Technician routing using column generation")
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


if (NOT DEFINED ENV{GUROBI_HOME} AND NOT DEFINED GUROBI_DIR)
    message(FATAL_ERROR "No cache variable named GUROBI_DIR is defined, and GUROBI_HOME was not found in the environment variables. Please, define one of them.")
endif ()

#Source code
set(
    SOURCE_FILES_CORE

    src/instance/constants.h
    src/instance/instance.h
    src/instance/instance.cpp
    src/instance/parser.h
    src/instance/parser.cpp
    src/instance/preprocessing.h
    src/instance/preprocessing.cpp

    src/clustering/clustering.h
    src/clustering/clustering.cpp

    src/routes/route.h
    src/routes/route.cpp
    src/routes/route_optimizer.h
    src/routes/route_optimizer.cpp

    src/master_problem/master.h
    src/master_problem/master.cpp
    src/master_problem/rmp_solver.h
    src/master_problem/rmp_solver.cpp
    src/master_problem/node.h 
    src/master_problem/node.cpp
    src/master_problem/master_solver.h 
    src/master_problem/master_solver.cpp

    src/pulse/pulse.h
    src/pulse/pulse.cpp
    src/pulse/pulse_grouped.h
    src/pulse/pulse_grouped.cpp
    src/pulse/pulse_multithreaded.h
    src/pulse/pulse_multithreaded.cpp
    src/pulse/pulse_grouped_multithreaded.h
    src/pulse/pulse_grouped_multithreaded.cpp

    src/pricing_problem/full_pricing.h
    src/pricing_problem/full_pricing.cpp 
    src/pricing_problem/subproblem.h
    src/pricing_problem/subproblem.cpp
    src/pricing_problem/time_window_lunch.h
    src/pricing_problem/time_window_lunch.cpp

    src/repair/repair.h
    src/repair/repair.cpp

    src/algorithms/column_generation.h
    src/algorithms/column_generation.cpp
    src/algorithms/parameters.h
    src/algorithms/parameters.cpp
    src/algorithms/sequential_column_generation.h
    src/algorithms/sequential_column_generation.cpp

    src/data_analysis/analysis.h 
    src/data_analysis/analysis.cpp
    src/data_analysis/plot.h
    src/data_analysis/plot.cpp
    src/data_analysis/export.h
    src/data_analysis/export.cpp
)

set(
    SOURCE_FILES_COMPACT
    src/compact_formulation/compact_solution.h 
    src/compact_formulation/compact_solver.h
    src/compact_formulation/compact_solver.cpp
    src/compact_formulation/solution_converter.h
    src/compact_formulation/solution_converter.cpp
)

set(
    SOURCE_FILES_BRANCH_PRICE
    src/algorithms/branch_and_price.h
    src/algorithms/branch_and_price.cpp
)

################# Libraries #################
# Add the core library
add_library(technician_routing_core ${SOURCE_FILES_CORE})
target_include_directories(technician_routing_core PUBLIC src)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})
target_link_libraries(technician_routing_core PUBLIC gurobi)
add_subdirectory(pathwyse)
target_link_libraries(technician_routing_core PUBLIC pathwyse_core)
add_subdirectory(matplotplusplus)
target_link_libraries(technician_routing_core PUBLIC matplot)
find_package(TBB REQUIRED)
target_link_libraries(technician_routing_core PUBLIC TBB::tbb)

# Add the compact formulation library
add_library(technician_routing_compact ${SOURCE_FILES_COMPACT})

# Add the branch and price library
add_library(technician_routing_BP ${SOURCE_FILES_BRANCH_PRICE})
target_link_libraries(technician_routing_BP PUBLIC technician_routing_core)

################# Executables #################
# Create the column generation executable
add_executable(TRP_cg src/main_column_generation.cpp)
target_link_libraries(TRP_cg PUBLIC technician_routing_core)

# Create a testing executable
add_executable(TRP_test src/main_testing.cpp)
target_link_libraries(TRP_test PUBLIC technician_routing_core)

# Also create a compact formulation executable
add_executable(TRP_compact src/main_compact_formulation.cpp)
target_link_libraries(TRP_compact PUBLIC technician_routing_compact)

# Create the branch & price executable
add_executable(TRP_BP src/main_branch_price.cpp)
target_link_libraries(TRP_BP PUBLIC technician_routing_BP) 