#Minimum CMake version
cmake_minimum_required(VERSION 3.20)

# C++ standard
set(CMAKE_CXX_STANDARD 23)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_CXX_FLAGS  "/O2 /w  /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS  " -w  -O3 -pthread -DNDEBUG")
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
    SOURCE_FILES_LIB

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
    src/master_problem/integer_solution.h
    src/master_problem/integer_solution.cpp

    src/pricing_problem/pricing.h
    src/pricing_problem/pricing.cpp 
    src/pricing_problem/subproblem.h
    src/pricing_problem/subproblem.cpp
    src/pricing_problem/time_window_lunch.h
    src/pricing_problem/time_window_lunch.cpp
    src/pricing_problem/tabu.h
    src/pricing_problem/tabu.cpp

    src/repair/repair.h
    src/repair/repair.cpp

    src/compact_formulation/compact_solution.h 
    src/compact_formulation/compact_solver.h
    src/compact_formulation/compact_solver.cpp
    src/compact_formulation/solution_converter.h
    src/compact_formulation/solution_converter.cpp

    src/algorithms/branch_and_price.h
    src/algorithms/branch_and_price.cpp
    src/algorithms/column_generation.h
    src/algorithms/column_generation.cpp
    src/algorithms/genetic_solver.h
    src/algorithms/genetic_solver.cpp
    src/algorithms/heuristics.h
    src/algorithms/heuristics.cpp

    src/data_analysis/analysis.h 
    src/data_analysis/analysis.cpp
    src/data_analysis/plot.h
    src/data_analysis/plot.cpp
    src/data_analysis/export.h
    src/data_analysis/export.cpp
)

# Add the library
add_library(technician_routing_lib ${SOURCE_FILES_LIB})
# Add the include directory
target_include_directories(technician_routing_lib PUBLIC src)

# Add the cmake folder to the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Add the Gurobi headers from /opt/gurobi/include
find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})

# Add the pathwyse subdirectory
add_subdirectory(pathwyse)

# Add Matplotplusplus
add_subdirectory(matplotplusplus)

#Link against the Gurobi library
target_link_libraries(technician_routing_lib PUBLIC gurobi)
#Link against the pathwyse core library
target_link_libraries(technician_routing_lib PUBLIC pathwyse_core)
#Link against Matplotplusplus
target_link_libraries(technician_routing_lib PUBLIC matplot)
# Add the necessary library for parallel running
target_link_libraries(technician_routing_lib PUBLIC tbb)


# Create the column generation executable
add_executable(TRP_cg src/main_column_generation.cpp)
# Link against the core library
target_link_libraries(TRP_cg PUBLIC technician_routing_lib)


# Also create a compact formulation executable
add_executable(TRP_compact src/main_compact_formulation.cpp)
# Link against the core library
target_link_libraries(TRP_compact PUBLIC technician_routing_lib)


# Create the branch & price executable
add_executable(TRP_BP src/main_branch_price.cpp)
# Link against the core library
target_link_libraries(TRP_BP PUBLIC technician_routing_lib) 


# Create an executable to test out predifined solutions
add_executable(TRP_test_solutions src/main_route_testing.cpp)
# Link against the core library
target_link_libraries(TRP_test_solutions PUBLIC technician_routing_lib)


# Create an executable to test the genetic algorithm
add_executable(TRP_genetic src/main_genetic_algorithm.cpp)
# Link against the core library
target_link_libraries(TRP_genetic PUBLIC technician_routing_lib)