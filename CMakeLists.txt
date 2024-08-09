#Minimum CMake version
cmake_minimum_required(VERSION 3.20)

# C++ standard
set(CMAKE_CXX_STANDARD 23)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_CXX_FLAGS  "/O2 /w  /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS  "-O3 -w -pthread -DNDEBUG")
endif()


#Project information
set(PROJECT_VERSION 0.1)
project(technician_routing VERSION ${PROJECT_VERSION} DESCRIPTION "Technician routing using column generation")
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if (NOT DEFINED ENV{GUROBI_HOME} AND NOT DEFINED GUROBI_DIR)
    message(FATAL_ERROR "No cache variable named GUROBI_DIR is defined, and GUROBI_HOME was not found in the environment variables. Please, define one of them.")
endif ()


# Add the cmake folder to the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Add the Gurobi headers from /opt/gurobi/include
find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})

# Add the pathwyse subdirectory
add_subdirectory(pathwyse)

#Source code
set(
    SOURCE_FILES_LIB

    src/instance.h
    src/instance.cpp
    src/parser.h 
    src/parser.cpp
    src/preprocessing.h
    src/preprocessing.cpp

    src/route.h
    src/route.cpp
    src/analysis.h
    src/analysis.cpp

    src/master.h
    src/RMP_solver.h
    src/RMP_solver.cpp
    src/pricing.h
    src/pricing.cpp
    src/column_generation.h
    src/column_generation.cpp

    src/time_window_lunch.h
    src/time_window_lunch.cpp

    src/compact_solver.h
    src/compact_solver.cpp
    src/solution_converter.h
    src/solution_converter.cpp

    src/branch_and_price.h
    src/branch_and_price.cpp

    src/heuristics.h
    src/heuristics.cpp
)
add_library(technician_routing_lib ${SOURCE_FILES_LIB})
#Link against the Gurobi library
target_link_libraries(technician_routing_lib PUBLIC gurobi)
#Link against the pathwyse core library
target_link_libraries(technician_routing_lib PUBLIC pathwyse_core)

# Create the column generation executable
add_executable(TRP_cg main.cpp)
# Link against the core library
target_link_libraries(TRP_cg PUBLIC technician_routing_lib)

# Also create a compact formulation executable
add_executable(TRP_compact main_compact.cpp)
# Link against the core library
target_link_libraries(TRP_compact PUBLIC technician_routing_lib)

# Create the branch & price executable
add_executable(TRP_BP main_bp.cpp)
# Link against the core library
target_link_libraries(TRP_BP PUBLIC technician_routing_lib) 