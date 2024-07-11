#Minimum CMake version
cmake_minimum_required(VERSION 3.20)

# C++ standard
set(CMAKE_CXX_STANDARD 23)

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

#Source code
add_executable(technician_routing
    main.cpp
    src/instance.h
    src/parser.h 
    src/parser.cpp
    src/utils.h
    src/cg_solver.h
)

#Link against the Gurobi library
target_link_libraries(technician_routing PUBLIC gurobi)
