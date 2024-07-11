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

#Source code
add_executable(technician_routing
    main.cpp
    src/instance.h
    src/parser.h 
    src/parser.cpp
    src/utils.h
)
