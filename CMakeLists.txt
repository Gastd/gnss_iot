cmake_minimum_required(VERSION 2.8)
project(gnss_iot)

include_directories(include)
add_definitions(-g -O3 -Wall)

include(ExternalProject)
ExternalProject_Add(minmea
    GIT_REPOSITORY https://github.com/Gastd/minmea
    GIT_TAG master
    # PREFIX ${CMAKE_CURRENT_BINARY_DIR}/minmea
    SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/minmea-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/minmea-src/build"
    # CONFIGURE_COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${CMAKE_BINARY_DIR}/minmea-src/"
    BUILD_COMMAND "${CMAKE_COMMAND}" --build .
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)
ExternalProject_Get_Property(minmea binary_dir)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/minmea-src)
ExternalProject_Get_Property(minmea install_dir)
# message(${install_dir})

set(SOURCES src/iot_gps.c
            src/tcp_iot.c
            src/state_machine.c)

add_executable(iot_gps ${SOURCES})
add_dependencies(iot_gps minmea)
target_link_libraries( iot_gps
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}minnea.so
    -pthread
)
