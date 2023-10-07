cmake_minimum_required(VERSION 3.22)
project(triangles)

set(CMAKE_CXX_STANDARD 17)

file(GLOB SRCS
        "${PROJECT_SOURCE_DIR}/src/main.cpp"
)

add_subdirectory(tests)
add_executable(triangles ${SRCS})
