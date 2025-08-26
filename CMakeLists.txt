cmake_minimum_required(VERSION 3.10)
project(mavlink_protocol)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(CMAKE_C_COMPILER "<path_to_your_compiler>")
# set(CMAKE_CXX_COMPILER "<path_to_your_compiler>")
include_directories(
    include
    mavlink/include/v2.0
    src/talk
    )

file(GLOB_RECURSE PROTO_SRC "src/talk/*cpp")
add_library(baseprotocol SHARED ${PROTO_SRC})

file(GLOB_RECURSE APPLICATION_SRC "src/*cpp")
add_executable(${PROJECT_NAME} ${APPLICATION_SRC})
target_link_libraries(${PROJECT_NAME} 
                            baseprotocol
                            pthread)
