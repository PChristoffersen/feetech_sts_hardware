cmake_Minimum_required(VERSION 3.5)
set(project "SCServo")
project(${project})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-vla -std=c++11 -O3")

file(GLOB hdrs src/*.h)
file(GLOB srs src/*.cpp)

add_library(${project} STATIC ${hdrs} ${srs})
#add_executable(${project} main.cpp ${hdrs} ${srs})
target_compile_options(${project} PRIVATE -std=c++11 -O3 -Wno-vla -fPIC)