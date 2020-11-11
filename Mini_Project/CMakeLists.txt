cmake_minimum_required(VERSION 3.17)
project(Mini_Project)

set(CMAKE_CXX_STANDARD 14)
set(Boost_INCLUDE_DIR D:/Users/John/Downloads/MinGW/include/)
set(Boost_LIBRARY_DIR D:/Users/John/Downloads/MinGW/lib/)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
add_executable(Mini_Project main.cpp)