cmake_minimum_required(VERSION 3.16)
project(wonderduck_object_gpu LANGUAGES CXX)

find_package(ncnn CONFIG REQUIRED)

add_executable(object-gpu-runner ../object-gpu-runner.cpp)
target_compile_features(object-gpu-runner PRIVATE cxx_std_17)
target_link_libraries(object-gpu-runner PRIVATE ncnn)
