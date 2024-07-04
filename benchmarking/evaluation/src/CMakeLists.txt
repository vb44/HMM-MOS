cmake_minimum_required(VERSION 3.18)

project(eval)

add_compile_options(-std=c++2a -O3)

add_executable(evalIndFile
               evalIndFile.cpp)

add_executable(evalLabels
               evalLabels.cpp)