cmake_minimum_required(VERSION 3.18)

project(HMM_MOS)

find_package(TBB REQUIRED)
find_package(nanoflann REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(unordered_dense CONFIG REQUIRED)
find_package(tsl-robin-map REQUIRED)

add_compile_options(-std=c++2a -O3)

# Add an executable called qcdmos
add_executable(qcdmos main.cpp ConfigParser.cpp Map.cpp Scan.cpp utils.cpp)

target_link_libraries(qcdmos
                      TBB::tbb
                      nanoflann::nanoflann
                      yaml-cpp::yaml-cpp
                      unordered_dense::unordered_dense)