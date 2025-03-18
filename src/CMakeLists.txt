cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(hmmMOS)

find_package(TBB REQUIRED)
find_package(nanoflann REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(unordered_dense CONFIG REQUIRED)
find_package(tsl-robin-map REQUIRED)

add_compile_options(-std=c++2a -O3)

# Add an executable called hmmMOS
add_executable(hmmMOS 
               main.cpp 
               ConfigParser.cpp 
               Map.cpp 
               Scan.cpp 
               utils.cpp)

target_link_libraries(hmmMOS
                      TBB::tbb
                      nanoflann::nanoflann
                      yaml-cpp::yaml-cpp
                      unordered_dense::unordered_dense)