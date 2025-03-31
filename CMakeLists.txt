cmake_minimum_required(VERSION 3.16)

project(hmmMOS)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(TBB REQUIRED)
find_package(nanoflann REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(unordered_dense CONFIG REQUIRED)

add_compile_options(-std=c++2a -O3)

set(SOURCES 
    src/main.cpp 
    src/Map.cpp 
    src/Scan.cpp 
    src/utils.cpp
)

add_executable(hmmMOS ${SOURCES})

target_link_libraries(hmmMOS
    TBB::tbb
    nanoflann::nanoflann
    yaml-cpp::yaml-cpp
    unordered_dense::unordered_dense
)