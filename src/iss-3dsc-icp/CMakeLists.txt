cmake_minimum_required(VERSION 2.6)
project(iss-3dsc-icp)
 
find_package(PCL REQUIRED)
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_INCLUDE_DIRS})

add_executable(iss-3dsc-icp src/main.cpp src/Registration.cpp src/KeyPointsAnd3DSC.cpp)
target_link_libraries (iss-3dsc-icp ${PCL_LIBRARIES} ${Boost_LIBRARIES})



