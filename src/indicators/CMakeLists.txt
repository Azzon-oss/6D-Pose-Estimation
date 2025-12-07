cmake_minimum_required(VERSION 2.6)
project(improve_icp)
 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_INCLUDE_DIRS})

add_executable(improve_icp src/main.cpp)
target_link_libraries (improve_icp ${PCL_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS improve_icp RUNTIME DESTINATION bin)

