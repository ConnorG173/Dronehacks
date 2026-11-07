cmake_minimum_required(VERSION 3.10)
project(gcopter_planner CXX)

# Upstream builds Release with "-O3 -Wall -fPIC" and -std=c++14. We use C++17
# because OMPL >= 1.6 headers require it; the optimization flags are upstream's.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)

set(GCOPTER_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gcopter/include)

# The planner.
add_executable(gcopter_planner src/main.cpp)
target_include_directories(gcopter_planner PRIVATE ${GCOPTER_INCLUDE} ${OMPL_INCLUDE_DIRS})
target_link_libraries(gcopter_planner PRIVATE Eigen3::Eigen ${OMPL_LIBRARIES})

# Parity harness: upstream's demo pipeline, same headers and flags.
add_executable(upstream_demo tests/upstream_demo.cpp)
target_include_directories(upstream_demo PRIVATE ${GCOPTER_INCLUDE} ${OMPL_INCLUDE_DIRS})
target_link_libraries(upstream_demo PRIVATE Eigen3::Eigen ${OMPL_LIBRARIES})
