cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
project(test_poly)

find_package(Eigen3)

include_directories(
  mav_trajectory_generation/include
  ${EIGEN3_INCLUDE_DIR}
)

# add the data to the target, so it becomes visible in some IDE
add_executable(test_poly test_poly.cpp 
  mav_trajectory_generation/src/vertex.cpp
  mav_trajectory_generation/src/polynomial.cpp
  mav_trajectory_generation/src/segment.cpp
  mav_trajectory_generation/src/motion_defines.cpp
  mav_trajectory_generation/src/rpoly/rpoly_ak1.cpp
)

target_link_libraries(test_poly PUBLIC
)
