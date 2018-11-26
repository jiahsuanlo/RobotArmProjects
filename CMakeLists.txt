cmake_minimum_required(VERSION 3.5)

# set project name
set(PROJ_NAME robotarm)
project(${PROJ_NAME})

# set CMAKE flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_DEBUG_POSTFIX d)

# set ode and Eigen folders to the actual paths
set(ODEDIR C:/dev/ode-0.15.2)
set(EIGEN_DIR C:/dev/eigen-3.3.5)

# add subdirectories
add_subdirectory(src/libutil)
add_subdirectory(src/robotarm_app)
