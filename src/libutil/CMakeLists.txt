cmake_minimum_required(VERSION 3.5)


# set include directories
include_directories(${ODEDIR}/include
	${ODEDIR}/ode/src ${ODEDIR}/ode/demo)
include_directories(${EIGEN_DIR})

add_library(robotlib STATIC
	BasicMath.cpp
	robotsystem.cpp)

set_target_properties(robotlib
PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

target_link_libraries(robotlib debug ${ODEDIR}/lib/Debug/oded.lib)
target_link_libraries(robotlib optimized ${ODEDIR}/lib/Release/ode.lib)
