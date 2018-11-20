cmake_minimum_required(VERSION 3.5)
project("PyOrbSlam")
set(CMAKE_CXX_FLAGS "-std=c++0x")

#VERSION 1.0.0 DESCRIPTION "Library which allows to use ORB SLAM in python programs")
add_library(PyOrbSlam SHARED
  PyOrbSlam.cpp
)
set_target_properties(PyOrbSlam PROPERTIES PREFIX "")

#set_target_properties(mylib PROPERTIES VERSION ${PROJECT_VERSION})

find_package(PythonLibs REQUIRED)
set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS} /usr/local/lib/python3.7/dist-packages/numpy/core/include)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(PyOrbSlam ${PYTHON_LIBRARIES})

#find_library(PROTOBUF_LIBRARY protobuf HINTS /usr/lib/x86_64-linux-gnu)
#target_link_libraries(test PUBLIC ${PROTOBUF_LIBRARY})

include_directories(${CMAKE_BINARY_DIR}/../../ORB_SLAM2/)
include_directories(${CMAKE_BINARY_DIR}/../../ORB_SLAM2/include)

find_library(ORB_SLAM2_LIBRARY libORB_SLAM2.so HINTS ${CMAKE_BINARY_DIR}/../../ORB_SLAM2/lib)
target_link_libraries(PyOrbSlam ${ORB_SLAM2_LIBRARY})

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

#target_link_libraries (PyOrbSlam Eigen3)

