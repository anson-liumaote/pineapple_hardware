cmake_minimum_required(VERSION 3.16)
project(pineapple_hardware)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
# Link directories
link_directories(
    ${CMAKE_SOURCE_DIR}/lib
)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
# find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(DDS_bridge
    src/DDS_bridge_main.cpp
    src/DDS_bridge/DDS_bridge.cpp
    src/DDS_bridge/imu_rs485.cpp
)
target_link_libraries(DDS_bridge
    ${EXTRA_UNITREE_MOTOR_LIB}
    usb_can
    pthread
    unitree_sdk2
    ${XSENS_OBJ_LIBS}
)

add_executable(test_sendrecv
    src/DDS_bridge/test_sendrecv.cpp
)
target_link_libraries(test_sendrecv
    pthread
    unitree_sdk2
)
find_package(pybind11 REQUIRED)

pybind11_add_module(pineapple_multi_imu
    src/python/py_multi_imu.cpp
    src/DDS_bridge/imu_rs485.cpp
)

target_include_directories(pineapple_multi_imu PRIVATE include)
target_link_libraries(pineapple_multi_imu PRIVATE pthread)