cmake_minimum_required(VERSION 3.16)
project(pineapple_hardware)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/xspublic
    ${CMAKE_SOURCE_DIR}/include/xspublic/xscontroller
    ${CMAKE_SOURCE_DIR}/include/xspublic/xscommon
    ${CMAKE_SOURCE_DIR}/include/xspublic/xstypes
)
# Link directories
link_directories(
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/include/xspublic/xscontroller
    ${CMAKE_SOURCE_DIR}/include/xspublic/xscommon
    ${CMAKE_SOURCE_DIR}/include/xspublic/xstypes
)

# Xsens libs
set(XSENS_OBJ_LIBS
    xscontroller
    xscommon
    xstypes
    pthread
    rt
    dl
)

# Select proper library based on architecture
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(EXTRA_UNITREE_MOTOR_LIB UnitreeMotorSDK_Arm64)
else()
    set(EXTRA_UNITREE_MOTOR_LIB UnitreeMotorSDK_Linux64)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
# find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(test_can
    src/DDS_bridge/test_can_main.cpp
    src/DDS_bridge/CanMotorController.cpp
)
target_link_libraries(test_can usb_can)

add_executable(test_all_main
    src/DDS_bridge/test_all_main.cpp
    src/DDS_bridge/MotorController.cpp
    src/DDS_bridge/CanMotorController.cpp
)
target_link_libraries(test_all_main ${EXTRA_UNITREE_MOTOR_LIB} usb_can pthread)

add_executable(DDS_bridge
    src/DDS_bridge_main.cpp
    src/DDS_bridge/DDS_bridge.cpp
    src/DDS_bridge/MotorController.cpp
    src/DDS_bridge/CanMotorController.cpp
    src/DDS_bridge/imu_thread.cpp
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

set(CMAKE_BUILD_TYPE Debug)

# Add Xsens examples
add_executable(example_mti_receive_data
    src/DDS_bridge/example_mti_receive_data.cpp
)
target_link_libraries(example_mti_receive_data ${XSENS_OBJ_LIBS})
