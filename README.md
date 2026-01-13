# Pineapple Multi-IMU Hardware Interface
Multi-IMU interface for CSL wheel-biped robot


## How to build
1. Follow [csl_sdk2](https://github.com/anson-liumaote/csl_sdk2.git) to install multi-IMU sdk.
2. In pineapple_hardware
    ```
    mkdir build
    cd build
    cmake ..
    make
    ./install_can.sh
    ```
3. Setup serial port
    ```
    sudo chmod +x install_can.sh
    ./install_can.sh
    source ~/.bashrc
    ```
4. Follow this [link](https://gitee.com/kit-miao/dm-imu) to setup and calibrate DM IMU. Please use `RS485` mode.

## How to run
DDS bridge:
```
sudo ./DDS_bridge
```
Optional:
- test dds read and write:
    ```
    ./test_sendrecv
    ```
Multi-IMU Python Interfaces (without DDS!!):
```
sudo python3 py_multi_imu.py
```