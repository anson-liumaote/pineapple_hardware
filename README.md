# Pineapple Hardware Interface
DDS hardware interface for CSL wheel-biped robot
## Overview
### Hardware
- Joint motors: Unitree goM8010
- Wheel motors: DJI M3508
- IMU: Xsens Mti320

## Build
1. Follow this repo to install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
2. In pineapple_hardware
    ```
    mkdir build
    cd build
    cmake ..
    make
    ./install_can.sh
    ```
## Run
DDS bridge:
```
sudo ./DDS_bridge
```

Optioinal:
- test dds read and write:
    ```
    ./test_sendrecv
    ```
- test CAN read and write:
    ```
    ./test_can
    ```
- test all motors:
    ```
    ./test_all
    ```
