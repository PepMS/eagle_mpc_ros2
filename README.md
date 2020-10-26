# PX4 Model Predictive Control
ROS2 tools to control a PX4 based multicopter using Model Predictive Control techniques

## PX4 firmware side

### Compile PX4 with RTPS option to use this node with SITL
``` bash
cd path/to/the/PX4Firmware
make px4_sitl_rtps
```
### Build Fast RTPS
Follow [this installation link](https://dev.px4.io/master/en/setup/fast-rtps-installation.html). Then:
```bash
cd path/to/the/FastRTPS
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j6
sudo make install
```

