# PX4 Model Predictive Control
ROS2 tools to control a PX4 based multicopter using Model Predictive Control techniques

## Building

### Build Fast RTPS (only once)

Follow [this installation link](https://dev.px4.io/master/en/setup/fast-rtps-installation.html#fast-rtps). Then:
```bash
cd path/to/the/FastRTPS
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j6
sudo make install
```

### Build Fast RTPS-Gen (only once)

Follow [this installation link](https://dev.px4.io/master/en/setup/fast-rtps-installation.html#fast-rtps-gen).
Then:
```bash
cd path/to/the/FastRTPSGen
gradle assemble
gradle install
```

**Note:** when doing `gradle install`, `sudo` might be required. If that is the case, when doing `sudo gradle install`, gradle might not be in the sudoer PATH. If this is the case do: `sudo path/to/gradle/executable install`, usually: `sudo /opt/gradle/gradle-6.2.2/bin/gradle install.

### Build PX4 with RTPS option to use this node with SITL

``` bash
cd path/to/the/PX4Firmware
make px4_sitl_rtps
```

### Build ROS2 workspace


## Send new custom message over using FastRTPS

1. Create the message inside the folder msgs of PX4
2. List it in the `CMakeLists.txt` inside the folder msgs of PX4
3. List in the `/Firmware/msg/tools/uorb_rtps_message_ids.yaml` file so that it can be translated to FastRTPS
4. Build PX4 with RTPS enabled (see above)

