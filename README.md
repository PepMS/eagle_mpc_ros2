# Eagle MPC - ROS2 (WIP)

This repo was initially considered to implement an MPC controller along with PX4 firmware (hence its name).
However, this has evolved to the more general case where any controller outputting motor thrusts (or motor speed) can be implemented.

In case one wants to implement its own controller he/she only needs to create a derived class from the derived controller overwritting the necessary methods and properties.

## Building

### Dependencies (only once)

#### Build Fast RTPS

Follow [this installation link](https://dev.px4.io/master/en/setup/fast-rtps-installation.html#fast-rtps). Then:
```bash
cd path/to/the/FastRTPS
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j6
sudo make install
```

#### Build Fast RTPS-Gen

Follow [this installation link](https://dev.px4.io/master/en/setup/fast-rtps-installation.html#fast-rtps-gen).
Then:
```bash
cd path/to/the/FastRTPSGen
gradle assemble
gradle install
```

**Note:** when doing `gradle install`, `sudo` might be required. If that is the case, when doing `sudo gradle install`, gradle might not be in the sudoer PATH. If this is the case do: `sudo path/to/gradle/executable install`, usually: `sudo /opt/gradle/gradle-6.2.2/bin/gradle install`.

### PX4 related

#### Build PX4 with RTPS option to use this node with SITL

``` bash
cd path/to/the/PX4Firmware
make px4_sitl_rtps
```

#### Build ROS2 workspace
ROS2 Dashing is required ([install link](https://index.ros.org/doc/ros2/Installation/Dashing/)).

Then 
## Send new custom message over using FastRTPS

1. Create the message inside the folder msgs of PX4
2. List it in the `CMakeLists.txt` inside the folder msgs of PX4
3. List in the `/Firmware/msg/tools/uorb_rtps_message_ids.yaml` file so that it can be translated to FastRTPS
4. Build PX4 with RTPS enabled (see above)

## Creating your own controller

See the given example code: `example_controller.cpp
