# Servo42c robot arm controller

## Overview

A ROS2 controller for a robot arm, utilizing SERVO42C v1.0 controllers for stepper motors.
Using Ros2 jazzy.

At the moment only works with v.1.0 of protocol (no crc)

# TODO
- add IK via moveit
  - moveit should control joints
  - ~~moveit should be controllable via foxglove~~ ✅ Foxglove Bridge setup complete
- add calibration via accelerometers
  - add reset command for servos
  - add a package to query from acc
- test acceleration control

## Foxglove Bridge Setup

### Manual Launch
To test the Foxglove Bridge manually:
```bash
source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge
```
The bridge will listen on `ws://0.0.0.0:8765` for Foxglove Studio connections.

### Autostart on Boot (systemd)

To automatically launch the Foxglove Bridge when the system boots using `systemd` on Ubuntu 22.04 or similar:

1.  **Create a `systemd` service file:**
    ```bash
    sudo nano /etc/systemd/system/foxglove_bridge.service
    ```

2.  **Add the following content:**
    *Replace `myuser` with the correct username and `jazzy` with your ROS 2 distribution.*

    ```ini
    [Unit]
    Description=Foxglove Bridge ROS 2 Launch Service
    After=network.target

    [Service]
    User=myuser
    ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge'
    Restart=on-failure
    RestartSec=5

    [Install]
    WantedBy=multi-user.target
    ```
    *Note: This runs the foxglove_bridge node directly, which works better with systemd than using the launch file.*

3.  **Enable the service:**
    ```bash
    sudo systemctl enable foxglove_bridge.service
    ```

4.  **Start the service (optional):**
    ```bash
    sudo systemctl start foxglove_bridge.service
    ```

5.  **Check status:**
    ```bash
    sudo systemctl status foxglove_bridge.service
    ```
