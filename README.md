# Servo42c robot arm controller

## Overview

A ROS2 controller for a robot arm, utilizing SERVO42C v1.0 controllers for stepper motors.

At the moment only works with v.1.0 of protocol (no crc)

# TODO
- add IK via moveit
  - setup moveit config
  - moveit should control joints
  - moveit should be controllable via foxglove
- add calibration via accelerometers
  - add reset command for servos
  - add a package to query from acc
- fix faulty servo
- test manual acceleration control
- try basic velocity control for servo v.1.0
- test servo42c v.1.1
  - test basic protocol
  - test speed control
  - test acceleration control

## Autostart Foxglove Bridge on Boot (systemd)

To automatically launch the Foxglove Bridge when the system boots using `systemd` on Ubuntu 22.04 or similar:

1.  **Create a `systemd` service file:**
    ```bash
    sudo nano /etc/systemd/system/foxglove_bridge.service
    ```

2.  **Add the following content:**
    *Replace `lexa` with the correct username, `/home/lexa/app` with your workspace path if needed, and `humble` with your ROS 2 distribution.*

    ```ini
    [Unit]
    Description=Foxglove Bridge ROS 2 Launch Service
    After=network.target

    [Service]
    User=lexa 
    # WorkingDirectory=/home/lexa/app
    ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/lexa/app/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
    Restart=on-failure
    RestartSec=5

    [Install]
    WantedBy=multi-user.target
    ```
    *Note: The `source /home/lexa/app/install/setup.bash &&` part is only needed if the `foxglove_bridge` package or its dependencies are built locally in your workspace.*

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
