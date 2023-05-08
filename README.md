## Setup notes

 - Add the user to the `dialout` and `tty` groups so serial devices can be accessed without root:

`sudo usermod -a -G dialout $USER`

`sudo usermod -a -G tty $user`

`sudo chmod a+rw /dev/ttyS0`

 - Reboot

 - `rtklib` is needed for calculating RTK corrections and `open-iscsi` to provide us with a unique UUID (yes there are better ways of doing that)

`sudo apt install rtklib open-iscsi`

 - Change file permissions so it can be run without root:

```
sudo chmod +r /etc/iscsi/initiatorname.iscsi
```

 - Start the `rtkrcv` process:

```
tmux new-session -d -s "rtkrcv" 'bash -ic "python3 rtklib.py 11feb-linux.conf" || bash && bash'
```

 - (Changing the serial device as appropriate) then to compile:

```
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select telemetry
. install/setup.bash
```

 - Set fastdds discovery server environment variable: `export ROS_DISCOVERY_SERVER=10.13.13.4:11811`
 - Set domain: `export ROS2_DOMAIN_ID=142`

 - To run:

`ros2 run telemetry telemetry`

 - To read publisher:

```
ros2 topic list
ros2 topic echo rtk_top
```

(Or just use one of the launch files...)

## `robot_localization` install notes

You might need to install:
`sudo apt install libgeographic-dev ros-humble-geographic-msgs ros-humble-diagnostic-updater

