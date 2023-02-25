## Setup notes

 - Add the user to the `dialout` group so serial devices can be accessed without root:

`sudo usermod -a -G dialout $USER`

 - Reboot

 - `rtklib` is needed for calculating RTK corrections and `open-iscsi` to provide us with a unique UUID (yes there are better ways of doing that)

`sudo apt install rtklib open-iscsi`

 - Change file permissions so it can be run without root:

```
sudo chmod +r /etc/iscsi/initiatorname.iscsi
```

 - (Changing the serial device as appropriate) then to compile:

```
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select telemetry
. install/setup.bash
```

 - To run:

`ros2 run telemetry rtkrcv`

 - To read publisher:

```
ros2 topic list
ros2 topic echo rtk_top
```