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

 - Run the launch file:

`ros2 launch telemetry.launch.py`

It is recommended to do this in a tmux container. 

## Sending telemetry to server

Because the ROS server expects messages at a specific ROS domain ID, we need to have another
'session' in order to send the data to the server, so if you want to send data to the server, which is optional, repeat the previous steps (perhaps in another `tmux` container),
then:

 - Set fastdds discovery server environment variable: `export ROS_DISCOVERY_SERVER=10.13.13.4:11811`
 - Set domain: `export ROS2_DOMAIN_ID=142`

 - To run:

`ros2 run telemetry telemetry`

## Debugging RTKLIB

RTKLIB has a telnet command-line debugging tool which is nice for seeing information about satellites etc. It requires a password which is randomly set every launch. To get the password:

`tmux attach -t rtkrcv`

Then copy (`CTRL+C`) to your clipboard the password that's there. It is recommended to use another `tmux` container for attaching to the telnet console, because it appears that quitting the telnet console also stops the RTKRCV process (annoyingly):

`tmux new -s telnet`

And in that session:

`telnet 127.0.0.1 2120`

Paste in the password you copied earlier. Nice RTKRCV commands include `satellite` to show the connected satellites and `status` to see if it has a localization estimate.

## `robot_localization` install notes

You might need to install:
`sudo apt install libgeographic-dev ros-humble-geographic-msgs ros-humble-diagnostic-updater`

