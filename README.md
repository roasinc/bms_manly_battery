# bms_manly_battery
ROS2 package for MANLY Li-Ion Battery

## Usage

```
$ ros2 run bms_manly_battery main_node --ros-args -p port_name:=/dev/ttyS1 -p baudrate:=9600 -p -r rate:=5
```

## Topics

```
sensor_msgs/msg/BatteryState.msg
```

