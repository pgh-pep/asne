# asne

install gps drivers:
```bash
sudo apt install ros-humble-nmea-navsat-driver
```

can stuff:
```bash
pip install canopen python-can pyserial

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

generate urdf:
```bash
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/src/asne/asne_pkg/urdf/single_thrust_component_config.yaml thruster_yaml:=`pwd`/src/asne/asne_pkg/urdf/single_thrust_thruster_config.yaml wamv_target:=`pwd`/src/asne/asne_pkg/urdf/wamv_target.urdf wamv_locked:=False
```

run simulation:
```bash

```
