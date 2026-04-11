# asne

ssh nvidia@192.168.55.1

sudo chmod +x /etc/rc.local
sudo systemctl enable rc-local

sudo usermod -a -G dialout $USER

https://www.digikey.com/en/products/detail/mh-connectors/MHCCOV-9-SC-LG/16983971?gclsrc=aw.ds&gad_source=1&gad_campaignid=17336967819&gbraid=0AAAAADrbLliTyznh3APuOw4qZ3tjLSulT&gclid=EAIaIQobChMI88j2r5rckwMVgVxHAR01JQx_EAQYAiABEgKoF_D_BwE

install gps drivers:
```bash
sudo apt install ros-humble-nmea-navsat-driver
```

can stuff:
```bash
pip install canopen python-can smbus2

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
