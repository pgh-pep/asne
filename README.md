# asne


sudo apt install ros-humble-nmea-navsat-driver

pip install canopen python-can

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0