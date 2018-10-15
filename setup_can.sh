sudo ifconfig can0 down
sudo ip link set can0 type can restart-ms 100
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up