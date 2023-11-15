HOST_IP=192.168.98.44/24
INTERFACE=enp7s0f0
sudo ip a add $HOST_IP dev $INTERFACE
sudo ip link set dev $INTERFACE up
