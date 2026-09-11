# sudo ifconfig canfd0 down
# sudo pkill slcand

sudo slcand -o -c -s8 /dev/ttyACM0 canfd0
sudo ifconfig canfd0 up
sudo ifconfig canfd0 txqueuelen 1000

# ip link set canfd0 type can bitrate 1000000

ip -details link show canfd0
