#!/bin/bash
pkill pppd
# setup uart1 pin
# # GPIO1 D2 RMIO30
iomux 1 26 16
# # GPIO1 C3 RMIO28
iomux 1 19 17
# setup esp32c3 reset pin

#echo 57 > /sys/class/gpio/export
#echo out > /sys/class/gpio/gpio57/direction
#echo 1 > /sys/class/gpio/gpio57/value
#echo 0 > /sys/class/gpio/gpio57/value
#sleep 0.8
#echo 1 > /sys/class/gpio/gpio57/value
#echo 57 > /sys/class/gpio/unexport
./reset-esp.py
sleep 2
pppd -d /dev/ttyS1 921600

for i in {1..10}
do
    if ping -c 1 10.0.5.1 &> /dev/null
    then
       telnet 10.0.5.1 <<EOF
connect wifi-ssid wifi-password
EOF

       route add default gw 10.0.5.1
       echo 'nameserver 223.5.5.5' > /etc/resolv.conf
       echo "done"
       break
    fi
    sleep 1
done
