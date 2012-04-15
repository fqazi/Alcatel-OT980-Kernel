#!/system/bin/sh
#hciattach -c 1/42/75 /dev/ttyHS0 bcm2035 460800 flow
hciattach -f /dev/ttyHS0 bcm2035 460800 flow
hciconfig hci0 up
