#!/system/bin/sh
#start WIFI RX test. 

ch=${1}

/system/wifi_test/wl down
/system/wifi_test/wl up
/system/wifi_test/wl channel ${ch}
/system/wifi_test/wl pktcnt
