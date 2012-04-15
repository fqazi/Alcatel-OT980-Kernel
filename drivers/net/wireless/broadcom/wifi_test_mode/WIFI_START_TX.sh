#!/system/bin/sh
#start WIFI TX test.
rate=${1}
ch=${2}

/system/wifi_test/wl up
/system/wifi_test/wl mpc 0
/system/wifi_test/wl up
/system/wifi_test/wl PM 0
/system/wifi_test/wl channel ${ch}
/system/wifi_test/wl rate ${rate}
/system/wifi_test/wl rateset 54b
/system/wifi_test/wl txpwr 32
/system/wifi_test/wl txpwr1
/system/wifi_test/wl pkteng_start 00:11:22:33:44:55 tx 1000 1024 0
