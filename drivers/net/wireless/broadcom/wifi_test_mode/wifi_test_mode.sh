##############################################################################
#                                                                            #
#                         WIFI TEST SCRIPT.                                  #
#                                                                            #
#                                                                            #
##############################################################################
insmod /system/lib/modules/dhd.ko

firmware_path=/custpack/wifi_test/sdio-mfgtest-g-cdc-seqcmds.bin
nvram_path=/system/wlan/broadcom/nvram.txt
sleep 2

ifconfig eth0 up
sleep 0.1

chmod 777 /custpack/wifi_test/wl
sleep 0.1

/custpack/wifi_test/wl up
sleep 0.1

/custpack/wifi_test/wl mpc 0
sleep 0.1

/custpack/wifi_test/wl up
sleep 0.1

/custpack/wifi_test/wl PM 0
sleep 0.1

/custpack/wifi_test/wl channel 7
sleep 0.1

/custpack/wifi_test/wl rate 54
sleep 0.1

/custpack/wifi_test/wl rateset 54b
sleep 0.1

/custpack/wifi_test/wl txpwr 32
sleep 0.1

/custpack/wifi_test/wl txpwr1
sleep 0.1

/custpack/wifi_test/wl pkteng_start 00:11:22:33:44:55 tx 1000 1024 0

