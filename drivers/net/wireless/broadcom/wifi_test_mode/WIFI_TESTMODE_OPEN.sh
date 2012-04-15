#!/system/bin/sh

insmod /system/lib/modules/dhd.ko "firmware_path=/system/wifi_test/sdio-mfgtest-g-cdc-seqcmds.bin nvram_path=/system/wlan/broadcom/nvram.txt"
sleep 3
ifconfig eth0 up
