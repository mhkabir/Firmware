#!/bin/bash

if [ -z ${SKYCONTROLLER_IP+x} ]; then 
  ip=192.168.53.1
  echo "\$SKYCONTROLLER_IP is not set (use default: $ip)"
else 
  ip=$SKYCONTROLLER_IP
  echo "\$SKYCONTROLLER_IP is set to $ip"
fi
port=9050

echo "Connecting to Skycontroller: $ip:$port"

# adb returns also 0 as exit status if the connection fails
adb_return=$(adb connect $ip:$port)
adb_status=$(echo $adb_return | cut -f 1 -d " ")

if [[ $adb_status == "unable" ]]; then
  
  echo ""
  echo "Connection with the Skycontroller could not be established:"
  echo "  Make sure you are plugged in to the Skycontroller."
  echo ""
  exit 50

fi

echo "Connection successfully established"

sleep 1

adb shell mount -o remount,rw /
adb shell touch /home/root/parameters
adb shell mkdir -p /home/root/fs/microsd

${PX4_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabi/bin/arm-linux-gnueabi-strip \
  -R .comment -R .gnu.version \
  ../build_posix_skycontroller_default/src/firmware/posix/px4

../Tools/adb_upload.sh $@

echo "Disconnecting from Skycontroller"
adb disconnect
