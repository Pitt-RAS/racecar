#!/usr/bin/env bash

if [ -z ${ARDUINO_PATH} ];
then
    echo "Set ARDUINO_PATH to point to your Arduino install."
    exit 1
fi

pushd $(dirname $0)

${ARDUINO_PATH}/arduino-builder \
    -compile \
    -logger=machine \
    -hardware ${ARDUINO_PATH}/hardware \
    -tools ${ARDUINO_PATH}/tools-builder \
    -tools ${ARDUINO_PATH}/hardware/tools/avr \
    -built-in-libraries ${ARDUINO_PATH}/libraries \
    -libraries ~/Arduino/libraries \
    -fqbn=teensy:avr:teensy31:usb=serial,speed=96,opt=o2std,keys=en-us \
    -verbose \
    firmware/magellan_controller/magellan_controller.ino

popd
