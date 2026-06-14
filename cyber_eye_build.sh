#!/bin/bash

cd /home/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/
source env_install_toolchain.sh

make -C /home/media/samples/simple_test ARCH=arm CROSS_COMPILE=arm-rockchip830-linux-uclibcgnueabihf- cyber_eye_bind

cp /home/media/samples/simple_test/cyber_eye_bind /home/project/cfg/BoardConfig_IPC/overlay/custom-overlay/oem/usr/bin