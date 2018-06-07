#/bin/bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
#make imx_v6_v7_defconfig
#make imx_v7_android_defconfig
#make zImage -j8 KCFLAGS=-mno-android
cp config_linux .config
#cp config_android .config
make dtbs
make zImage -j8
cp arch/arm/boot/dts/e9v2-sabresd.dtb ./
cp arch/arm/boot/dts/e9v3-sabresd.dtb ./
cp arch/arm/boot/dts/imx6q-sabresd.dtb ./
cp arch/arm/boot/zImage ./

