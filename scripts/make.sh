#!/bin/bash
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
#


export CROSS_COMPILE=/misc/opt/toolkit/toolchain/gcc-musl/bin/aarch64-buildroot-linux-musl-

ARCH=arm64 make -j8 meson64_linux_defconfig all

cp -rvf arch/arm64/boot/Image image/
cp -rvf arch/arm64/boot/dts/amlogic/meson-c2-c305x-turbo.dtb image/kernel.dtb
