#!/bin/bash
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
#


#adnl oem "setenv bootargs init=/init console=ttyS0,921600n8 earlycon=meson,0xfe002000 loglevel=7;"
adnl oem "setenv bootargs init=/init console=ttyS0,921600n8 earlycon=meson,0xfe002000 initcall_debug=1 clk_ignore_unused loglevel=9;"
adnl Partition -M mem -P 0x10000000 -F image/Image;
adnl Partition -M mem -P 0x20000000 -F image/rootfs.cpio.gz.uboot;
adnl Partition -M mem -P 0x30000000 -F image/kernel.dtb;
adnl oem "booti 10000000 20000000 30000000"
