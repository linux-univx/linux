// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Amlogic MIPI RX Analog PHY driver
 *
 * Copyright (C) 2021 RTAVS, Ltd.
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 */

#ifndef __GDC_REGS__
#define __GDC_REGS__

/*
regbase: 0xff950000

0x00: GDC_API
0x04: GDC_PRODUCT
0x08: GDC_VERSION
0x0C: GDC_REVISION

0x10: GDC_CONFIG_ADDR
0x14: GDC_CONFIG_SIZE

0x20: GDC_DATA_IN_WIDTH
0x24: GDC_DATA_IN_HEIGHT


// for y + uv plane or  y + u + v plane

0x28: GDC_DATA_IN_ADDR_1        // y plane base
0x2c: GDC_DATA_IN_LINE_1        // y plane offset

0x30: GDC_DATA_IN_ADDR_2        // u / uv plane base
0x34: GDC_DATA_IN_LINE_2        // u / uv plane offset

0x38: GDC_DATA_IN_ADDR_3        // v plane
0x3c: GDC_DATA_IN_ADDR_3        // v plane


0x40: GDC_DATA_OUT_WIDTH
0x44: GDC_DATA_OUT_HEIGHT


0x60: GDC_STATUS
  BIT(0): busy
  BIT(1): error
  BIT(2)

0x64: GDC_CMD
 BIT(0): start
 BIT(1): stop

0x68: GDC_FMT
 BIT(0): 8bits
 BIT(1): 10bits
 BIT(2): grayscale
 BIT(3): rgba888/yuv444
 BIT(4):
 ....


0x70: GDC_CH1_DATA
0x74: GDC_CH2_DATA
0x78: GDC_CH3_DATA


0x80: GDC_DIAG_STALL_0
0x84: GDC_DIAG_STALL_1
0x88: GDC_DIAG_STALL_2
0x8c: GDC_DIAG_STALL_3
0x90: GDC_DIAG_STALL_4

0x94: GDC_DIAG_STALL_READ_INT

0x98: GDC_DIAG_STALL_COORD_INT
0x9c:

0xa0:

*/

#endif /* __GDC_REGS__ */
