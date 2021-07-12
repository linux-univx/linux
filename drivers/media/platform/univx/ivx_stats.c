// SPDX-License-Identifier: GPL-2.0
/*
 * ARM ISP IVX IQ Driver
 *
 * Copyright (C) 2020 RTAVS., Ltd.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>




/**
 * contains actual IQ calibration parameters for a given sensor/lens pair
 *
 **/


static struct v4l2_subdev soc_iq;



// initialize sub-device with give ops
v4l2_subdev_init(&soc_iq, &iq_ops);
// support direct access through /dev/
soc_iq.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
soc_iq.dev = &pdev->dev;
// register async subdev in the v4l2 framework
rc = v4l2_async_register_subdev(&soc_iq);
