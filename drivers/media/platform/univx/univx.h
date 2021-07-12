/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * ARM UNIVX ISP Driver
 *
 * Copyright (C) 2020 RTAVS., Ltd.
 */

#ifndef _UNIVX_H
#define _UNIVX_H

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
 * struct ivx_stream - All internal data for one instance of ISP
 */
struct ivx_stream {
    /* Control fields */
    uint32_t ctx_id;
    int stream_id;
    isp_v4l2_stream_type_t stream_type;
    int stream_started;
    uint32_t last_frame_id;

    /* Input stream */
    isp_v4l2_stream_common *stream_common;

    /* Stream format */
    struct v4l2_format cur_v4l2_fmt;

    /* Video buffer field*/
    struct list_head stream_buffer_list;
    struct list_head stream_buffer_list_busy;
    spinlock_t slock;

    /* Temporal fields for memcpy */
    struct task_struct *kthread_stream;
#if ISP_HAS_META_CB
    atomic_t running; //since metadata has no thread for syncing
#endif
    isp_fw_frame_mgr_t frame_mgr;
    int fw_frame_seq_count;
    struct vb2_queue* vb2_q;
};

#endif /* _UNIVX_H */
