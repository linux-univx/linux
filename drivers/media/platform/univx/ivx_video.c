// SPDX-License-Identifier: GPL-2.0
/*
 * MIPI Adapter cap Driver
 *
 * Copyright (C) 2020 RTAVS., Ltd.
 */

struct ivx_capture {
	struct isp_dev_t *isp_dev;
	struct isp_vdev_node vnode;
	enum isp_stream_id id;
	struct isp_capture_ops *ops;
	const struct isp_capture_config *config;
	bool is_streaming;
	bool is_stopping;
	wait_queue_head_t done;
	unsigned int sp_y_stride;
	uint32_t dma_addr[2];
	struct {
		/* protects queue, curr and next */
		spinlock_t lock;
		struct list_head queue;
		struct isp_dummy_buffer dummy;
		struct isp_buffer *curr;
		struct isp_buffer *next;
	} buf;
	struct {
		const struct isp_capture_fmt_cfg *cfg;
		struct v4l2_pix_format_mplane fmt;
		struct v4l2_pix_format_mplane p_fmt;
	} pix;
};
