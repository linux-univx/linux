// SPDX-License-Identifier: (GPL-2.0+ OR MIT)

#ifndef __WAVE_H__
#define __WAVE_H__

#include <linux/debugfs.h>
#include <linux/idr.h>
#include <linux/irqreturn.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/videodev2.h>
#include <linux/ratelimit.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>

#include "wave-regs.h"

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum wave_inst_type {
	WAVE_INST_ENCODER,
	WAVE_INST_DECODER,
};

enum wave_product {
	WAVE_511  = 0x5110,
	WAVE_512  = 0x5120,
	WAVE_515  = 0x5150,
	WAVE_517  = 0x5170,
	WAVE_520  = 0x5200,
	WAVE_521  = 0x5210,
	WAVE_521C = 0x521c,
	WAVE_521D = 0x521d, // wave521 dual core
	WAVE_525  = 0x5250,
};

struct wave_codec {
	u32 mode;
	u32 src_fourcc;
	u32 dst_fourcc;
	u32 max_w;
	u32 max_h;
};

enum wave_brc_type {
	BRC_TYPE_NONE = 0,
	BRC_TYPE_CBR = 1,
	BRC_TYPE_VBR = 2,
	BRC_TYPE_VBR_LOW_DELAY = 3,
};

struct wave_video_device;

struct wave_match_data {
	char *firmware;
	enum wave_product product;

	const struct wave_codec *codecs;
	unsigned int num_codecs;

	const struct wave_video_device **vdevs;
	unsigned int num_vdevs;

	const char * const *clks;
	unsigned int num_clks;

	size_t workbuf_size;
	size_t tempbuf_size;
	size_t iram_size;
};



//#define WAVE521DEC_WORKBUF_SIZE             (1.5*1024*1024)

/**
 * struct wave_buffer - wave buffer
 *
 * @name:  name of requester
 * @paddr: physical address (for hardware)
 * @vaddr: virtual address (kernel can read/write)
 * @size:  size of buffer
 */

struct wave_aux_buffer {
	dma_addr_t paddr;
	void *vaddr;
	size_t size;
	struct list_head head;
};

#define WAVE_MAX_INSTANCES	4
#define WAVE_MAX_BUS_CLK	5

struct wave_dev {
	struct v4l2_device v4l2_dev;
	struct video_device vfd[6];
	struct device *dev;
	const struct wave_match_data *variant;
	int firmware;

	/* read from codec IP */
	enum wave_product product;

	void __iomem *regs_base;

	unsigned int num_clks;
	struct clk_bulk_data clks[WAVE_MAX_BUS_CLK];

	struct reset_control *rstc;

	struct wave_aux_buffer codebuf;
	struct wave_aux_buffer tempbuf;
	struct wave_aux_buffer workbuf;

	struct gen_pool *sram_pool;
	struct wave_aux_buffer iram;

	struct mutex dev_mutex;
	struct mutex wave_mutex;
	struct workqueue_struct *workqueue;
	struct v4l2_m2m_dev *m2m_dev;
	struct ida ida;
	struct dentry *debugfs_root;
	struct ratelimit_state mb_err_rs;
};


// runtime param and static param

struct wave_enc_params {

};

struct wave_dec_params {
};

/* common h264/h265 encode param */
struct wave_params {
	u8 profile;
	u8 level;

	u8 min_qp_i;
	u8 min_qp_p;
	u8 min_qp_b;

	u8 max_qp_i;
	u8 max_qp_p;
	u8 max_qp_b;

	u8 rot_mode;	/* hflip or vflip*/
	u16 rot_angle;	/* rotate angle */


	/* s_param  */
	u32 framerate;
	bool framerate_changed;

	/* s_ctrl */
	u16 bitrate;
	bool bitrate_changed;

	u8 gop_size;
	bool gop_size_changed;

	int intra_refresh;
	bool intra_refresh_changed;

	u8 disable_deblocking_filter_idc;
	s8 slice_alpha_c0_offset_div2;
	s8 slice_beta_offset_div2;
	bool constrained_intra_pred_flag;
	s8 chroma_qp_index_offset;


	//struct wave_huff_tab	*jpeg_huff_tab;
	int codec_mode;
	int codec_mode_aux;

	enum v4l2_mpeg_video_multi_slice_mode slice_mode;
	bool slice_mode_changed;

	u16 vbv_delay;
	u32 vbv_size;
	u32 slice_max_bits;
	u32 slice_max_mb;

	bool force_ipicture;

	bool h264_intra_qp_changed;


	bool frame_rc_enable;
	bool mb_rc_enable;
};


struct wave_ctx_ops {
	int (*queue_init)(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq);
	int (*reqbufs)(struct wave_ctx *ctx, struct v4l2_requestbuffers *rb);
	int (*start_streaming)(struct wave_ctx *ctx);
	int (*prepare_run)(struct wave_ctx *ctx);
	void (*finish_run)(struct wave_ctx *ctx);
	void (*run_timeout)(struct wave_ctx *ctx);
	void (*seq_init_work)(struct work_struct *work);
	void (*seq_end_work)(struct work_struct *work);
	void (*release)(struct wave_ctx *ctx);
};


/* wave encode/decode instance ctx */
struct wave_ctx {
	struct wave_dev *dev;
	struct list_head list;

	struct v4l2_fh                 v4l2_fh;
	struct v4l2_ctrl_handler       v4l2_ctrl_hdl;

	struct wave_params params;

	const struct wave_ctx_ops	*ops;

};
