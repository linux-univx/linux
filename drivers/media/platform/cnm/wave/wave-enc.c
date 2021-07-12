


/**
 * #define V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP		(V4L2_CID_MPEG_BASE+350)
 * #define V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP		(V4L2_CID_MPEG_BASE+351)
 * #define V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP		(V4L2_CID_MPEG_BASE+352)
 * #define V4L2_CID_MPEG_VIDEO_H264_MIN_QP			(V4L2_CID_MPEG_BASE+353)
 * #define V4L2_CID_MPEG_VIDEO_H264_MAX_QP			(V4L2_CID_MPEG_BASE+354)
 * #define V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM		(V4L2_CID_MPEG_BASE+355)
 * #define V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE		(V4L2_CID_MPEG_BASE+356)
 * #define V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE		(V4L2_CID_MPEG_BASE+357)
 */


struct wave_format {
	unsigned int v4l2_pix_fmt;
	unsigned int num_planes;
	unsigned int min_width;
	unsigned int max_width;

	unsigned int min_height;
	unsigned int max_height;
};


static const wave_format wave_enc_src_fmts = {
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_YUV420,
        .num_planes   = 1,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_NV12,
        .num_planes   = 1,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_NV21,
        .num_planes   = 1,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_YUV420M,
        .num_planes   = 3,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_NV12M,
        .num_planes   = 2,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_NV21M,
        .num_planes   = 2,
        .max_width    = 8192,
        .min_width    = 8,
        .max_height   = 8192,
        .min_height   = 8,
    },
};

static const wave_format wave_enc_dst_fmts = {
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_HEVC,
        .num_planes   = 1,
        .min_width    = 8,
        .max_width    = 8192,
        .min_height   = 8,
        .max_height   = 8192,
    },
    {
        .v4l2_pix_fmt = V4L2_PIX_FMT_H264,
        .num_planes   = 1,
        .min_width    = 32,
        .max_width    = 8192,
        .min_height   = 32,
        .max_height   = 8192,
    },
};



static inline struct wave_ctx *ctrl_to_wave_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct wave_ctx, v4l2_ctrl_hdl);
}

static int wave_venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	const char * const *val_names = v4l2_ctrl_get_menu(ctrl->id);
	struct wave_ctx *ctx = ctrl_to_wave_ctx(ctrl);

	if (val_names)
		wave_dbg(2, ctx, "s_ctrl: id = 0x%x, name = \"%s\", val = %d (\"%s\")\n",
			 ctrl->id, ctrl->name, ctrl->val, val_names[ctrl->val]);
	else
		wave_dbg(2, ctx, "s_ctrl: id = 0x%x, name = \"%s\", val = %d\n",
			 ctrl->id, ctrl->name, ctrl->val);


	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			ctx->params.rot_mode |= WAVE_MIR_HOR;
		else
			ctx->params.rot_mode &= ~WAVE_MIR_HOR;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			ctx->params.rot_mode |= WAVE_MIR_VER;
		else
			ctx->params.rot_mode &= ~WAVE_MIR_VER;
		break;
	case V4L2_CID_ROTATE:
        ctx->params.rot_angle = ctrl->val;

	/* h264/h265 common param */
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		if (ctx->params.bitrate != ctrl->val / 1000)
			ctx->params.bitrate_changed = true;
		ctx->params.bitrate = ctrl->val / 1000;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->params.gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		ctx->params.frame_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		ctx->params.mb_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->params.force_ipicture = true;
		break;

	case V4L2_CID_MPEG_VIDEO_VBV_DELAY:
		ctx->params.vbv_delay = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VBV_SIZE:
		ctx->params.vbv_size = min(ctrl->val * 8192, 0x7fffffff);
		break;


	/* h265 param */
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
			ctx->params.profile   = HEVC_PROFILE_MAIN;
			ctx->params.bit_depth = 8;
			break;
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
			ctx->params.profile   = HEVC_PROFILE_STILLPICTURE;
			ctx->params.bit_depth = 8;
			break;
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
			ctx->params.profile   = HEVC_PROFILE_MAIN10;
			ctx->params.bit_depth = 10;
			break;
		default:
			ctx->params.profile   = 0;
			ctx->params.bit_depth = 0;
			break;
		}
		break;

	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
			inst->level = 10 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
			inst->level = 20 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
			inst->level = 21 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
			inst->level = 30 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
			inst->level = 31 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
			inst->level = 40 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
			inst->level = 41 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
			inst->level = 50 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
			inst->level = 51 * 3;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2:
			inst->level = 52 * 3;
			break;
		default:
			inst->level = 0;
			break;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		inst->min_qp_i = ctrl->val;
		inst->min_qp_p = ctrl->val;
		inst->min_qp_b = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		inst->max_qp_i = ctrl->val;
		inst->max_qp_p = ctrl->val;
		inst->max_qp_b = ctrl->val;
		break;






    /* h264 param */
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		ctx->params.profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		ctx->params.level = ctrl->val;
		break;

	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		if (ctx->params.h264_intra_qp != ctrl->val)
			ctx->params.h264_intra_qp_changed = true;
		ctx->params.h264_intra_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctx->params.h264_inter_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		ctx->params.h264_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctx->params.h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		ctx->params.h264_slice_alpha_c0_offset_div2 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		ctx->params.h264_slice_beta_offset_div2 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		ctx->params.h264_disable_deblocking_filter_idc = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CONSTRAINED_INTRA_PREDICTION:
		ctx->params.h264_constrained_intra_pred_flag = ctrl->val;
		break;

	case V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET:
		ctx->params.h264_chroma_qp_index_offset = ctrl->val;
		break;


	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctx->params.slice_mode = ctrl->val;
		ctx->params.slice_mode_changed = true;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->params.slice_max_mb = ctrl->val;
		ctx->params.slice_mode_changed = true;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
		ctx->params.slice_max_bits = ctrl->val * 8;
		ctx->params.slice_mode_changed = true;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		break;
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB:
		ctx->params.intra_refresh = ctrl->val;
		ctx->params.intra_refresh_changed = true;
		break;


	default:
		wave_dbg(1, ctx, "Invalid control, id=%d, val=%d\n",
			 ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return 0;

}

static const struct v4l2_ctrl_ops wave_venc_ctrl_ops = {
	.s_ctrl = wave_venc_s_ctrl,
	//.g_volatile_ctrl = wave_venc_g_volatile_ctrl,
};


// hw default param
// rc default param
// vui param
// setup ctu qp
// setup roi region
// setup RDO param
