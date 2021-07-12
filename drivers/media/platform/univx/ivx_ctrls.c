


struct camera_info {
    int facing;
    int orientation;
};

struct ivx_ae {

};

struct ivx_af {

};

struct ivx_awb {

};

struct ivx_camera {

};

struct ivx_context {
    uint32_t    ivx_id;

    struct ivx_ae ae_param;
    struct ivx_af af_param;
    struct ivx_awb awb_param;

    struct ivx_sensor sensor;

};



void ivx_ctrls_run(struct ivx_context *ivx, struct v4l2_event *event)
{
    switch(event->id) {
	case V4L2_CID_BRIGHTNESS:
		isp_s_brightness(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_CONTRAST:
		isp_s_contrast(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_SATURATION:
		isp_s_saturation(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_HUE:
		isp_s_hue(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		isp_s_auto_white_balance(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_EXPOSURE:
		isp_s_exposure(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTOGAIN:
		isp_s_auto_gain(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_GAIN:
		isp_s_gain(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		isp_s_power_line_frequency(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		isp_s_white_balance_temperature(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_SHARPNESS:
		isp_s_sharpness(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTOBRIGHTNESS:
		isp_s_auto_brightness(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_BAND_STOP_FILTER:
		isp_s_band_stop_filter(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_ILLUMINATORS_1:
		isp_s_illuminators_1(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_ILLUMINATORS_2:
		isp_s_illuminators_2(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		isp_s_exposure_auto(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		isp_s_exposure_absolute(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		isp_s_focus_absolute(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_FOCUS_RELATIVE:
		isp_s_focus_relative(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		isp_s_focus_auto(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		isp_s_auto_exposure_bias(ivx, exp_bias_qmenu[event->u.ctrl.value]);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		isp_s_auto_n_preset_white_balance(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_ISO_SENSITIVITY:
		isp_s_iso_sensitivity(ivx, iso_qmenu[event->u.ctrl.value]);
		break;
	case V4L2_CID_ISO_SENSITIVITY_AUTO:
		isp_s_iso_sensitivity_auto(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_EXPOSURE_METERING:
		isp_s_ae_metering_mode(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_SCENE_MODE:
		isp_s_scene_mode(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_3A_LOCK:
		//isp_s_3a_lock(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTO_FOCUS_START:
		isp_s_auto_focus_start(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTO_FOCUS_STOP:
		isp_s_auto_focus_stop(ivx, event->u.ctrl.value);
		break;
	case V4L2_CID_AUTO_FOCUS_RANGE:
		isp_s_auto_focus_range(ivx, event->u.ctrl.value);
		break;
	default:
		ISP_ERR("Unknown ctrl.\n");
		break;
	}
}
