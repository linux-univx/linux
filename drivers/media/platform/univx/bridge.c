// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic MIPI CSI2 Driver
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


#define VFE_BASE0                       0xFF654800
#define VFE_BASE1                       0xFF654C00

//csi2-vfe:
    #define CSI2_CLK_RESET                      0x00
    #define CSI2_GEN_CTRL0                      0x04
    #define CSI2_GEN_CTRL1                      0x08
    #define CSI2_X_START_END_ISP                0x0C    //
    #define CSI2_Y_START_END_ISP                0x10    //
    #define CSI2_X_START_END_MEM                0x14    //
    #define CSI2_Y_START_END_MEM                0x18    //
    #define CSI2_VC_MODE                        0x1C
    #define CSI2_VC_MODE2_MATCH_MASK_L          0x20
    #define CSI2_VC_MODE2_MATCH_MASK_H          0x24
    #define CSI2_VC_MODE2_MATCH_TO_VC_L         0x28
    #define CSI2_VC_MODE2_MATCH_TO_VC_H         0x2C
    #define CSI2_VC_MODE2_MATCH_TO_IGNORE_L     0x30
    #define CSI2_VC_MODE2_MATCH_TO_IGNORE_H     0x34
    #define CSI2_DDR_START_PIX                  0x38
    #define CSI2_DDR_START_PIX_ALT              0x3C
    #define CSI2_DDR_STRIDE_PIX                 0x40
    #define CSI2_DDR_START_OTHER                0x44
    #define CSI2_DDR_START_OTHER_ALT            0x48
    #define CSI2_DDR_MAX_BYTES_OTHER            0x4C
    #define CSI2_INTERRUPT_CTRL_STAT            0x50

    #define CSI2_GEN_STAT0                      0x80
    #define CSI2_ERR_STAT0                      0x84
    #define CSI2_PIC_SIZE_STAT                  0x88
    #define CSI2_DDR_WPTR_STAT_PIX              0x8C
    #define CSI2_DDR_WPTR_STAT_OTHER            0x90
    #define CSI2_STAT_MEM_0                     0x94
    #define CSI2_STAT_MEM_1                     0x98

    #define CSI2_STAT_GEN_SHORT_08              0xA0
    #define CSI2_STAT_GEN_SHORT_09              0xA4
    #define CSI2_STAT_GEN_SHORT_0A              0xA8
    #define CSI2_STAT_GEN_SHORT_0B              0xAC
    #define CSI2_STAT_GEN_SHORT_0C              0xB0
    #define CSI2_STAT_GEN_SHORT_0D              0xB4
    #define CSI2_STAT_GEN_SHORT_0E              0xB8
    #define CSI2_STAT_GEN_SHORT_0F              0xBC

#define RD_BASE                             0xFF655000

    #define MIPI_ADAPT_DDR_RD0_CNTL0            0x00
    #define MIPI_ADAPT_DDR_RD0_CNTL1            0x04
    #define MIPI_ADAPT_DDR_RD0_CNTL2            0x08
    #define MIPI_ADAPT_DDR_RD0_CNTL3            0x0C
    #define MIPI_ADAPT_DDR_RD0_CNTL4            0x10
    #define MIPI_ADAPT_DDR_RD0_ST0              0x14
    #define MIPI_ADAPT_DDR_RD0_ST1              0x18
    #define MIPI_ADAPT_DDR_RD0_ST2              0x1C

    #define MIPI_ADAPT_DDR_RD1_CNTL0            0x40
    #define MIPI_ADAPT_DDR_RD1_CNTL1            0x44
    #define MIPI_ADAPT_DDR_RD1_CNTL2            0x48
    #define MIPI_ADAPT_DDR_RD1_CNTL3            0x4C
    #define MIPI_ADAPT_DDR_RD1_CNTL4            0x50
    #define MIPI_ADAPT_DDR_RD1_ST0              0x54
    #define MIPI_ADAPT_DDR_RD1_ST1              0x58
    #define MIPI_ADAPT_DDR_RD1_ST2              0x5C


#define PIXEL_BASE                          0xFF655000
    #define MIPI_ADAPT_PIXEL0_CNTL0             0x80
    #define MIPI_ADAPT_PIXEL0_CNTL1             0x84
    #define MIPI_ADAPT_PIXEL1_CNTL0             0x88
    #define MIPI_ADAPT_PIXEL1_CNTL1             0x8C


#define ALIGN_BASE                          0xFF655000
    #define MIPI_ADAPT_ALIG_CNTL0               0xC0
    #define MIPI_ADAPT_ALIG_CNTL1               0xC4
    #define MIPI_ADAPT_ALIG_CNTL2               0xC8
    #define MIPI_ADAPT_ALIG_CNTL6               0xD8
    #define MIPI_ADAPT_ALIG_CNTL7               0xDC
    #define MIPI_ADAPT_ALIG_CNTL8               0xE0

    #define MIPI_OTHER_CNTL0                    0x100
    #define MIPI_ADAPT_IRQ_MASK0                0x180
    #define MIPI_ADAPT_IRQ_PENDING0             0x184



#define MISC_BASE                           0xFF655000

    // CLK offsets.
    #define HHI_MIPI_ISP_CLK_CNTL               (0x70 << 2)
    #define HHI_MIPI_CSI_PHY_CLK_CNTL           (0xD0 << 2)
    #define HHI_CSI_PHY_CNTL0                   (0xD3 << 2)
    #define HHI_CSI_PHY_CNTL1                   (0x114 << 2)

    // Power domain.
    #define AO_RTI_GEN_PWR_SLEEP0               (0x3a << 2)
    #define AO_RTI_GEN_PWR_ISO0                 (0x3b << 2)

    // Memory PD.
    #define HHI_ISP_MEM_PD_REG0                 (0x45 << 2)
    #define HHI_ISP_MEM_PD_REG1                 (0x46 << 2)

    // Reset
    #define RESET4_LEVEL                        0x90








/*
 * This is a Synopsys MIPI CSI-2 host controller core
 *
 * v4l2_subdev bridge driver
 *
 */

#define DEVICE_NAME "amlogic-mipi-csi2"

#define CSI2_HOST_VERSION                    0x000
#define CSI2_HOST_N_LANES                    0x004
#define CSI2_HOST_PHY_SHUTDOWNZ              0x008
#define CSI2_HOST_DPHY_RSTZ                  0x00C
#define CSI2_HOST_CSI2_RESETN                0x010
#define CSI2_HOST_PHY_STAT                   0x014
#define CSI2_HOST_DATA_IDS_1                 0x018
#define CSI2_HOST_DATA_IDS_2                 0x01C
#define CSI2_HOST_ERR1                       0x020
#define CSI2_HOST_ERR2                       0x024
#define CSI2_HOST_MASK1                      0x028
#define CSI2_HOST_MASK2                      0x02C

#define PHY_TST_CRTL0                        0x030
#define PHY_TST_CRTL1                        0x034


/*
 * there must be 5 pads: 1 input pad from sensor, and
 * the 4 virtual channel output pads
 */
#define CSI2_SINK_PAD			0
#define CSI2_NUM_SINK_PADS		1
#define CSI2_NUM_SRC_PADS		4
#define CSI2_NUM_PADS			5
#define CSI2_NUM_PADS_SINGLE_LINK	2
#define MAX_CSI2_SENSORS		2

/*
 * The default maximum bit-rate per lane in Mbps, if the
 * source subdev does not provide V4L2_CID_LINK_FREQ.
 */
#define CSI2_DEFAULT_MAX_MBPS 849

#define IMX_MEDIA_GRP_ID_CSI2      BIT(8)
#define CSIHOST_MAX_ERRINT_COUNT	10

enum csi2_pads {
	CSI2_PAD_SINK = 0,
	CSI2X_PAD_SOURCE0,
	CSI2X_PAD_SOURCE1,
	CSI2X_PAD_SOURCE2,
	CSI2X_PAD_SOURCE3
};

struct csi2_match_data {
	int chip_id;
	int num_pads;
};

struct csi2_sensor {
	struct v4l2_subdev *sd;
	struct v4l2_mbus_config mbus;
	int lanes;
};

struct csi2_dev {
	struct device          *dev;
	struct v4l2_subdev      sd;
	struct media_pad       pad[CSI2_NUM_PADS];
	struct clk             *pix_clk; /* what is this? */
	void __iomem           *base;

	struct v4l2_async_notifier	notifier;
	struct v4l2_fwnode_bus_mipi_csi2 bus;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt format_mbus;

	int                     stream_count;

	struct v4l2_subdev      *src_sd;    // for camera sensor subdev
    struct phy              *dphy;      // csi dphy

	bool                    sink_linked[CSI2_NUM_SRC_PADS];
	struct csi2_sensor	sensors[MAX_CSI2_SENSORS];
	const struct csi2_match_data	*match_data;
	int num_sensors;
};




static inline void csi2_write(struct csi2_dev *csi2, u32 addr, u8 value)
{
	int ret;
    ret = writel(value, csi2->base + addr);
	if (ret < 0)
		dev_err(priv->dev, "Failed to write reg %d: %d\n", addr, ret);

	return ret;
}

static inline u32 csi2_read(struct csi2_dev *csi2, u32 addr)
{
    return readl(csi2->base + addr);
}


static inline struct csi2_dev *sd_to_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct csi2_dev, sd);
}

static struct csi2_sensor *sd_to_sensor(struct csi2_dev *csi2,
					struct v4l2_subdev *sd)
{
	int i;

	for (i = 0; i < csi2->num_sensors; ++i)
		if (csi2->sensors[i].sd == sd)
			return &csi2->sensors[i];

	return NULL;
}

static struct v4l2_subdev *get_remote_sensor(struct v4l2_subdev *sd)
{
	struct media_pad *local, *remote;
	struct media_entity *sensor_entity;

	local = &sd->entity.pads[CSI2_PAD_SINK];
	remote = media_entity_remote_pad(local);
	if (!remote) {
		v4l2_warn(sd, "No link between dphy and sensor\n");
		return NULL;
	}

	sensor_entity = media_entity_remote_pad(local)->entity;
	return media_entity_to_v4l2_subdev(sensor_entity);
}


static void csi2_enable(struct csi2_dev *csi2, bool enable)
{
	if (enable) {
		int lanes = csi2->bus.num_data_lanes;
		/* reset csi2rx_host */
		csi2_write(csi2, CSI2_RESETN, 0x0);
		//udelay(10);
		csi2_write(csi2, CSI2_RESETN, 0xffffffff);

		/* release dphy reset */
		csi2_write(csi2, CSI2_DPHY_RSTZ, 0xffffffff);
		/* init lanes num */
		csi2_write(csi2, CSI2_N_LANES, (lanes - 1) & 0x3);
		/* enable dphy power */
		csi2_write(csi2, CSI2_PHY_SHUTDOWNZ, 0xfffffff);
	} else {
		csi2_write(csi2, CSI2_PHY_SHUTDOWNZ, 0);
		csi2_write(csi2, CSI2_DPHY_RSTZ, 0);
		csi2_write(csi2, CSI2_RESETN, 0);
	}
}



#if 0

/*
 * use V4L2_CID_PIXEL_RATE/V4L2_CID_LINK_FREQ calc hsfreq and hs_settle
 * prefer cid_link_freq
 */

int csi2_get_link_freq(struct csi2_dev *csi2, u32 *link_freq)
{
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(csi2->src_sd->ctrl_handler,
			      V4L2_CID_LINK_FREQ);
	if (!ctrl)
		return -EINVAL;

	*link_freq = v4l2_ctrl_g_ctrl_int32(ctrl);
	return 0;
}

int csi2_get_pixel_clock(struct csi2_dev *csi2, u32 *pixel_clock)
{
	struct v4l2_ctrl *ctrl;

	// sensor subdev
	struct v4l2_subdev *subdev = csi2->src_sd;

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl)
		return -EINVAL;

	*pixel_clock = v4l2_ctrl_g_ctrl_int64(ctrl);

	return 0;
}

static int csi2_calc_max_mbps(struct csi2_dev *csi2, u32 *mbps_per_lane)
{
	struct v4l2_ctrl *ctrl;
	u32 link_freq;
	int ret;

	ret = csi2_get_link_freq(csi2, &link_freq);
	if (!ret && link_freq) {
		*mbps_per_lane = DIV_ROUND_UP_ULL(2 * link_freq, USEC_PER_SEC);
	} else {
		*mbps_per_lane = CSI2_DEFAULT_MAX_MBPS;
	}

	return 0;
}


/*
 * csi2_calc_settle_cnt - Calculate settle count value
 * @csi2: csi2 subdev
 *
 * Helper function to calculate settle count value. This is
 * based on the CSI2 T_hs_settle parameter which in turn
 * is calculated based on the CSI2 transmitter pixel clock
 * frequency.
 *
 * Return settle count value or 0 if the CSI2 pixel clock
 * frequency is not available
 */
static u8 csi2_calc_settle_cnt(struct csi2_dev *csi2)
{
	u32 pixel_clock; /* Hz */
	u32 mipi_clock; /* Hz */
	u32 ui; /* ps */
	u32 timer_period; /* ps */
	u32 t_hs_prepare_max; /* ps */
	u32 t_hs_prepare_zero_min; /* ps */
	u32 t_hs_settle; /* ps */
	u8 settle_cnt;
	int ret;

	// from V4L2_CID_LINK_FREQ
	ret = csi2_get_link_freq(csi2, &mipi_clock);
	if (!ret) {
		dev_err(csi2->dev, "Cannot get CSI2 transmitter's link freq\n");
		return 0;
	}
	if (!pixel_clock) {
		dev_err(csi2->dev, "Got pixel  == 0, cannot continue\n");
		return 0;
	}

	ui = div_u64(1000000000000, mipi_clock);

	ui /= 2;

	/*
	 * Minimum value: 85000 ps + 6 * @hs_clk_rate period in ps
	 * Maximum value: 145000 ps + 10 * @hs_clk_rate period in ps
	 */
	// hs_settle = 160ns --(85 ns + 6 * ui < hs_settle < 145 ns + 10 * ui)


	t_hs_prepare_max = 85000 + 6 * ui;
	t_hs_prepare_zero_min = 145000 + 10 * ui;
	t_hs_settle = (t_hs_prepare_max + t_hs_prepare_zero_min) / 2;

	// uint32_t settle = ((85 + 145 + (16 * UI) / 2) / 5;
	//timer_period = div_u64(1000000000000, csi2->timer_clk_rate) = 5;

	settle_cnt = t_hs_settle / 5; // 5 ns cycle_time

	return settle_cnt;
}

#endif


static int csi2_start(struct csi2_dev *csi2)
{
	int ret;

	ret = clk_prepare_enable(csi2->pix_clk);
	if (ret)
		return ret;

    phy_init(csi2->dphy)

    // hardcode clka + data lanes
    phy_set_mode_ext(csi2->dphy, PHY_MODE_MIPI_DPHY, 0);

	phy_power_on(csi2->dphy);

	csi2_enable(csi2, 1);

	// enable irq

	// enable subdev stream
	ret = v4l2_subdev_call(csi2->src_sd, video, s_stream, true);
	if (ret)
		goto err_disable_pclk;


	pr_debug("stream sd: %s\n", csi2->src_sd->name);

    return 0;
}

static void csi2_stop(struct csi2_dev *csi2)
{

	if (v4l2_subdev_call(csi2->src_sd, video, s_stream, false))
		dev_warn(csi2rx->dev, "Couldn't disable our subdev\n");

	phy_power_off(csi2->dphy);

	csi2_enable(csi2, 0);
	clk_disable_unprepare(csi2->pix_clk);

}


/** v4l2 subdev video ops */
static int csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi2_dev *csi2 = sd_to_dev(sd);
	int ret = 0;

	mutex_lock(&csi2->lock);

	dev_info(csi2->dev, "stream %s, src_sd: %p\n",
		enable ? "ON" : "OFF",
		csi2->src_sd);

	/*
	 * enable/disable streaming only if stream_count is
	 * going from 0 to 1 / 1 to 0.
	 */
	if (enable) {
		/*
		 * If we're not the first users, there's no need to
		 * enable the whole controller.
		 */
		if (!csi2->stream_count) {
			ret = csi2_start(csi2);
			if (ret)
				goto out;
		}
		csi2->stream_count++;
	} else {
		csi2->stream_count--;

		/*
		 * Let the last user turn off the loghts.
		 */
		if (!csi2->stream_count) {
			csi2_stop(csi2);
		}
	}

out:
	mutex_unlock(&csi2->lock);
	return ret;
}

static int csi2_g_mbus_config(struct v4l2_subdev *sd,
			      struct v4l2_mbus_config *mbus)
{
	struct v4l2_subdev *sensor_sd = get_remote_sensor(sd);
	int ret;

	ret = v4l2_subdev_call(sensor_sd, video, g_mbus_config, mbus);
	if (ret)
		return ret;

	return 0;
}


/** v4l2 subdev pad ops */
/* csi2 accepts all fmt/size from sensor */
static int csi2_get_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sensor = get_remote_sensor(sd);

	/*
	 * Do not allow format changes and just relay whatever
	 * set currently in the sensor.
	 */
	return v4l2_subdev_call(sensor, pad, get_fmt, NULL, fmt);
}

/** v4l2 media entity ops */

static int csi2_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct csi2_dev *csi2 = sd_to_dev(sd);
	struct v4l2_subdev *remote_sd;
	int ret = 0;

	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	mutex_lock(&csi2->lock);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (csi2->sink_linked[local->index - 1]) {
				ret = -EBUSY;
				goto out;
			}
			csi2->sink_linked[local->index - 1] = true;
		} else {
			csi2->sink_linked[local->index - 1] = false;
		}
	} else {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (csi2->src_sd) {
				ret = -EBUSY;
				goto out;
			}
			csi2->src_sd = remote_sd;
		} else {
			csi2->src_sd = NULL;
		}
	}

out:
	mutex_unlock(&csi2->lock);
	return ret;
}


/** mipi csi2 irq */

static irqreturn_t csirx_irq1_handler(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct csi2_dev *csi2 = sd_to_dev(dev_get_drvdata(dev));
	static int csi_err1_cnt;
	u32 val;

	val = csi2_read(CSI2_HOST_ERR1);
	if (val) {
		csi2_write(csi2, CSI2_HOST_ERR1, 0x0);
		if (++csi_err1_cnt > CSIHOST_MAX_ERRINT_COUNT) {
			v4l2_err(&csi2->sd, "mask csi2 host mask1!\n");
			csi2_write(csi2->base, CSI2_HOST_MASK1, 0xffffffff);
			csi_err1_cnt = 0;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t csirx_irq2_handler(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct csi2_dev *csi2 = sd_to_dev(dev_get_drvdata(dev));
	static int csi_err2_cnt;
	u32 val;

	val = csi2_read(CSI2_HOST_ERR2);
	if (val) {
		csi2_write(csi2, CSI2_HOST_ERR2, 0x0);
		if (++csi_err1_cnt > CSIHOST_MAX_ERRINT_COUNT) {
			v4l2_err(&csi2->sd, "mask csi2 host mask1!\n");
			csi2_write(csi2->base, CSI2_HOST_MASK2, 0xffffffff);
			csi_err1_cnt = 0;
		}
	}

	return IRQ_HANDLED;
}


#if 0
// v4l2 async api


static int csi2_async_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct csi2_dev *csi2;

	csi2 = container_of(notifier, struct csi2_dev, notifier);

	int pad;

	csi2->sensor.sd = subdev;
	pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;

	cif_dev->sensor.pad = pad;

	return 0;
}

static void csi2_async_unbind(struct v4l2_async_notifier *notifier,
					  struct v4l2_subdev *sd,
					  struct v4l2_async_subdev *asd)
{
	struct csi2_dev *csi2;

	csi2 = container_of(notifier, struct csi2_dev, notifier);

	phy_exit(csi2->dphy);
}

static int csi2_async_complete(struct v4l2_async_notifier *notifier)
{
	int ret;
	struct csi2_dev *dev;

	dev = container_of(notifier, struct csi2_dev, notifier);

	mutex_lock(&dev->media_dev.graph_mutex);

	ret = csi2_create_links(dev);
	if (ret < 0)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (ret < 0)
		goto unlock;

unlock:
	mutex_unlock(&dev->media_dev.graph_mutex);
	return ret;
}


static const struct v4l2_async_notifier_operations subdev_notifier_ops = {
	.bound = csi2_async_bound,
	.unbind = csi2_async_unbind,
	.complete = csi2_sync_complete,
};
#endif


static const struct v4l2_subdev_video_ops csi2_video_ops = {
	.g_mbus_config = csi2_g_mbus_config,
	.s_stream = csi2_s_stream,
};

static const struct v4l2_subdev_pad_ops csi2_pad_ops = {
	.get_fmt = csi2_get_set_fmt,
	.set_fmt = csi2_get_set_fmt,
};

static const struct v4l2_subdev_ops csi2_subdev_ops = {
	.video = &csi2_video_ops,
	.pad = &csi2_pad_ops,
};


static const struct media_entity_operations csi2_entity_ops = {
	.link_setup = csi2_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};


static int csi2_parse_endpoint(struct device *dev,
			       struct v4l2_fwnode_endpoint *ofep,
			       struct v4l2_async_subdev *asd)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct csi2_dev *csi2 = sd_to_dev(sd);

	if (!fwnode_device_is_available(asd->match.fwnode)) {
		v4l2_err(&csi2->sd, "remote is not available\n");
		return -EINVAL;
	}

	if (ofep->bus_type != V4L2_MBUS_CSI2_DPHY) {
		v4l2_err(&csi2->sd, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	csi2->bus = ofep->bus.mipi_csi2;

	dev_dbg(csi2->dev, "data lanes: %d\n", csi2->bus.num_data_lanes);
	dev_dbg(csi2->dev, "flags: 0x%08x\n", csi2->bus.flags);

	return 0;
}


static int csi2_get_resource(struct csi2_dev *csi2, struct platform_device *pdev)
{
	struct resource *res;
	int i, ret, irq;


	csi2->match_data = of_device_get_match_data(&pdev->dev);

	csi2->dphy = devm_phy_optional_get(&pdev->dev, "dphy");
	if (IS_ERR(csi2->dphy)) {
		dev_err(&pdev->dev, "Couldn't get external D-PHY\n");
		return PTR_ERR(csi2->dphy);
	}

	csi2->pllref_clk = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(csi2->pllref_clk)) {
		v4l2_err(&csi2->sd, "failed to get pll reference clock\n");
		return PTR_ERR(csi2->pllref_clk);
	}

	csi2->dphy_clk = devm_clk_get(&pdev->dev, "dphy");
	if (IS_ERR(csi2->dphy_clk)) {
		v4l2_err(&csi2->sd, "failed to get dphy clock\n");
		return PTR_ERR(csi2->dphy_clk);
	}

	csi2->pix_clk = devm_clk_get(&pdev->dev, "pix");
	if (IS_ERR(csi2->pix_clk)) {
		v4l2_err(&csi2->sd, "failed to get pixel clock\n");
		return PTR_ERR(csi2->pix_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		v4l2_err(&csi2->sd, "failed to get platform resources\n");
		return -ENODEV;
	}

	csi2->base = devm_ioremap(&pdev->dev, res->start, PAGE_SIZE);
	if (!csi2->base)
		return -ENOMEM;

	irq = platform_get_irq_byname(pdev, "csi-intr1");
	if (irq > 0) {
		ret = devm_request_irq(&pdev->dev, irq,
				       csirx_irq1_handler, 0,
				       dev_driver_string(&pdev->dev),
				       &pdev->dev);
		if (ret < 0)
			v4l2_err(&csi2->sd, "request csi-intr1 irq failed: %d\n",
				 ret);
	} else {
		v4l2_err(&csi2->sd, "No found irq csi-intr1\n");
	}

	irq = platform_get_irq_byname(pdev, "csi-intr2");
	if (irq > 0) {
		ret = devm_request_irq(&pdev->dev, irq,
				       csirx_irq2_handler, 0,
				       dev_driver_string(&pdev->dev),
				       &pdev->dev);
		if (ret < 0)
			v4l2_err(&csi2->sd, "request csi-intr2 failed: %d\n",
				 ret);
	} else {
		v4l2_err(&csi2->sd, "No found irq csi-intr2\n");
	}

	ret = clk_prepare_enable(csi2->pllref_clk);
	if (ret) {
		v4l2_err(&csi2->sd, "failed to enable pllref_clk\n");
		goto rmmutex;
	}

	ret = clk_prepare_enable(csi2->dphy_clk);
	if (ret) {
		v4l2_err(&csi2->sd, "failed to enable dphy_clk\n");
		goto pllref_off;
	}


	return ret;
}


static int csi2_parse_dt(struct csi2_dev *csi2)
{
	int ret;

	return ret;
}

static int csi2_probe(struct platform_device *pdev)
{
	unsigned int sink_port = 0;
	struct csi2_dev *csi2;
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int i, ret, irq;

	csi2 = devm_kzalloc(&pdev->dev, sizeof(*csi2), GFP_KERNEL);
	if (!csi2)
		return -ENOMEM;

	platform_set_drvdata(pdev, &csi2);
	csi2->dev = &pdev->dev;

	mutex_init(&csi2->lock);

	ret = csi2_get_resource(csi2, pdev);
	if (ret) {
		goto rmmutex;
	}

	v4l2_subdev_init(&csi2->sd, &csi2_subdev_ops);
	v4l2_set_subdevdata(&csi2->sd, &pdev->dev);

	csi2->sd.entity.ops = &csi2_entity_ops;

	csi2->sd.dev = &pdev->dev;
	csi2->sd.owner = THIS_MODULE;
	csi2->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(csi2->sd.name, DEVICE_NAME, sizeof(csi2->sd.name));
	csi2->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	//csi2->sd.grp_id = IMX_MEDIA_GRP_ID_CSI2;


	for (i = 0; i < CSI2_NUM_PADS; i++) {
		csi2->pad[i].flags = (i == CSI2_SINK_PAD) ?
						MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	}

	ret = media_entity_pads_init(&csi2->sd.entity, CSI2_NUM_PADS,
				     csi2->pad);
	if (ret)
		goto rmmutex;

	ret = v4l2_async_register_fwnode_subdev(
					&csi2->sd, sizeof(struct v4l2_async_subdev),
					&sink_port, 1, csi2_parse_endpoint);
	if (ret)
		goto rmmutex;

	return 0;

rmmutex:
	mutex_destroy(&csi2->lock);
	return ret;
}

static int csi2_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csi2_dev *csi2 = sd_to_dev(sd);

	v4l2_async_unregister_subdev(sd);
	clk_disable_unprepare(csi2->dphy_clk);
	clk_disable_unprepare(csi2->pllref_clk);
	mutex_destroy(&csi2->lock);
	media_entity_cleanup(&sd->entity);

	return 0;
}


static const struct csi2_match_data a311d_csi2_match_data = {
	.num_pads = CSI2_NUM_PADS
};


static const struct of_device_id csi2_dt_ids[] = {
	{ .compatible = "amlogic,a311d-mipi-csi2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, csi2_dt_ids);

static struct platform_driver csi2_driver = {
	.probe = csi2_probe,
	.remove = csi2_remove,

	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = csi2_dt_ids,
	},
};

module_platform_driver(csi2_driver);

MODULE_DESCRIPTION("Amlogic MIPI CSI-2 Receiver driver");
MODULE_AUTHOR("Kaspter Ju <camus@rtavs.com>");
MODULE_LICENSE("GPL");
