// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wave multi-standard codec IP
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gcd.h>
#include <linux/genalloc.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <linux/ratelimit.h>
#include <linux/reset.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "wave.h"

#define WAVE_NAME	"wave"

int wave_debug;
module_param(wave_debug, int, 0644);
MODULE_PARM_DESC(wave_debug, "Debug level (0-2)");

void wave_write(struct wave_dev *dev, u32 data, u32 reg)
{
	v4l2_dbg(3, wave_debug, &dev->v4l2_dev, "%s: data=0x%x, reg=0x%x\n",
		 __func__, data, reg);
	writel(data, dev->regs_base + reg);
}

unsigned int wave_read(struct wave_dev *dev, u32 reg)
{
	u32 data;

	data = readl(dev->regs_base + reg);
	v4l2_dbg(3, wave_debug, &dev->v4l2_dev, "%s: data=0x%x, reg=0x%x\n",
		 __func__, data, reg);
	return data;
}

static inline int wave_initialized(struct wave_dev *dev)
{
	return wave_read(dev, WAVE_REG_VCPU_CUR_PC) != 0;
}

static inline unsigned long wave_busy(struct wave_dev *dev)
{
	return wave_read(dev, WAVE_REG_BUSY_STATUS);
}

static int wave_wait_timeout(struct wave_dev *dev)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	while (wave_busy(dev)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}
	return 0;
}

#if 0
int wave_hw_reset(struct wave_ctx *ctx)
{
	struct wave_dev *dev = ctx->dev;
	unsigned long timeout;
	unsigned int idx;
	int ret;

	lockdep_assert_held(&dev->wave_mutex);

	if (!dev->rstc)
		return -ENOENT;

	ret = reset_control_reset(dev->rstc);
	if (ret < 0)
		return ret;

#if 0
	// wave doesn't send response. Force to set BUSY flag to 0.
	wave_write(dev, WAVE_REG_BUSY_FLAG, WAVE_REG_BUSY_STATUS);
	wave_write(dev, CODA_REG_RUN_ENABLE, CODA_REG_BIT_CODE_RUN);
	ret = wave_wait_timeout(dev);
	wave_write(dev, idx, CODA_REG_BIT_RUN_INDEX);
#endif

	return ret;
}
#endif
