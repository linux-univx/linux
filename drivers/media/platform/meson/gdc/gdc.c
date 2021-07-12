

struct gdc_fmt {
    u32 fourcc;
    u16 planes;
    u32 types;
};


/*
 * NV12/YV12/RGB444P/YUV444P/YGREY
 */
static struct gdc_fmt formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_NV12,
		.planes = 2;
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.fourcc	= V4L2_PIX_FMT_YVU420,  //YV12
		.planes = 2;
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.fourcc	= V4L2_PIX_FMT_RGB444,
		.planes = 3;
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.fourcc	= V4L2_PIX_FMT_YUV444,
		.planes = 3;
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.fourcc	= V4L2_PIX_FMT_GREY,
		.planes = 1;
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)



struct gdc_frame {
	struct vb2_v4l2_buffer *buf;

	/* Image Format */
	struct v4l2_pix_format pix_fmt;

	/* Crop */
	struct v4l2_rect crop;

	/* Image format */
	const struct gdc_fmt *fmt;
};

// src and dst frame properties
struct gdc_frame_raw {
	const struct gdc_fmt *fmt;

	unsigned long			payload[VIDEO_MAX_PLANES];

	uint32_t input_width;  //gdc input width resolution
	uint32_t input_height; //gdc input height resolution
	uint32_t input_y_stride; //gdc input y stride resolution
	uint32_t input_c_stride; //gdc input uv stride

	uint32_t output_width;  //gdc output width resolution
	uint32_t output_height; //gdc output height resolution
	uint32_t output_y_stride; //gdc output y stride
	uint32_t output_c_stride; //gdc output uv stride
};




struct gdc_ctx {
	struct v4l2_fh		fh;
	struct gdc_dev	*gdc;

	struct gdc_frame	src;
	struct gdc_frame	dst;



    struct v4l2_ctrl_handler ctrl_handler;

};


struct gdc_dev {
	struct v4l2_device	v4l2_dev;
	struct v4l2_m2m_dev	*m2m_dev;

	struct video_device	vfd;
	struct device		*dev;

    struct regmap *map;

	/* Device file mutex */
	struct mutex		mutex;

	void __iomem		*base;

	struct clk		*core_clk;
	struct clk		*axi_clk;

	struct reset_control	*reset;
};




static struct gdc_frame *gdc_get_frame(struct gdc_ctx *ctx, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->src;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->dst;
	default:
		return ERR_PTR(-EINVAL);
	}
}





struct gdc_table {

};


/**
 *  v4l2_ctrl ??
 */
struct gdc_config {
	uint32_t config_addr;   //gdc config address
	uint32_t config_size;   //gdc config size in 32bit

};


/**
 * struct gdc_addr -  GDC physical address set
 * @y:	 luminance plane address
 * @cb:	 Cb plane address
 * @cr:	 Cr plane address
 */
struct gdc_addr {
	dma_addr_t y;
	dma_addr_t cb;
	dma_addr_t cr;
};




static inline u32 gdc_read(struct gdc_dev *dev, u32 reg)
{
	return readl(dev->base + reg);
}

static inline void gdc_write(struct gdc_dev *dev,
				     u32 reg, u32 value)
{
	writel(value, dev->base + reg);
}

static inline void gdc_set_bits(struct gdc_dev *dev,
					u32 reg, u32 bits)
{
	writel(readl(dev->base + reg) | bits, dev->base + reg);
}

static inline void gdc_clr_set_bits(struct gdc_dev *dev,
					    u32 reg, u32 clr, u32 set)
{
	u32 val = readl(dev->base + reg);

	val &= ~clr;
	val |= set;

	writel(val, dev->base + reg);
}

static int gdc_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				   unsigned int *nplanes, unsigned int sizes[], struct device *alloc_devs[])
{
	struct gdc_ctx *ctx = vb2_get_drv_priv(vq);
    int i;

	struct gdc_frame *frame = gdc_get_frame(ctx, vq->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

    *nplanes = frame->fmt->planes;

    // TODO: double check
	for (i = 0; i < frame->fmt->planes; i++)
		sizes[i] = frame->payload[i];

	return 0;
}


static int gdc_buf_prepare(struct vb2_buffer *vb)
{
	struct gdc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct gdc_frame *frame = gdc_get_frame(ctx, vb->vb2_queue->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		for (i = 0; i < frame->fmt->planes; i++)
			vb2_set_plane_payload(vb, i, frame->payload[i]);
	}

	return 0;
}

static void gdc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct gdc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	pr_debug("ctx: %p, ctx->state: 0x%x", ctx, ctx->state);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}


static const struct vb2_ops gdc_qops = {
	.queue_setup	= gdc_queue_setup,  // called from VIDIOC_REQBUFS
	.buf_prepare	= gdc_buf_prepare,
	.buf_queue	    = gdc_buf_queue,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
};


static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct gdc_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &gdc_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->gdc->mutex;
	src_vq->dev = ctx->gdc->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &gdc_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->gdc->mutex;
	dst_vq->dev = ctx->gdc->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int gdc_enable_clocks(struct gdc_dev *gdc)
{
	int ret;

	ret = clk_prepare_enable(gdc->core_clk);
	if (ret) {
		dev_err(gdc->dev, "Cannot enable gdc sclk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(gdc->axi_clk);
	if (ret) {
		dev_err(gdc->dev, "Cannot enable gdc aclk: %d\n", ret);
		goto err_disable_core_clk;
	}


	return 0;

err_disable_core_clk:
	clk_disable_unprepare(gdc->core_clk);

	return ret;
}

static void gdc_disable_clocks(struct gdc_dev *gdc)
{
	clk_disable_unprepare(gdc->core_clk);
	clk_disable_unprepare(gdc->axi_clk);
}

static int gdc_parse_dt(struct gdc_dev *gdc)
{
	gdc->reset = devm_reset_control_get(gdc->dev, NULL);
	if (IS_ERR(gdc->reset)) {
		dev_err(gdc->dev, "Failed to get reset control\n");
		return PTR_ERR(gdc->rstc);
	}

	gdc->core_clk = devm_clk_get(gdc->dev, "core");
	if (IS_ERR(gdc->core_clk)) {
		dev_err(gdc->dev, "Failed to get core clock\n");
		return PTR_ERR(gdc->core_clk);
	}

	gdc->axi_clk = devm_clk_get(gdc->dev, "axi");
	if (IS_ERR(gdc->mod_clk)) {
		dev_err(gdc->dev, "Failed to get axi clock\n");

		return PTR_ERR(gdc->axi_clk);
	}

    return 0;
}





static const struct v4l2_file_operations gdc_fops = {
	.owner		= THIS_MODULE,
	.open		= gdc_open,
	.release	= gdc_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct video_device gdc_video_device = {
	.name		= GDC_NAME,
	.fops		= &gdc_fops,
	.ioctl_ops	= &gdc_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
	.vfl_dir	= VFL_DIR_M2M,
	.device_caps	= V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

static const struct v4l2_m2m_ops gdc_m2m_ops = {
	.device_run	= gdc_device_run,
	.job_ready	= gdc_job_ready,
	.job_abort	= gdc_job_abort,
};


static const struct regmap_config gdc_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = GDC_XXX_FIXME,
};


static int gdc_probe(struct platform_device *pdev)
{
	struct gdc_dev *gdc;
	struct video_device *vfd;
	struct resource *res;
	void __iomem *regbase;
	int irq, ret;

	gdc = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!gdc)
		return -ENOMEM;

	gdc->dev = &pdev->dev;
	gdc->vfd = gdc_video_device;

	mutex_init(&gdc->mutex);

	ret = gdc_parse_dt(gdc);
	if (ret) {
		dev_err(&pdev->dev, "Unable to parse OF data\n");
		return ret;
    }

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regbase))
		return PTR_ERR(regbase);

    // setup regmap
    gdc->map = devm_regmap_init_mmio(gdc->dev, regbase, &gdc_regmap_conf);
	if (IS_ERR(gdc->map))
		return PTR_ERR(gdc->map);


	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return irq;

	ret = devm_request_irq(gdc->dev, irq, gdc_irq,
			       0, dev_name(gdc->dev), dev);
	if (ret) {
		dev_err(gdc->dev, "Failed to request IRQ\n");
		return ret;
	}

	ret = v4l2_device_register(&pdev->dev, &gdc->v4l2_dev);
	if (ret) {
		dev_err(gdc->dev, "Failed to register V4L2 device\n");
		return ret;
	}

	vfd = &gdc->vfd;
	vfd->lock = &gdc->mutex;
	vfd->v4l2_dev = &gdc->v4l2_dev;

	snprintf(vfd->name, sizeof(vfd->name), "%s", gdc_video_device.name);
	video_set_drvdata(vfd, gdc);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		v4l2_err(&gdc->v4l2_dev, "Failed to register video device\n");
		goto err_v4l2;
	}

	v4l2_info(&gdc->v4l2_dev, "Device registered as /dev/video%d\n", vfd->num);

	platform_set_drvdata(pdev, gdc);

	gdc->m2m_dev = v4l2_m2m_init(&gdc_m2m_ops);
	if (IS_ERR(gdc->m2m_dev)) {
		v4l2_err(&gdc->v4l2_dev, "Failed to initialize V4L2 M2M device\n");
		ret = PTR_ERR(gdc->m2m_dev);
		goto err_video;
	}

	pm_runtime_enable(gdc->dev);

	return 0;

err_video:
	video_unregister_device(&gdc->vfd);
err_v4l2:
	v4l2_device_unregister(&gdc->v4l2_dev);

	return ret;
}

static int gdc_remove(struct platform_device *pdev)
{
	struct gdc_dev *gdc = platform_get_drvdata(pdev);

	v4l2_m2m_release(gdc->m2m_dev);
	video_unregister_device(&gdc->vfd);
	v4l2_device_unregister(&gdc->v4l2_dev);

	pm_runtime_force_suspend(&gdc->dev);

	return 0;
}












static int __maybe_unused gdc_runtime_suspend(struct device *dev)
{
	struct gdc_dev *gdc = dev_get_drvdata(dev);

	gdc_disable_clocks(gdc);

	return 0;
}

static int __maybe_unused gdc_runtime_resume(struct device *dev)
{
	struct gdc_dev *gdc = dev_get_drvdata(dev);

	return gdc_enable_clocks(gdc);
}

static const struct dev_pm_ops gdc_pm_ops = {
	.runtime_resume		= gdc_runtime_resume,
	.runtime_suspend	= gdc_runtime_suspend,
};

static const struct of_device_id gdc_dt_match[] = {
	{ .compatible = "amlogic,g12b-gdc" }, // g12b-gdc
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gdc_dt_match);

static struct platform_driver gdc_driver = {
	.probe		= gdc_probe,
	.remove		= gdc_remove,
	.driver		= {
		.name		= GDC_NAME,
		.pm		= &gdc_pm_ops,
		.of_match_table	= gdc_dt_match,
	},
};
module_platform_driver(gdc_driver);
