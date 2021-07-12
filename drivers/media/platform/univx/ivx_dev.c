

/**
 *
 * MMU is not supported so all input and output buffers must be
 * contiguous and be enough to store an output frame.
 *
 * **/

#define IVX_DRIVER_NAME  "arm-univx"


struct ivx_match_data {
	int chip_id;
	const char * const *clks;
	const char * const *rsts;
	int clks_num;
	int rsts_num;
};

struct ivx_dev {

	void __iomem *base_addr;
	int irq;
	struct device *dev;

	unsigned int clk_size;
	struct clk_bulk_data clks[ARMIVX_MAX_BUS_CLK];

	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct media_device media_dev;

	struct v4l2_async_notifier notifier;        //za

	struct vb2_alloc_ctx *alloc_ctx;
}

#if 0

sensor + csi_dphy + csi_host = media_pipeline(软件抽象)

struct media_pipeline {

	struct v4l2_subdev *subdevs[ARMIVX_MAX_PIPELINE];
};


struct csi_channel {

}


#endif





static int ivx_create_links(struct ivx_dev *dev)
{
	struct v4l2_subdev *sd = dev->sensor.sd;
	int ret;

	ret = media_entity_get_fwnode_pad(&sd->entity, sd->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret)
		return ret;

	ret = media_create_pad_link(&sd->entity, 0,
				    &dev->stream.vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(dev->dev, "failed to create link");
		return ret;
	}

	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct ivx_dev *dev;
	int ret;

	dev = container_of(notifier, struct ivx_dev, notifier);

	mutex_lock(&dev->media_dev.graph_mutex);

	ret = ivx_create_links(dev);
	if (ret < 0)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (ret < 0)
		goto unlock;

unlock:
	mutex_unlock(&dev->media_dev.graph_mutex);
	return ret;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct ivx_dev *cif_dev = container_of(notifier,
					struct ivx_dev, notifier);

	int pad;

	cif_dev->sensor.sd = subdev;
	pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;

	cif_dev->sensor.pad = pad;

	return 0;
}

static const struct v4l2_async_notifier_operations subdev_notifier_ops = {
	.bound = subdev_notifier_bound,
    .unbind = subdev_notifier_unbind,
	.complete = subdev_notifier_complete,
};



static int ivx_probe(struct platform_device *pdev)
{
	const struct ivx_match_data *mdata;
	struct ivx_dev *ivx;
	struct device *dev = &pdev->dev;
	int ret, irq;


	mdata = of_device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	ivx = devm_kzalloc(dev, sizeof(*ivx), GFP_KERNEL);
	if (!ivx)
		return -ENOMEM;

	dev_set_drvdata(dev, ivx);
	rkisp1->dev = dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;


    //打开clock & pm

	v4l2_dev = &ivx->v4l2_dev;
	v4l2_dev->mdev = &ivx->media_dev;
	strscpy(v4l2_dev->name, ARMIVX_DRIVER_NAME, sizeof(v4l2_dev->name));

	ret = v4l2_device_register(ivx->dev, &ivx->v4l2_dev);
	if (ret)
		return ret;

    media_device_init(&ivx->media_dev);
	ret = media_device_register(&ivx->media_dev);
	if (ret) {
		dev_err(dev, "Failed to register media device: %d\n", ret);
		goto err_unreg_v4l2_dev;
	}




}
