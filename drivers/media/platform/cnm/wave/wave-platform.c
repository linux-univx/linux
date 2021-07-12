

static struct regmap_config meson_regmap_config = {
	.name = "regmap",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xfff,
	.cache_type = REGCACHE_NONE,
};

static struct regmap_config meson_sram_config = {
	.name = "sram",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x7fff,
	.cache_type = REGCACHE_NONE,
};


/*
 * File operations
 */


static int wave_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct wave_dev *dev = video_get_drvdata(vdev);
	struct wave_ctx *ctx;
	unsigned int max = ~0;
	char *name;
	int ret;
	int idx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	max = WAVE_MAX_INSTANCES - 1;
	idx = ida_alloc_max(&dev->ida, max, GFP_KERNEL);
	if (idx < 0) {
		ret = idx;
		goto err_wave_max;
	}

	name = kasprintf(GFP_KERNEL, "context%d", idx);
	if (!name) {
		ret = -ENOMEM;
		goto err_wave_name_init;
	}

	ctx->debugfs_entry = debugfs_create_dir(name, dev->debugfs_root);
	kfree(name);

	ctx->cvd = to_wave_video_device(vdev);
	ctx->inst_type = ctx->cvd->type;
	ctx->ops = ctx->cvd->ops;
	ctx->use_bit = !ctx->cvd->direct;
	init_completion(&ctx->completion);

	INIT_WORK(&ctx->pic_run_work, wave_pic_run_work);
	if (ctx->ops->seq_init_work)
		INIT_WORK(&ctx->seq_init_work, ctx->ops->seq_init_work);
	if (ctx->ops->seq_end_work)
		INIT_WORK(&ctx->seq_end_work, ctx->ops->seq_end_work);

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->dev = dev;
	ctx->idx = idx;

	wave_dbg(1, ctx, "open instance (%p)\n", ctx);

	ctx->reg_idx = idx;

	/* Power up and upload firmware if necessary */
	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed to power up: %d\n", ret);
		goto err_pm_get;
	}

	ret = clk_bulk_prepare_enable(dev->num_clks, dev->clks);
	if (ret)
		goto err_clk;

	set_default_params(ctx);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx,
					    ctx->ops->queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);

		v4l2_err(&dev->v4l2_dev, "%s return error (%d)\n",
			 __func__, ret);
		goto err_ctx_init;
	}

	ret = wave_ctrls_setup(ctx);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to setup coda controls\n");
		goto err_ctrls_setup;
	}

	ctx->fh.ctrl_handler = &ctx->ctrls;

	mutex_init(&ctx->bitstream_mutex);
	mutex_init(&ctx->buffer_mutex);
	mutex_init(&ctx->wakeup_mutex);
	INIT_LIST_HEAD(&ctx->buffer_meta_list);
	spin_lock_init(&ctx->buffer_meta_lock);

	return 0;

err_ctrls_setup:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_clk:
	clk_disable_unprepare(dev->clk_per);
err_pm_get:
	pm_runtime_put_sync(dev->dev);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
err_wave_name_init:
	ida_free(&dev->ida, ctx->idx);
err_wave_max:
	kfree(ctx);
	return ret;
}

static int wave_release(struct file *file)
{
	struct wave_dev *dev = video_drvdata(file);
	struct wave_ctx *ctx = fh_to_ctx(file->private_data);

	wave_dbg(1, ctx, "release instance (%p)\n", ctx);

	/* If this instance is running, call .job_abort and wait for it to end */
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	/* In case the instance was not running, we still need to call SEQ_END */
	if (ctx->ops->seq_end_work) {
		queue_work(dev->workqueue, &ctx->seq_end_work);
		flush_work(&ctx->seq_end_work);
	}

	wave_free_aux_buf(dev, &ctx->workbuf);

	v4l2_ctrl_handler_free(&ctx->ctrls);
	clk_disable_unprepare(dev->clk_ahb);
	clk_disable_unprepare(dev->clk_per);
	pm_runtime_put_sync(dev->dev);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	ida_free(&dev->ida, ctx->idx);
	if (ctx->ops->release)
		ctx->ops->release(ctx);
	debugfs_remove_recursive(ctx->debugfs_entry);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations wave_fops = {
	.owner		= THIS_MODULE,
	.open		= wave_open,
	.release	= wave_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static int wave_hw_init(struct wave_dev *dev)
{
	u32 data;
	u16 *p;
	int i, ret;

	ret = clk_prepare_enable(dev->clk_per);
	if (ret)
		goto err_clk_per;

	ret = clk_prepare_enable(dev->clk_ahb);
	if (ret)
		goto err_clk_ahb;

	reset_control_reset(dev->rstc);

	/*
	 * Copy the first CODA_ISRAM_SIZE in the internal SRAM.
	 * The 16-bit chars in the code buffer are in memory access
	 * order, re-sort them to CODA order for register download.
	 * Data in this SRAM survives a reboot.
	 */
	p = (u16 *)dev->codebuf.vaddr;
	if (dev->variant->product == CODA_DX6) {
		for (i = 0; i < (CODA_ISRAM_SIZE / 2); i++)  {
			data = CODA_DOWN_ADDRESS_SET(i) |
				CODA_DOWN_DATA_SET(p[i ^ 1]);
			wave_write(dev, data, CODA_REG_BIT_CODE_DOWN);
		}
	} else {
		for (i = 0; i < (CODA_ISRAM_SIZE / 2); i++) {
			data = CODA_DOWN_ADDRESS_SET(i) |
				CODA_DOWN_DATA_SET(p[round_down(i, 4) +
							3 - (i % 4)]);
			wave_write(dev, data, CODA_REG_BIT_CODE_DOWN);
		}
	}

	/* Clear registers */
	for (i = 0; i < 64; i++)
		wave_write(dev, 0, CODA_REG_BIT_CODE_BUF_ADDR + i * 4);

	/* Tell the BIT where to find everything it needs */
	wave_write(dev, dev->workbuf.paddr, CODA_REG_BIT_WORK_BUF_ADDR);

	wave_write(dev, dev->codebuf.paddr,
		      CODA_REG_BIT_CODE_BUF_ADDR);
	wave_write(dev, 0, CODA_REG_BIT_CODE_RUN);

	/* Set default values */
	switch (dev->variant->product) {
	case CODA_DX6:
		wave_write(dev, CODADX6_STREAM_BUF_PIC_FLUSH,
			   CODA_REG_BIT_STREAM_CTRL);
		break;
	default:
		wave_write(dev, CODA7_STREAM_BUF_PIC_FLUSH,
			   CODA_REG_BIT_STREAM_CTRL);
	}

	if (dev->variant->product == CODA_960)
		wave_write(dev, CODA9_FRAME_ENABLE_BWB,
				CODA_REG_BIT_FRAME_MEM_CTRL);
	else
		wave_write(dev, 0, CODA_REG_BIT_FRAME_MEM_CTRL);

	if (dev->variant->product != CODA_DX6)
		wave_write(dev, 0, CODA7_REG_BIT_AXI_SRAM_USE);

	wave_write(dev, CODA_INT_INTERRUPT_ENABLE,
		      CODA_REG_BIT_INT_ENABLE);

	/* Reset VPU and start processor */
	data = wave_read(dev, CODA_REG_BIT_CODE_RESET);
	data |= CODA_REG_RESET_ENABLE;
	wave_write(dev, data, CODA_REG_BIT_CODE_RESET);
	udelay(10);
	data &= ~CODA_REG_RESET_ENABLE;
	wave_write(dev, data, CODA_REG_BIT_CODE_RESET);
	wave_write(dev, CODA_REG_RUN_ENABLE, CODA_REG_BIT_CODE_RUN);

	clk_disable_unprepare(dev->clk_ahb);
	clk_disable_unprepare(dev->clk_per);

	return 0;

err_clk_ahb:
	clk_disable_unprepare(dev->clk_per);
err_clk_per:
	return ret;
}

static int wave_register_device(struct wave_dev *dev, int i)
{
	struct video_device *vfd = &dev->vfd[i];
	const char *name;
	int ret;

	if (i >= dev->variant->num_vdevs)
		return -EINVAL;
	name = dev->variant->vdevs[i]->name;

	strscpy(vfd->name, dev->variant->vdevs[i]->name, sizeof(vfd->name));
	vfd->fops	= &wave_fops;
	vfd->ioctl_ops	= &wave_ioctl_ops;
	vfd->release	= video_device_release_empty;
	vfd->lock	= &dev->dev_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	vfd->vfl_dir	= VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vfd, dev);

	/* Not applicable, use the selection API instead */
	v4l2_disable_ioctl(vfd, VIDIOC_CROPCAP);
	v4l2_disable_ioctl(vfd, VIDIOC_G_CROP);
	v4l2_disable_ioctl(vfd, VIDIOC_S_CROP);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (!ret)
		v4l2_info(&dev->v4l2_dev, "%s registered as %s\n",
			  name, video_device_node_name(vfd));
	return ret;
}

static void wave_copy_firmware(struct wave_dev *dev, const u8 * const buf,
			       size_t size)
{
	u32 *src = (u32 *)buf;
	/* Copy the already reordered firmware image */
	memcpy(dev->codebuf.vaddr, src, size);
}


static void wave_fw_callback(const struct firmware *fw, void *context)
{
	struct wave_dev *dev = context;
	int i, ret;

	/* allocate auxiliary per-device code buffer for the BIT processor */
	ret = wave_alloc_aux_buf(dev, &dev->codebuf, fw->size, "codebuf",
				 dev->debugfs_root);
	if (ret < 0)
		goto put_pm;

	wave_copy_firmware(dev, fw->data, fw->size);
	release_firmware(fw);

	ret = wave_hw_init(dev);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "HW initialization failed\n");
		goto put_pm;
	}

	ret = wave_check_firmware(dev);
	if (ret < 0)
		goto put_pm;

	dev->m2m_dev = v4l2_m2m_init(&wave_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		goto put_pm;
	}

	for (i = 0; i < dev->variant->num_vdevs; i++) {
		ret = wave_register_device(dev, i);
		if (ret) {
			v4l2_err(&dev->v4l2_dev,
				 "Failed to register %s video device: %d\n",
				 dev->variant->vdevs[i]->name, ret);
			goto rel_vfd;
		}
	}

	pm_runtime_put_sync(dev->dev);
	return;

rel_vfd:
	while (--i >= 0)
		video_unregister_device(&dev->vfd[i]);
	v4l2_m2m_release(dev->m2m_dev);
put_pm:
	pm_runtime_put_sync(dev->dev);
}

static int wave_load_firmware(struct wave_dev *dev)
{
	char *fw;

	fw = dev->variant->firmware;

	dev_dbg(dev->dev, "requesting firmware '%s' for %s\n", fw,
		wave_product_name(dev->variant->product));

	return request_firmware_nowait(THIS_MODULE, true, fw, dev->dev,
				       GFP_KERNEL, dev, wave_fw_callback);
}




static int wave_init_rsvd_memory(struct wave_dev *dev)
{
	int ret, irq;
	struct gen_pool *pool;

	/* allocate auxiliary per-device buffers for the BIT processor */
	ret = wave_alloc_aux_buf(dev, &dev->workbuf,
				 dev->variant->workbuf_size, "workbuf",
				 dev->debugfs_root);
	if (ret < 0)
		goto err_v4l2_register;

	if (dev->variant->tempbuf_size) {
		ret = wave_alloc_aux_buf(dev, &dev->tempbuf,
					 dev->variant->tempbuf_size, "tempbuf",
					 dev->debugfs_root);
		if (ret < 0)
			goto err_v4l2_register;
	}

	dev->iram.size = dev->variant->iram_size;
	dev->iram.vaddr = gen_pool_dma_alloc(dev->sram_pool, dev->iram.size,
					     &dev->iram.paddr);
	if (!dev->iram.vaddr) {
		dev_warn(&pdev->dev, "unable to alloc iram\n");
	} else {
		memset(dev->iram.vaddr, 0, dev->iram.size);
		dev->iram.blob.data = dev->iram.vaddr;
		dev->iram.blob.size = dev->iram.size;
		dev->iram.dentry = debugfs_create_blob("iram", 0644,
						       dev->debugfs_root,
						       &dev->iram.blob);
	}


	/* Get IRAM pool from device tree */
	pool = of_gen_pool_get(np, "sram", 0);
	if (!pool) {
		dev_err(&pdev->dev, "iram pool not available\n");
		return -ENOMEM;
	}
	dev->sram_pool = pool;

}











static int wave_probe(struct platform_device *pdev)
{
	struct wave_dev *dev;
	int ret, irq;

	//struct device_node *np = pdev->dev.of_node;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;

	dev->variant = of_device_get_match_data(&pdev->dev);
	dev->num_clks = dev->variant->num_clks;

#if 0
	/* physical addresses limited to 32 bits */
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	dma_set_coherent_mask(&pdev->Dev, DMA_BIT_MASK(32));
#endif

	for (i = 0; i < dev->num_clks; i++)
		dev->clks[i].id = dev->variant->clks[i];

	ret = devm_clk_bulk_get(dev, dev->num_clks, dev->clks);
	if (ret)
		return ret;

	/* Get  memory for physical registers */
	dev->regs_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->regs_base))
		return PTR_ERR(dev->regs_base);

	/* wave IRQ */
	irq = platform_get_irq_byname(pdev, "wave-irq");
	if (irq < 0)
		irq = platform_get_irq(pdev, 0);

	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get wave irq %d.\n", irq);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, wave_irq_handler, 0,
			       WAVE_NAME "-video", dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	dev->rstc = devm_reset_control_get_optional_exclusive(&pdev->dev,
							      NULL);
	if (IS_ERR(dev->rstc)) {
		ret = PTR_ERR(dev->rstc);
		dev_err(&pdev->dev, "failed get reset control: %d\n", ret);
		return ret;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->wave_mutex);
	ida_init(&dev->ida);

	dev->debugfs_root = debugfs_create_dir("wave", NULL);

	dev->workqueue = alloc_workqueue("wave", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		dev_err(&pdev->dev, "unable to alloc workqueue\n");
		ret = -ENOMEM;
		goto err_v4l2_register;
	}

	platform_set_drvdata(pdev, dev);

	/*
	 * Start activated so we can directly call wave_hw_init in
	 * wave_fw_callback regardless of whether CONFIG_PM is
	 * enabled or whether the device is associated with a PM domain.
	 */
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = wave_load_firmware(dev);
	if (ret)
		goto err_alloc_workqueue;
	return 0;

err_alloc_workqueue:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	destroy_workqueue(dev->workqueue);
err_v4l2_register:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static int wave_remove(struct platform_device *pdev)
{
	struct wave_dev *dev = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->vfd); i++) {
		if (video_get_drvdata(&dev->vfd[i]))
			video_unregister_device(&dev->vfd[i]);
	}
	if (dev->m2m_dev)
		v4l2_m2m_release(dev->m2m_dev);
	pm_runtime_disable(&pdev->dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	destroy_workqueue(dev->workqueue);
	if (dev->iram.vaddr)
		gen_pool_free(dev->sram_pool, (unsigned long)dev->iram.vaddr,
			      dev->iram.size);
	wave_free_aux_buf(dev, &dev->codebuf);
	wave_free_aux_buf(dev, &dev->tempbuf);
	wave_free_aux_buf(dev, &dev->workbuf);
	debugfs_remove_recursive(dev->debugfs_root);
	ida_destroy(&dev->ida);
	return 0;
}


static int __maybe_unused wave_runtime_resume(struct device *dev)
{
#if 0
	struct wave_dev *cdev = dev_get_drvdata(dev);
	int ret = 0;

	if (dev->pm_domain && cdev->codebuf.vaddr) {
		ret = wave_hw_init(cdev);
		if (ret)
			v4l2_err(&cdev->v4l2_dev, "HW initialization failed\n");
	}

	return ret;
#else
	return 0;
#endif
}

static int __maybe_unused vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused vpu_resume(struct device *dev)
{
	return 0;
}

static int __maybe_unused vpu_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops wave_pm_ops = {
	SET_RUNTIME_PM_OPS(wave_runtime_suspend, wave_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wave_suspend, wave_resume)
};

/*
 * 	struct clk *clk_per;
	struct clk *clk_apb;
	struct clk *clk_axi;
	struct clk *clk_bpu;
	struct clk *clk_vce;
 **/

static const char * const wave521_clks[] = {
	"dos",
	"pclk",
	"aclk",
	"bclk",
	"cclk",
};

#define WAVE5_MAX_CODE_BUF_SIZE (1024 * 1024)
#define WAVE521ENC_WORKBUF_SIZE (128 * 1024) //HEVC 128K, AVC 40K

static const struct wave_match_data wave521_match_data = {
	.firmware     = "wave521_enc_fw.bin",
	.product      = WAVE_521,
	.codecs       = wave521_codecs,
	.num_codecs   = ARRAY_SIZE(wave521_codecs),
	.vdevs        = wave521_video_devices,
	.num_vdevs    = ARRAY_SIZE(wave521_video_devices),
	.workbuf_size = 128 * 1024, /* enc: h265 128K, h264 40K */
	.tempbuf_size = 204 * 1024,
	.iram_size    = 0x1f000, /* leave 4k for suspend code */
	.clks         = wave521_clks,
	.num_clks     = ARRAY_SIZE(wave521_clks),
};

static const struct of_device_id wave_dt_ids[] = {
	{
		.compatible = "cnm,wave521-vpu",
		.data = &wave521_match_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, wave_dt_ids);

static struct platform_driver wave_driver = {
	.probe	= wave_probe,
	.remove	= wave_remove,
	.driver	= {
		.name	= WAVE_NAME,
		.of_match_table = wave_dt_ids,
		.pm	= &wave_pm_ops,
	},
};

module_platform_driver(wave_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kaspter Ju <camus@rtavs.com>");
MODULE_DESCRIPTION("Chip&Media WAVE multi-standard codec V4L2 driver");
