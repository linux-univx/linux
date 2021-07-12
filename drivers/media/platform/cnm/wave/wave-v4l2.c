


static void wave_device_run(void *priv)
{
#if 0
	struct vpu_instance *inst = priv;

	DPRINTK(inst->dev, 1, "inst type=%d state=%d\n",
		inst->type, inst->state);

	inst->ops->start_process(inst);
#endif
}

static int wave_job_ready(void *priv)
{
#if 0
	struct vpu_instance *inst = priv;

	DPRINTK(inst->dev, 1, "inst type=%d state=%d\n",
		inst->type, inst->state);

	if (inst->state == VPU_INST_STATE_STOP)
		return 0;
#endif
	return 1;
}

static void wave_job_abort(void *priv)
{
#if 0
	struct vpu_instance *inst = priv;

	DPRINTK(inst->dev, 1, "inst type=%d state=%d\n",
		inst->type, inst->state);

	inst->ops->stop_process(inst);
#endif
}

static const struct v4l2_m2m_ops wave_m2m_ops = {
	.device_run = wave_device_run,
	.job_ready  = wave_job_ready,
	.job_abort  = wave_job_abort,
};
