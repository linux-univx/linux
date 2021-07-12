

/**
 * vb2 operations
 */

static int ivx_vb2_queue_setup(struct vb2_queue *queue,
				  unsigned int *num_buffers,
				  unsigned int *num_planes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct ivx_capture *cap = queue->drv_priv;
	const struct v4l2_pix_format_mplane *pixm = &cap->pix.fmt;
	unsigned int i;

	if ( (queue->num_buffers + *num_buffers) < 3 ) {
		*num_buffers = 3 - queue->num_buffers;
	}

	if (*num_planes) {
		if (*num_planes != pixm->num_planes)
			return -EINVAL;

		for (i = 0; i < pixm->num_planes; i++)
			if (sizes[i] < pixm->plane_fmt[i].sizeimage)
				return -EINVAL;
	} else {
		*num_planes = pixm->num_planes;
		for (i = 0; i < pixm->num_planes; i++)
			sizes[i] = pixm->plane_fmt[i].sizeimage;
	}

	return 0;
}

static int ivx_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ivx_capture *cap = vb2_get_drv_priv(vb->vb2_queue);
	u32 size = cap->f_current.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(video->dev,
			"Error user buffer too small (%ld < %u)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static void ivx_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ivx_capture *cap = vb2_get_drv_priv(vb->vb2_queue);


}


static const struct vb2_ops ivx_vb2_ops = {
    .queue_setup = ivx_vb2_queue_setup, // called from VIDIOC_REQBUFS
    .buf_prepare = ivx_vb2_buf_prepare,
    .buf_queue = ivx_vb2_buf_queue,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};
