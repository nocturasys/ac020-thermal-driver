// SPDX-License-Identifier: GPL-2.0
/*
 * AC020 Thermal Camera — MIPI CSI-2 V4L2 Subdev Driver
 * for Raspberry Pi CM5 (Linux 6.6.y)
 *
 * The AC020 is a thermal camera module with an integrated ISP.
 * It outputs already-processed YUV422 (YUYV) frames over MIPI CSI-2.
 * The Pi's own ISP is completely bypassed — frames arrive ready to use.
 *
 * Control interface:
 *   - I2C (addr 0x3C): proprietary command protocol with CRC16-CCITT
 *   - Start/stop streaming by writing a 28-byte command packet
 *   - Pass-through ioctl (CMD_GET / CMD_SET) for direct register access
 *
 * MIPI parameters:
 *   - 2 data lanes
 *   - 80 MHz link frequency
 *   - DataType 0x2A (YUV422 8-bit)
 *   - Non-continuous clock
 *
 * Ported from Rockchip RV1126 driver supplied by the camera manufacturer.
 * Original: Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 * CM5 port:  NocturaSystems
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION  KERNEL_VERSION(1, 0, 0)
#define DRIVER_NAME     "ac020-thermal"

/* ------------------------------------------------------------------ */
/* MIPI / pixel rate                                                   */
/* ------------------------------------------------------------------ */

/* 80 MHz link rate, 2 lanes, DDR → 80*2*2/8 = 40 MPix/s             */
#define AC020_LINK_FREQ     80000000ULL
#define AC020_PIXEL_RATE    40000000UL

/* ------------------------------------------------------------------ */
/* I2C command-protocol constants                                      */
/* ------------------------------------------------------------------ */

/* Write the 28-byte command packet to this 16-bit register address   */
#define AC020_CMD_REG       0x1D00

/* ------------------------------------------------------------------ */
/* Userspace pass-through ioctl                                        */
/*                                                                     */
/* Usage (from userspace via /dev/v4l-subdevX):                       */
/*   struct ac020_ioctl_data req = {                                   */
/*       .wIndex  = <register address>,                                */
/*       .data    = <user buffer>,                                     */
/*       .wLength = <byte count>,                                      */
/*   };                                                                */
/*   ioctl(fd, CMD_GET, &req);   // read registers                    */
/*   ioctl(fd, CMD_SET, &req);   // write registers                   */
/* ------------------------------------------------------------------ */

#define AC020_IOCTL_MAGIC   0xEF

struct ac020_ioctl_data {
	u8          bRequestType;
	u8          bRequest;
	u16         wValue;
	u16         wIndex;      /* 16-bit register address                */
	u8 __user  *data;        /* caller-allocated buffer in user space  */
	u16         wLength;     /* number of bytes to read/write          */
	u32         timeout;     /* unused on Linux, kept for ABI compat   */
};

#define CMD_GET  _IOWR(AC020_IOCTL_MAGIC, 1, struct ac020_ioctl_data)
#define CMD_SET  _IOW (AC020_IOCTL_MAGIC, 2, struct ac020_ioctl_data)

/* ------------------------------------------------------------------ */
/* Module parameters                                                   */
/* ------------------------------------------------------------------ */

static int mode;           /* 0 = 640×512 (default), 1 = 1280×1024   */
static int fps  = 30;
static int img_type = 0x16; /* image-type byte in the start command   */

module_param(mode,     int, 0644);
module_param(fps,      int, 0644);
module_param(img_type, int, 0644);

MODULE_PARM_DESC(mode,     "Resolution: 0=640x512 (default), 1=1280x1024");
MODULE_PARM_DESC(fps,      "Frame rate (default: 30)");
MODULE_PARM_DESC(img_type, "Image-type byte for start command (default: 0x16 = YUV)");

/* ------------------------------------------------------------------ */
/* Supported resolutions                                               */
/* ------------------------------------------------------------------ */

struct ac020_mode {
	u32 width;
	u32 height;
};

static const struct ac020_mode ac020_modes[] = {
	[0] = { .width = 640,  .height = 512  },
	[1] = { .width = 1280, .height = 1024 },
};

/* ------------------------------------------------------------------ */
/* Command packets                                                     */
/*                                                                     */
/* Layout (28 bytes):                                                  */
/*   [0-3]   Command header (0x01 0x30 0xC1/0xC2 0x00)               */
/*   [4-11]  Reserved zeros                                            */
/*   [12-13] Payload length (little-endian) = 0x000A (10 bytes)       */
/*   [14-15] CRC16 of payload  [18-27]  (filled at stream start)      */
/*   [16-17] CRC16 of header   [0-15]   (filled at stream start)      */
/*   [18]    Path   0x00=start, 0x01=stop                             */
/*   [19]    Image type  (overwritten from img_type module param)      */
/*   [20]    Destination                                               */
/*   [21]    FPS                                                       */
/*   [22-23] Width  little-endian  (overwritten at stream start)      */
/*   [24-25] Height little-endian  (overwritten at stream start)      */
/*   [26-27] Reserved                                                  */
/* ------------------------------------------------------------------ */

static u8 start_cmd[28] = {
	0x01, 0x30, 0xc1, 0x00,   /* header                              */
	0x00, 0x00, 0x00, 0x00,   /* reserved                            */
	0x00, 0x00, 0x00, 0x00,   /* reserved                            */
	0x0a, 0x00,               /* payload length = 10                 */
	0x00, 0x00,               /* [14-15] inner CRC placeholder       */
	0x00, 0x00,               /* [16-17] outer CRC placeholder       */
	0x00,                     /* [18] path: start                    */
	0x16,                     /* [19] image type (overwritten)       */
	0x03,                     /* [20] dst                            */
	0x1e,                     /* [21] fps placeholder                */
	0x80, 0x02,               /* [22-23] width  640 LE (overwritten) */
	0x00, 0x02,               /* [24-25] height 512 LE (overwritten) */
	0x00, 0x00,               /* reserved                            */
};

static u8 stop_cmd[28] = {
	0x01, 0x30, 0xc2, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x0a, 0x00,
	0x00, 0x00,               /* [14-15] inner CRC placeholder       */
	0x00, 0x00,               /* [16-17] outer CRC placeholder       */
	0x01,                     /* [18] path: stop                     */
	0x16, 0x00, 0x0e,
	0x80, 0x02, 0x00, 0x02,
	0x00, 0x00,
};

/* ------------------------------------------------------------------ */
/* Driver private state                                                */
/* ------------------------------------------------------------------ */

struct ac020 {
	struct v4l2_subdev        sd;
	struct media_pad          pad;
	struct v4l2_mbus_framefmt fmt;       /* active format             */
	struct mutex              lock;
	struct i2c_client        *client;
	struct v4l2_ctrl_handler  ctrls;
	struct v4l2_ctrl         *link_freq;
	struct v4l2_ctrl         *pixel_rate;
	bool                      streaming;
	int                       mode_idx;  /* index into ac020_modes[]  */
};

static inline struct ac020 *to_ac020(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ac020, sd);
}

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (used by the camera command protocol)                   */
/* ------------------------------------------------------------------ */

static u16 ac020_crc16(const u8 *data, int len)
{
	u16 crc = 0x0000;
	int i;

	while (len--) {
		crc ^= (u16)(*data++) << 8;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

/* ------------------------------------------------------------------ */
/* I2C helpers                                                         */
/* ------------------------------------------------------------------ */

static int ac020_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = addr,
		},
		{
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = len,
			.buf   = buf,
		},
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret == 2) ? 0 : (ret < 0 ? ret : -EIO);
}

static int ac020_i2c_write(struct i2c_client *client,
			    u16 reg, const u8 *buf, int len)
{
	u8 *pkt;
	struct i2c_msg msg;
	int ret;

	pkt = kmalloc(len + 2, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	pkt[0] = reg >> 8;
	pkt[1] = reg & 0xff;
	memcpy(pkt + 2, buf, len);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = len + 2;
	msg.buf   = pkt;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(pkt);
	return (ret == 1) ? 0 : (ret < 0 ? ret : -EIO);
}

/* ------------------------------------------------------------------ */
/* Command packet helpers                                              */
/* ------------------------------------------------------------------ */

static void ac020_prepare_cmd(u8 *cmd, const struct ac020_mode *m)
{
	u16 crc;

	cmd[19] = (u8)img_type;
	cmd[21] = (u8)clamp(fps, 1, 60);
	cmd[22] = m->width  & 0xff;
	cmd[23] = m->width  >> 8;
	cmd[24] = m->height & 0xff;
	cmd[25] = m->height >> 8;

	/* Inner CRC: covers payload bytes [18..27] (10 bytes) */
	crc = ac020_crc16(cmd + 18, 10);
	cmd[14] = crc & 0xff;
	cmd[15] = crc >> 8;

	/* Outer CRC: covers header bytes [0..15] (16 bytes) */
	crc = ac020_crc16(cmd, 16);
	cmd[16] = crc & 0xff;
	cmd[17] = crc >> 8;
}

/* ------------------------------------------------------------------ */
/* Format helpers                                                      */
/* ------------------------------------------------------------------ */

static void ac020_fill_fmt(struct v4l2_mbus_framefmt *fmt,
			    const struct ac020_mode *m)
{
	fmt->width        = m->width;
	fmt->height       = m->height;
	fmt->code         = MEDIA_BUS_FMT_YUYV8_2X8;
	fmt->colorspace   = V4L2_COLORSPACE_SRGB;
	fmt->field        = V4L2_FIELD_NONE;
	fmt->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
}

/* ------------------------------------------------------------------ */
/* V4L2 subdev — video ops                                             */
/* ------------------------------------------------------------------ */

static int ac020_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ac020 *ac020 = to_ac020(sd);
	struct i2c_client *client = ac020->client;
	int ret = 0;

	mutex_lock(&ac020->lock);

	if (on && !ac020->streaming) {
		ac020_prepare_cmd(start_cmd, &ac020_modes[ac020->mode_idx]);
		ret = ac020_i2c_write(client, AC020_CMD_REG,
				      start_cmd, sizeof(start_cmd));
		if (ret)
			dev_err(&client->dev,
				"failed to start streaming: %d\n", ret);
		else
			ac020->streaming = true;

	} else if (!on && ac020->streaming) {
		/*
		 * The manufacturer notes that sending the stop command and
		 * restarting takes a long time (camera re-initialises its ISP).
		 * Uncomment the block below only if you need a hard stop and
		 * are willing to accept the latency on next start.
		 *
		 * ac020_prepare_cmd(stop_cmd, &ac020_modes[ac020->mode_idx]);
		 * ac020_i2c_write(client, AC020_CMD_REG,
		 *                 stop_cmd, sizeof(stop_cmd));
		 */
		ac020->streaming = false;
	}

	mutex_unlock(&ac020->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* V4L2 subdev — pad ops                                               */
/* ------------------------------------------------------------------ */

static int ac020_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_get_try_format(sd, state, 0);

	ac020_fill_fmt(fmt, &ac020_modes[clamp(mode, 0,
				(int)ARRAY_SIZE(ac020_modes) - 1)]);
	return 0;
}

static int ac020_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= 1)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_YUYV8_2X8;
	return 0;
}

static int ac020_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_YUYV8_2X8)
		return -EINVAL;
	if (fse->index >= ARRAY_SIZE(ac020_modes))
		return -EINVAL;

	fse->min_width  = fse->max_width  = ac020_modes[fse->index].width;
	fse->min_height = fse->max_height = ac020_modes[fse->index].height;
	return 0;
}

static int ac020_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= 1)
		return -EINVAL;
	if (fie->code != MEDIA_BUS_FMT_YUYV8_2X8)
		return -EINVAL;

	fie->interval.numerator   = 1;
	fie->interval.denominator = fps;
	return 0;
}

static int ac020_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct ac020 *ac020 = to_ac020(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_try_format(sd, state, 0);
	} else {
		mutex_lock(&ac020->lock);
		fmt->format = ac020->fmt;
		mutex_unlock(&ac020->lock);
	}
	return 0;
}

static int ac020_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct ac020 *ac020 = to_ac020(sd);
	const struct ac020_mode *new_mode;
	unsigned int i, best = 0;
	u32 diff, best_diff = UINT_MAX;

	/* Force the only supported mbus code */
	fmt->format.code = MEDIA_BUS_FMT_YUYV8_2X8;

	/* Find the closest supported resolution */
	for (i = 0; i < ARRAY_SIZE(ac020_modes); i++) {
		diff = abs((int)ac020_modes[i].width  - (int)fmt->format.width) +
		       abs((int)ac020_modes[i].height - (int)fmt->format.height);
		if (diff < best_diff) {
			best_diff = diff;
			best = i;
		}
	}
	new_mode = &ac020_modes[best];
	ac020_fill_fmt(&fmt->format, new_mode);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, state, 0) = fmt->format;
	} else {
		mutex_lock(&ac020->lock);
		ac020->fmt      = fmt->format;
		ac020->mode_idx = best;
		mutex_unlock(&ac020->lock);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* V4L2 subdev — core ops (ioctl pass-through)                        */
/* ------------------------------------------------------------------ */

static long ac020_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ac020_ioctl_data *p = arg;
	u8 *kbuf;
	long ret;

	if (!p || !p->data || !p->wLength)
		return -EINVAL;

	switch (cmd) {
	case CMD_GET:
		kbuf = kmalloc(p->wLength, GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		ret = ac020_i2c_read(client, p->wIndex, kbuf, p->wLength);
		if (!ret && copy_to_user(p->data, kbuf, p->wLength))
			ret = -EFAULT;
		kfree(kbuf);
		break;

	case CMD_SET:
		kbuf = kmalloc(p->wLength, GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		if (copy_from_user(kbuf, p->data, p->wLength)) {
			kfree(kbuf);
			return -EFAULT;
		}
		ret = ac020_i2c_write(client, p->wIndex, kbuf, p->wLength);
		kfree(kbuf);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

/* ------------------------------------------------------------------ */
/* Ops tables                                                          */
/* ------------------------------------------------------------------ */

static const struct v4l2_subdev_core_ops ac020_core_ops = {
	.subscribe_event   = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl             = ac020_ioctl,
};

static const struct v4l2_subdev_video_ops ac020_video_ops = {
	.s_stream = ac020_s_stream,
};

static const struct v4l2_subdev_pad_ops ac020_pad_ops = {
	.init_cfg            = ac020_init_cfg,
	.enum_mbus_code      = ac020_enum_mbus_code,
	.enum_frame_size     = ac020_enum_frame_sizes,
	.enum_frame_interval = ac020_enum_frame_interval,
	.get_fmt             = ac020_get_fmt,
	.set_fmt             = ac020_set_fmt,
};

static const struct v4l2_subdev_ops ac020_subdev_ops = {
	.core  = &ac020_core_ops,
	.video = &ac020_video_ops,
	.pad   = &ac020_pad_ops,
};

/* ------------------------------------------------------------------ */
/* I2C probe / remove                                                  */
/* ------------------------------------------------------------------ */

static const s64 ac020_link_freqs[] = { AC020_LINK_FREQ };

static int ac020_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ac020 *ac020;
	struct v4l2_subdev *sd;
	int ret;

	ac020 = devm_kzalloc(dev, sizeof(*ac020), GFP_KERNEL);
	if (!ac020)
		return -ENOMEM;

	ac020->client   = client;
	ac020->mode_idx = clamp(mode, 0, (int)ARRAY_SIZE(ac020_modes) - 1);
	ac020_fill_fmt(&ac020->fmt, &ac020_modes[ac020->mode_idx]);
	mutex_init(&ac020->lock);

	/* --- Controls ------------------------------------------------- */
	v4l2_ctrl_handler_init(&ac020->ctrls, 2);

	ac020->link_freq = v4l2_ctrl_new_int_menu(
		&ac020->ctrls, NULL,
		V4L2_CID_LINK_FREQ,
		ARRAY_SIZE(ac020_link_freqs) - 1, 0,
		ac020_link_freqs);
	if (ac020->link_freq)
		ac020->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ac020->pixel_rate = v4l2_ctrl_new_std(
		&ac020->ctrls, NULL,
		V4L2_CID_PIXEL_RATE,
		AC020_PIXEL_RATE, AC020_PIXEL_RATE, 1,
		AC020_PIXEL_RATE);

	if (ac020->ctrls.error) {
		ret = ac020->ctrls.error;
		dev_err(dev, "control init failed: %d\n", ret);
		goto err_mutex;
	}

	/* --- Subdev init ---------------------------------------------- */
	sd = &ac020->sd;
	v4l2_i2c_subdev_init(sd, client, &ac020_subdev_ops);
	sd->flags       |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->ctrl_handler = &ac020->ctrls;

	/* Allocates the active subdev state and calls init_cfg */
	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0) {
		dev_err(dev, "subdev init finalize failed: %d\n", ret);
		goto err_ctrl;
	}

	/* --- Media entity --------------------------------------------- */
	ac020->pad.flags    = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ac020->pad);
	if (ret < 0)
		goto err_subdev;

	/* --- Register ------------------------------------------------- */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(dev, "async register failed: %d\n", ret);
		goto err_entity;
	}

	dev_info(dev,
		 "AC020 thermal camera registered — mode %d (%ux%u @ %d fps)\n",
		 ac020->mode_idx,
		 ac020_modes[ac020->mode_idx].width,
		 ac020_modes[ac020->mode_idx].height,
		 fps);
	return 0;

err_entity:
	media_entity_cleanup(&sd->entity);
err_subdev:
	v4l2_subdev_cleanup(sd);
err_ctrl:
	v4l2_ctrl_handler_free(&ac020->ctrls);
err_mutex:
	mutex_destroy(&ac020->lock);
	return ret;
}

static void ac020_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd  = i2c_get_clientdata(client);
	struct ac020 *ac020     = to_ac020(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&ac020->ctrls);
	mutex_destroy(&ac020->lock);
}

/* ------------------------------------------------------------------ */
/* Module boilerplate                                                  */
/* ------------------------------------------------------------------ */

static const struct i2c_device_id ac020_id[] = {
	{ "ac020-thermal", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac020_id);

static const struct of_device_id ac020_of_match[] = {
	{ .compatible = "thermal_cam,ac020-mipi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ac020_of_match);

static struct i2c_driver ac020_i2c_driver = {
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = ac020_of_match,
	},
	.probe    = ac020_probe,
	.remove   = ac020_remove,
	.id_table = ac020_id,
};
module_i2c_driver(ac020_i2c_driver);

MODULE_AUTHOR("NocturaSystems");
MODULE_DESCRIPTION("AC020 Thermal Camera MIPI CSI-2 driver for Raspberry Pi CM5");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
