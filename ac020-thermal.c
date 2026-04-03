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
 *   - I2C (addr 0x3C): proprietary 18-byte command protocol with CRC16-CCITT
 *   - Commands written to register 0x1D00, status read from 0x0200
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

/* 18-byte command packet written to this register                     */
#define AC020_CMD_REG       0x1D00
/* Read 1 byte from here after every write to check completion         */
#define AC020_STATUS_REG    0x0200

/* Status byte bits (CMD_Status)                                       */
#define AC020_STS_BUSY      BIT(0)  /* 0=idle, 1=busy                 */
#define AC020_STS_FAIL      BIT(1)  /* 0=pass, 1=fail                 */
#define AC020_STS_ERR_MASK  0xFC    /* error code in bits [7:2]        */

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

static int mode = 2;       /* 0=640x512, 1=1280x1024, 2=256x192, 3=384x288 */
static int fps  = 25;

module_param(mode, int, 0644);
module_param(fps,  int, 0644);

MODULE_PARM_DESC(mode, "Resolution: 0=640x512, 1=1280x1024, 2=256x192 (default), 3=384x288");
MODULE_PARM_DESC(fps,  "Frame rate in Hz: 25 (default), 30, 50");

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
	[2] = { .width = 256,  .height = 192  },
	[3] = { .width = 384,  .height = 288  },
};

/* ------------------------------------------------------------------ */
/* Command packet builder                                              */
/*                                                                     */
/* VDCMD packet layout (18 bytes, written to register 0x1D00):        */
/*   [0]     Command Class                                             */
/*   [1]     Module Command Index                                      */
/*   [2]     SubCmd                                                    */
/*   [3]     Reserved (0x00)                                           */
/*   [4-7]   Parameter 1  (Para1[0..3])                               */
/*   [8-11]  Parameter 2  (Para2[0..3])                               */
/*   [12-13] Expected response length (Len, little-endian)            */
/*   [14-15] Reserved (0x00, 0x00)                                    */
/*   [16-17] CRC16-CCITT over bytes [0..15]                           */
/*                                                                     */
/* After every write, poll register 0x0200 (CMD_Status):              */
/*   bit[0] = 0 → IDLE   bit[0] = 1 → BUSY                           */
/*   bit[1] = 0 → PASS   bit[1] = 1 → FAIL                           */
/*   bits[7:2] = error code (0 = ok)                                  */
/* ------------------------------------------------------------------ */

/* Command class / module IDs used in the init sequence               */
#define AC020_CLS_CTRL   0x10   /* general control class              */
#define AC020_MOD_SYS    0x10   /* system module                      */
#define AC020_MOD_YUV    0x03   /* YUV format module                  */

/* SubCmd IDs                                                          */
#define AC020_CMD_OUT_FMT  0x46 /* digital/MIPI output format         */
#define AC020_CMD_DET_FPS  0x44 /* detector frame rate                */
#define AC020_CMD_IMG_SRC  0x45 /* image data source                  */
#define AC020_CMD_YUV_FMT  0x4D /* YUV sub-format (UYVY/YUYV/…)      */

/* Para1[1] values for CMD_OUT_FMT                                    */
#define AC020_OUTPUT_MIPI  0x03 /* MIPI progressive                   */

/* Para1[0] for CMD_IMG_SRC                                           */
#define AC020_SRC_YUV      0x05 /* YUV image output                   */

/* Para1[0] for CMD_YUV_FMT                                           */
#define AC020_YUYV         0x01 /* YUYV byte order                    */

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

/*
 * Build an 18-byte VDCMD packet.
 *   cls/mod/sub : command header bytes
 *   p1[0..3]    : Parameter 1
 *   p2[0..3]    : Parameter 2  (usually zeros)
 *   resp_len    : expected response byte count (Len field)
 * CRC16-CCITT is computed over bytes [0..15] and appended at [16..17].
 */
static void ac020_build_cmd(u8 out[18],
			     u8 cls, u8 mod, u8 sub,
			     u8 p1_0, u8 p1_1, u8 p1_2, u8 p1_3,
			     u8 p2_0, u8 p2_1, u8 p2_2, u8 p2_3,
			     u16 resp_len)
{
	u16 crc;

	out[0]  = cls;
	out[1]  = mod;
	out[2]  = sub;
	out[3]  = 0x00;           /* reserved                            */
	out[4]  = p1_0;
	out[5]  = p1_1;
	out[6]  = p1_2;
	out[7]  = p1_3;
	out[8]  = p2_0;
	out[9]  = p2_1;
	out[10] = p2_2;
	out[11] = p2_3;
	out[12] = resp_len & 0xff;
	out[13] = resp_len >> 8;
	out[14] = 0x00;           /* reserved                            */
	out[15] = 0x00;           /* reserved                            */

	crc     = ac020_crc16(out, 16);
	out[16] = crc & 0xff;
	out[17] = crc >> 8;
}

/*
 * Poll CMD_Status register (0x0200) until the camera is idle.
 * Returns 0 on success, -ETIMEDOUT if the camera stays busy, or
 * -EIO if the command failed.
 */
static int ac020_wait_idle(struct i2c_client *client)
{
	u8 status;
	int tries = 200;   /* 200 × 1 ms = 200 ms max                    */

	do {
		msleep(1);
		if (ac020_i2c_read(client, AC020_STATUS_REG, &status, 1))
			return -EIO;
		if (!(status & AC020_STS_BUSY)) {
			if (status & (AC020_STS_FAIL | AC020_STS_ERR_MASK)) {
				dev_err(&client->dev,
					"camera cmd failed, status=0x%02x\n",
					status);
				return -EIO;
			}
			return 0;
		}
	} while (--tries);

	dev_err(&client->dev, "camera cmd timed out\n");
	return -ETIMEDOUT;
}

/*
 * Build, send, and wait for one 18-byte command.
 */
static int ac020_send_cmd(struct i2c_client *client,
			   u8 cls, u8 mod, u8 sub,
			   u8 p1_0, u8 p1_1, u8 p1_2, u8 p1_3)
{
	u8 pkt[18];
	int ret;

	ac020_build_cmd(pkt, cls, mod, sub,
			p1_0, p1_1, p1_2, p1_3,
			0, 0, 0, 0,   /* Para2 = zeros                   */
			0);            /* no response expected for writes */

	ret = ac020_i2c_write(client, AC020_CMD_REG, pkt, sizeof(pkt));
	if (ret)
		return ret;

	return ac020_wait_idle(client);
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
		u8 fps_byte = (u8)clamp(fps, 1, 60);

		/*
		 * Camera initialisation sequence:
		 *  1. Image source → YUV  (processed image with pseudo-colour)
		 *  2. YUV byte order → YUYV
		 *  3. Detector frame rate
		 *  4. Enable MIPI progressive output at the requested fps
		 */
		ret = ac020_send_cmd(client,
				     AC020_CLS_CTRL, AC020_MOD_SYS,
				     AC020_CMD_IMG_SRC,
				     AC020_SRC_YUV, 0, 0, 0);
		if (ret) {
			dev_err(&client->dev, "set image source failed: %d\n", ret);
			goto out;
		}

		ret = ac020_send_cmd(client,
				     AC020_CLS_CTRL, AC020_MOD_YUV,
				     AC020_CMD_YUV_FMT,
				     AC020_YUYV, 0, 0, 0);
		if (ret) {
			dev_err(&client->dev, "set YUV format failed: %d\n", ret);
			goto out;
		}

		ret = ac020_send_cmd(client,
				     AC020_CLS_CTRL, AC020_MOD_SYS,
				     AC020_CMD_DET_FPS,
				     fps_byte, 0, 0, 0);
		if (ret) {
			dev_err(&client->dev, "set frame rate failed: %d\n", ret);
			goto out;
		}

		/* Para1: [enabled=1] [output type=MIPI] [fps] [0] */
		ret = ac020_send_cmd(client,
				     AC020_CLS_CTRL, AC020_MOD_SYS,
				     AC020_CMD_OUT_FMT,
				     0x01, AC020_OUTPUT_MIPI, fps_byte, 0);
		if (ret) {
			dev_err(&client->dev, "enable MIPI output failed: %d\n", ret);
			goto out;
		}

		ac020->streaming = true;
		dev_dbg(&client->dev, "streaming started @ %d fps\n", fps);

	} else if (!on && ac020->streaming) {
		/* Disable all digital output (Para1[0] = 0x00) */
		ac020_send_cmd(client,
			       AC020_CLS_CTRL, AC020_MOD_SYS,
			       AC020_CMD_OUT_FMT,
			       0x00, 0, 0, 0);
		ac020->streaming = false;
	}

out:

	mutex_unlock(&ac020->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* V4L2 subdev — pad ops                                               */
/* ------------------------------------------------------------------ */

/*
 * ac020_init_try_fmt — lazy initialiser for the per-fh try format.
 * Called from get_fmt / set_fmt when the try format has not been set yet
 * (width == 0).  In kernel 6.8+ init_cfg was removed from v4l2_subdev_pad_ops;
 * v4l2_subdev_state_get_format() replaces v4l2_subdev_get_try_format().
 */
static void ac020_init_try_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(state, 0);

	if (!try_fmt->width)
		ac020_fill_fmt(try_fmt, &ac020_modes[clamp(mode, 0,
					(int)ARRAY_SIZE(ac020_modes) - 1)]);
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
		ac020_init_try_fmt(sd, state);
		fmt->format = *v4l2_subdev_state_get_format(state, 0);
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
		*v4l2_subdev_state_get_format(state, 0) = fmt->format;
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
	/* init_cfg removed in kernel 6.8 — try format initialised lazily */
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
