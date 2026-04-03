# AC020 Thermal Camera — Linux Driver for Raspberry Pi CM5

V4L2 MIPI CSI-2 subdev driver for the AC020 thermal camera module.

## How it works

The AC020 contains its own **integrated ISP** — it outputs fully processed
**YUV422 (YUYV) frames** directly over MIPI CSI-2. The Raspberry Pi's own
ISP is completely bypassed.

```
Thermal sensor → AC020 ISP → MIPI CSI-2 (YUV422) → RP1 CFE → /dev/videoX
```

Camera control happens over **I2C** at address `0x3C`:
- A proprietary 28-byte command packet (with CRC16-CCITT) starts/stops
  streaming
- Direct register read/write is available via a pass-through ioctl
  (`CMD_GET` / `CMD_SET`) on the `/dev/v4l-subdevX` node

## MIPI parameters

| Parameter       | Value              |
|-----------------|--------------------|
| Data lanes      | 2                  |
| Link frequency  | 80 MHz             |
| MIPI DataType   | 0x2A (YUV422 8-bit)|
| Clock mode      | Non-continuous     |

## Supported resolutions

| Mode | Width | Height | FPS |
|------|-------|--------|-----|
| 0    | 640   | 512    | 30  |
| 1    | 1280  | 1024   | 30  |

---

## Prerequisites

```bash
# On the Pi CM5 (Raspberry Pi OS Bookworm 64-bit)
sudo apt update
sudo apt install -y linux-headers-$(uname -r) build-essential device-tree-compiler
# If the above fails (headers already bundled), just install the build tools:
sudo apt install -y build-essential device-tree-compiler
```

---

## Build the kernel module

```bash
git clone https://github.com/nocturasys/ac020-thermal-driver.git
cd ac020-thermal-driver

# Build
make

# Install (copies .ko and runs depmod)
sudo make install
```

---

## Build and install the DTS overlay

```bash
# Compile
make dtbo

# Install to /boot/overlays/
sudo make install_dtbo
```

---

## Enable in /boot/config.txt

```ini
# Default: CAM1 connector, 640×512 @ 30 fps
dtoverlay=ac020-thermal-cam1

# Use CAM0 connector instead
dtoverlay=ac020-thermal-cam1,cam0
```

Reboot after editing config.txt.

---

## Verify the driver loaded

```bash
# Check the driver was found
dmesg | grep ac020

# Inspect the media pipeline
media-ctl -p -d /dev/media0

# List available formats
v4l2-ctl --list-formats-ext -d /dev/video0
```

Expected `dmesg` output:
```
ac020-thermal 1-003c: AC020 thermal camera registered — mode 0 (640x512 @ 30 fps)
```

---

## Capture a frame

### With v4l2-ctl
```bash
# Capture one raw YUYV frame
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=640,height=512,pixelformat=YUYV \
  --stream-mmap --stream-count=1 --stream-to=frame.raw

# Convert to PNG with ffmpeg
ffmpeg -f rawvideo -pix_fmt yuyv422 -s 640x512 -i frame.raw frame.png
```

### With ffmpeg live stream
```bash
ffplay -f v4l2 -input_format yuyv422 -video_size 640x512 /dev/video0
```

### With GStreamer
```bash
gst-launch-1.0 v4l2src device=/dev/video0 \
  ! video/x-raw,format=YUY2,width=640,height=512,framerate=30/1 \
  ! videoconvert ! autovideosink
```

---

## Module parameters

Parameters can be set at load time or changed via `/sys/module/ac020_thermal/parameters/`:

| Parameter  | Default | Description                              |
|------------|---------|------------------------------------------|
| `mode`     | `0`     | Resolution: 0=640×512, 1=1280×1024       |
| `fps`      | `30`    | Target frame rate                        |
| `img_type` | `0x16`  | Image-type byte in the start command     |

```bash
# Load with 1280×1024
sudo modprobe ac020_thermal mode=1

# Change fps at runtime
echo 25 | sudo tee /sys/module/ac020_thermal/parameters/fps
```

---

## Direct register access (ioctl)

The driver exposes `CMD_GET` / `CMD_SET` ioctls on `/dev/v4l-subdevX`
for sending raw I2C commands to the camera — useful for configuring
features not exposed through V4L2 controls.

```c
#include <sys/ioctl.h>
#include <fcntl.h>

#define AC020_IOCTL_MAGIC 0xEF

struct ac020_ioctl_data {
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;     /* register address */
    uint8_t *data;       /* user buffer      */
    uint16_t wLength;
    uint32_t timeout;
};

#define CMD_GET _IOWR(AC020_IOCTL_MAGIC, 1, struct ac020_ioctl_data)
#define CMD_SET _IOW (AC020_IOCTL_MAGIC, 2, struct ac020_ioctl_data)

int fd = open("/dev/v4l-subdev1", O_RDWR);

/* Example: read 4 bytes from register 0x0200 */
uint8_t buf[4];
struct ac020_ioctl_data req = {
    .wIndex  = 0x0200,
    .data    = buf,
    .wLength = 4,
};
ioctl(fd, CMD_GET, &req);
```

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| `dmesg` shows no AC020 message | I2C address / bus wrong in DTS? `i2cdetect -y 1` |
| `/dev/video0` missing | `media-ctl -p` — is the pipeline complete? |
| Blank / garbled image | Wrong `img_type`? Try `img_type=0x00` |
| `VIDIOC_STREAMON` fails | Check `dmesg` for I2C transfer errors |
| 1280×1024 doesn't work | Try `mode=1` module param AND set v4l2 format to 1280×1024 |

---

## Architecture notes

- **No Pi ISP involved**: libcamera's IPA (image processing algorithms)
  will not be invoked. Use plain V4L2 or libcamera in "software ISP" mode.
- **Stop command disabled by default**: The camera re-initialises its ISP
  on every start/stop, which takes several seconds. Streaming is simply
  left running until the driver is unloaded.
- **CRC16-CCITT**: Both the payload and the header of each command packet
  are individually CRC-protected by the camera firmware.
