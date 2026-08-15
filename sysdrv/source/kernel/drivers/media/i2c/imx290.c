// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX462 / IMX290 Dedicated 1080p60 2-Lane MIPI Driver
 * Tailored for Rockchip RV1106 / Linux 5.10
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/rk-camera-module.h>
#include <asm/unaligned.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX290_REG_SIZE_SHIFT				16
#define IMX290_REG_ADDR_MASK				0xffff
#define IMX290_REG_8BIT(n)				((1U << IMX290_REG_SIZE_SHIFT) | (n))
#define IMX290_REG_16BIT(n)				((2U << IMX290_REG_SIZE_SHIFT) | (n))
#define IMX290_REG_24BIT(n)				((3U << IMX290_REG_SIZE_SHIFT) | (n))

#define IMX290_STANDBY					IMX290_REG_8BIT(0x3000)
#define IMX290_XMSTA					IMX290_REG_8BIT(0x3002)
#define IMX290_ADBIT					IMX290_REG_8BIT(0x3005)
#define IMX290_CTRL_07					IMX290_REG_8BIT(0x3007)
#define IMX290_FR_FDG_SEL				IMX290_REG_8BIT(0x3009)
#define IMX290_BLKLEVEL					IMX290_REG_16BIT(0x300a)
#define IMX290_GAIN						IMX290_REG_8BIT(0x3014)
#define IMX290_VMAX						IMX290_REG_24BIT(0x3018)
#define IMX290_HMAX						IMX290_REG_16BIT(0x301c)
#define IMX290_SHS1						IMX290_REG_24BIT(0x3020)
#define IMX290_WINWV_OB					IMX290_REG_8BIT(0x303a)
#define IMX290_WINPV					IMX290_REG_16BIT(0x303c)
#define IMX290_WINWV					IMX290_REG_16BIT(0x303e)
#define IMX290_WINPH					IMX290_REG_16BIT(0x3040)
#define IMX290_WINWH					IMX290_REG_16BIT(0x3042)
#define IMX290_OUT_CTRL					IMX290_REG_8BIT(0x3046)
#define IMX290_XSOUTSEL					IMX290_REG_8BIT(0x304b)
#define IMX290_INCKSEL1					IMX290_REG_8BIT(0x305c)
#define IMX290_INCKSEL2					IMX290_REG_8BIT(0x305d)
#define IMX290_INCKSEL3					IMX290_REG_8BIT(0x305e)
#define IMX290_INCKSEL4					IMX290_REG_8BIT(0x305f)
#define IMX290_PGCTRL					IMX290_REG_8BIT(0x308c)
#define IMX290_ADBIT1					IMX290_REG_8BIT(0x3129)
#define IMX290_INCKSEL5					IMX290_REG_8BIT(0x315e)
#define IMX290_INCKSEL6					IMX290_REG_8BIT(0x3164)
#define IMX290_ADBIT2					IMX290_REG_8BIT(0x317c)
#define IMX290_ADBIT3					IMX290_REG_8BIT(0x31ec)
#define IMX290_REPETITION				IMX290_REG_8BIT(0x3405)
#define IMX290_PHY_LANE_NUM				IMX290_REG_8BIT(0x3407)
#define IMX290_OPB_SIZE_V				IMX290_REG_8BIT(0x3414)
#define IMX290_Y_OUT_SIZE				IMX290_REG_16BIT(0x3418)
#define IMX290_CSI_DT_FMT				IMX290_REG_16BIT(0x3441)
#define IMX290_CSI_LANE_MODE			IMX290_REG_8BIT(0x3443)
#define IMX290_EXTCK_FREQ				IMX290_REG_16BIT(0x3444)
#define IMX290_TCLKPOST					IMX290_REG_16BIT(0x3446)
#define IMX290_THSZERO					IMX290_REG_16BIT(0x3448)
#define IMX290_THSPREPARE				IMX290_REG_16BIT(0x344a)
#define IMX290_TCLKTRAIL				IMX290_REG_16BIT(0x344c)
#define IMX290_THSTRAIL					IMX290_REG_16BIT(0x344e)
#define IMX290_TCLKZERO					IMX290_REG_16BIT(0x3450)
#define IMX290_TCLKPREPARE				IMX290_REG_16BIT(0x3452)
#define IMX290_TLPX						IMX290_REG_16BIT(0x3454)
#define IMX290_X_OUT_SIZE				IMX290_REG_16BIT(0x3472)
#define IMX290_INCKSEL7					IMX290_REG_8BIT(0x3480)

#define IMX290_PIXEL_RATE				148500000

struct imx290_regval {
	u32 reg;
	u32 val;
};

/* Verified 1080p60 Base Settings */
static const struct imx290_regval imx290_global_init[] = {
	{ IMX290_WINWV_OB, 12 },
	{ IMX290_WINPH, 0 },
	{ IMX290_WINPV, 0 },
	{ IMX290_WINWH, 1948 },
	{ IMX290_WINWV, 1097 },
	{ IMX290_XSOUTSEL, 0x0a },
	{ IMX290_REG_8BIT(0x3011), 0x00 },
	{ IMX290_REG_8BIT(0x3012), 0x64 },
	{ IMX290_REG_8BIT(0x3013), 0x00 },
	{ IMX290_REG_8BIT(0x300f), 0x00 },
	{ IMX290_REG_8BIT(0x3010), 0x21 },
	{ IMX290_REG_8BIT(0x3016), 0x09 },
	{ IMX290_REG_8BIT(0x3070), 0x02 },
	{ IMX290_REG_8BIT(0x3071), 0x11 },
	{ IMX290_REG_8BIT(0x309b), 0x10 },
	{ IMX290_REG_8BIT(0x309c), 0x22 },
	{ IMX290_REG_8BIT(0x30a2), 0x02 },
	{ IMX290_REG_8BIT(0x30a6), 0x20 },
	{ IMX290_REG_8BIT(0x30a8), 0x20 },
	{ IMX290_REG_8BIT(0x30aa), 0x20 },
	{ IMX290_REG_8BIT(0x30ac), 0x20 },
	{ IMX290_REG_8BIT(0x30b0), 0x43 },
	{ IMX290_REG_8BIT(0x3119), 0x9e },
	{ IMX290_REG_8BIT(0x311c), 0x1e },
	{ IMX290_REG_8BIT(0x311e), 0x08 },
	{ IMX290_REG_8BIT(0x3128), 0x05 },
	{ IMX290_REG_8BIT(0x313d), 0x83 },
	{ IMX290_REG_8BIT(0x3150), 0x03 },
	{ IMX290_REG_8BIT(0x317e), 0x00 },
	{ IMX290_REG_8BIT(0x32b8), 0x50 },
	{ IMX290_REG_8BIT(0x32b9), 0x10 },
	{ IMX290_REG_8BIT(0x32ba), 0x00 },
	{ IMX290_REG_8BIT(0x32bb), 0x04 },
	{ IMX290_REG_8BIT(0x32c8), 0x50 },
	{ IMX290_REG_8BIT(0x32c9), 0x10 },
	{ IMX290_REG_8BIT(0x32ca), 0x00 },
	{ IMX290_REG_8BIT(0x32cb), 0x04 },
	{ IMX290_REG_8BIT(0x332c), 0xd3 },
	{ IMX290_REG_8BIT(0x332d), 0x10 },
	{ IMX290_REG_8BIT(0x332e), 0x0d },
	{ IMX290_REG_8BIT(0x3358), 0x06 },
	{ IMX290_REG_8BIT(0x3359), 0xe1 },
	{ IMX290_REG_8BIT(0x335a), 0x11 },
	{ IMX290_REG_8BIT(0x3360), 0x1e },
	{ IMX290_REG_8BIT(0x3361), 0x61 },
	{ IMX290_REG_8BIT(0x3362), 0x10 },
	{ IMX290_REG_8BIT(0x33b0), 0x50 },
	{ IMX290_REG_8BIT(0x33b2), 0x1a },
	{ IMX290_REG_8BIT(0x33b3), 0x04 },
	{ IMX290_OPB_SIZE_V, 10 },
	{ IMX290_X_OUT_SIZE, 1920 },
	{ IMX290_Y_OUT_SIZE, 1080 },
	{ IMX290_ADBIT, 0x01 },        /* 12-bit ADC */
	{ IMX290_OUT_CTRL, 0x01 },     /* 12-bit MIPI Output */
	{ IMX290_ADBIT1, 0x00 },
	{ IMX290_ADBIT2, 0x00 },
	{ IMX290_ADBIT3, 0x0e },
	{ IMX290_CSI_DT_FMT, 0x0c0c }, /* RAW12 */
	{ IMX290_BLKLEVEL, 0x00f0 },   /* 240 default black level */
};

struct imx290 {
	struct device *dev;
	struct clk *xclk;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *rst_gpio;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	u32 module_index;
};

static inline struct imx290 *to_imx290(struct v4l2_subdev *_sd) {
	return container_of(_sd, struct imx290, sd);
}

static const s64 link_freq_2lanes[] = { 445500000 };

static int imx290_write(struct imx290 *imx290, u32 addr, u32 value, int *err)
{
	u8 data[3];
	int ret;

	if (err && *err)
		return *err;

	put_unaligned_le24(value, data);
	ret = regmap_raw_write(imx290->regmap, addr & IMX290_REG_ADDR_MASK,
			       data, (addr >> IMX290_REG_SIZE_SHIFT) & 3);
	if (ret < 0 && err)
		*err = ret;
	return ret;
}

static int imx290_set_register_array(struct imx290 *imx290,
				     const struct imx290_regval *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = imx290_write(imx290, settings->reg, settings->val, NULL);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int imx290_start_streaming(struct imx290 *imx290)
{
	int ret = 0;

	/* Standby on before register configuration */
	imx290_write(imx290, IMX290_STANDBY, 0x01, &ret);
	imx290_write(imx290, IMX290_XMSTA, 0x01, &ret);
	usleep_range(10000, 12000);

	ret = imx290_set_register_array(imx290, imx290_global_init, ARRAY_SIZE(imx290_global_init));
	if (ret) return ret;

	/* Clocks for 37.125 MHz */
	imx290_write(imx290, IMX290_EXTCK_FREQ, 0x2520, &ret);
	imx290_write(imx290, IMX290_INCKSEL7, 0x49, &ret);
	imx290_write(imx290, IMX290_INCKSEL1, 0x18, &ret);
	imx290_write(imx290, IMX290_INCKSEL2, 0x03, &ret);
	imx290_write(imx290, IMX290_INCKSEL3, 0x20, &ret);
	imx290_write(imx290, IMX290_INCKSEL4, 0x01, &ret);
	imx290_write(imx290, IMX290_INCKSEL5, 0x1a, &ret);
	imx290_write(imx290, IMX290_INCKSEL6, 0x1a, &ret);

	/* 2-Lane MIPI Data */
	imx290_write(imx290, IMX290_PHY_LANE_NUM, 1, &ret);
	imx290_write(imx290, IMX290_CSI_LANE_MODE, 1, &ret);
	imx290_write(imx290, IMX290_FR_FDG_SEL, 0x01, &ret);
	imx290_write(imx290, IMX290_CTRL_07, 0x00, &ret);

	/* D-PHY 445.5MHz Timings */
	imx290_write(imx290, IMX290_REPETITION, 0x00, &ret);
	imx290_write(imx290, IMX290_TCLKPOST, 119, &ret);
	imx290_write(imx290, IMX290_THSZERO, 103, &ret);
	imx290_write(imx290, IMX290_THSPREPARE, 71, &ret);
	imx290_write(imx290, IMX290_TCLKTRAIL, 55, &ret);
	imx290_write(imx290, IMX290_THSTRAIL, 63, &ret);
	imx290_write(imx290, IMX290_TCLKZERO, 255, &ret);
	imx290_write(imx290, IMX290_TCLKPREPARE, 63, &ret);
	imx290_write(imx290, IMX290_TLPX, 55, &ret);
	imx290_write(imx290, IMX290_PGCTRL, 0x00, &ret);

	__v4l2_ctrl_handler_setup(imx290->sd.ctrl_handler);

	/* Standby cancel and settle */
	imx290_write(imx290, IMX290_STANDBY, 0x00, &ret);
	msleep(35);

	return imx290_write(imx290, IMX290_XMSTA, 0x00, &ret);
}

static int imx290_stop_streaming(struct imx290 *imx290)
{
	int ret = 0;
	imx290_write(imx290, IMX290_STANDBY, 0x01, &ret);
	msleep(20);
	return imx290_write(imx290, IMX290_XMSTA, 0x01, &ret);
}

static int imx290_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx290 *imx290 = container_of(ctrl->handler, struct imx290, ctrls);
	int ret = 0, vmax;

	if (!pm_runtime_get_if_in_use(imx290->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx290_write(imx290, IMX290_GAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = imx290_write(imx290, IMX290_VMAX, ctrl->val + 1080, NULL);
		ctrl = imx290->exposure;
		fallthrough;
	case V4L2_CID_EXPOSURE:
		vmax = imx290->vblank->val + 1080;
		if (ctrl->val >= vmax) ctrl->val = vmax - 2;
		ret = imx290_write(imx290, IMX290_SHS1, vmax - ctrl->val - 1, NULL);
		break;
	case V4L2_CID_HBLANK:
		ret = imx290_write(imx290, IMX290_HMAX, ctrl->val + 1920, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(imx290->dev);
	pm_runtime_put_autosuspend(imx290->dev);
	return ret;
}

static const struct v4l2_ctrl_ops imx290_ctrl_ops = { .s_ctrl = imx290_set_ctrl };

static int imx290_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx290 *imx290 = to_imx290(sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(imx290->dev);
		if (ret < 0) return ret;
		ret = imx290_start_streaming(imx290);
	} else {
		imx290_stop_streaming(imx290);
		pm_runtime_mark_last_busy(imx290->dev);
		pm_runtime_put_autosuspend(imx290->dev);
	}
	return ret;
}

static int imx290_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	fmt->format.width = 1920;
	fmt->format.height = 1080;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

static int imx290_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = 1920;
	sel->r.height = 1080;
	return 0;
}

static int imx290_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad, struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = V4L2_MBUS_CSI2_2_LANE;
	return 0;
}

static int imx290_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 10000;
	fi->interval.denominator = 600000; /* 60 FPS */
	return 0;
}

static void imx290_get_module_inf(struct imx290 *imx290, struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, "imx462", sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx290->module_name ? imx290->module_name : "imx462", sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx290->len_name ? imx290->len_name : "default", sizeof(inf->base.lens));
}

static long imx290_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx290 *imx290 = to_imx290(sd);
	if (cmd == RKMODULE_GET_MODULE_INFO) {
		imx290_get_module_inf(imx290, (struct rkmodule_inf *)arg);
		return 0;
	}
	return -ENOIOCTLCMD;
}

static const struct v4l2_subdev_core_ops imx290_core_ops = {
	.ioctl = imx290_ioctl,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx290_video_ops = {
	.s_stream = imx290_set_stream,
	.g_frame_interval = imx290_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx290_pad_ops = {
	.get_fmt = imx290_get_fmt,
	.set_fmt = imx290_get_fmt,
	.get_selection = imx290_get_selection,
	.get_mbus_config = imx290_get_mbus_config,
};

static const struct v4l2_subdev_ops imx290_subdev_ops = {
	.core = &imx290_core_ops,
	.video = &imx290_video_ops,
	.pad = &imx290_pad_ops,
};

static const struct media_entity_operations imx290_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx290_power_on(struct imx290 *imx290)
{
	int ret = clk_prepare_enable(imx290->xclk);
	if (ret) return ret;
	gpiod_set_value_cansleep(imx290->rst_gpio, 0);
	usleep_range(30000, 31000);
	return 0;
}

static void imx290_power_off(struct imx290 *imx290)
{
	clk_disable_unprepare(imx290->xclk);
	gpiod_set_value_cansleep(imx290->rst_gpio, 1);
}

static int imx290_runtime_resume(struct device *dev) {
	return imx290_power_on(to_imx290(dev_get_drvdata(dev)));
}

static int imx290_runtime_suspend(struct device *dev) {
	imx290_power_off(to_imx290(dev_get_drvdata(dev)));
	return 0;
}

static const struct dev_pm_ops imx290_pm_ops = {
	SET_RUNTIME_PM_OPS(imx290_runtime_suspend, imx290_runtime_resume, NULL)
};

static const struct regmap_config imx290_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int imx290_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx290 *imx290;
	int ret;

	imx290 = devm_kzalloc(dev, sizeof(*imx290), GFP_KERNEL);
	if (!imx290) return -ENOMEM;

	imx290->dev = dev;
	imx290->regmap = devm_regmap_init_i2c(client, &imx290_regmap_config);
	if (IS_ERR(imx290->regmap)) return -ENODEV;

	of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX, &imx290->module_index);
	of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING, &imx290->module_facing);
	of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME, &imx290->module_name);
	of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME, &imx290->len_name);

	imx290->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx290->xclk)) return PTR_ERR(imx290->xclk);

	imx290->rst_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx290->rst_gpio)) return PTR_ERR(imx290->rst_gpio);

	clk_set_rate(imx290->xclk, 37125000);
	ret = imx290_power_on(imx290);
	if (ret) return ret;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	v4l2_i2c_subdev_init(&imx290->sd, client, &imx290_subdev_ops);
	imx290->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	imx290->sd.dev = dev;
	imx290->sd.fwnode = dev_fwnode(dev);
	imx290->sd.entity.ops = &imx290_subdev_entity_ops;
	imx290->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx290->pad.flags = MEDIA_PAD_FL_SOURCE;
	media_entity_pads_init(&imx290->sd.entity, 1, &imx290->pad);

	v4l2_ctrl_handler_init(&imx290->ctrls, 5);
	imx290->gain = v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops, V4L2_CID_ANALOGUE_GAIN, 0, 98, 1, 0);
	imx290->exposure = v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops, V4L2_CID_EXPOSURE, 1, 1123, 1, 1000);
	imx290->vblank = v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops, V4L2_CID_VBLANK, 45, 0x3ffff - 1080, 1, 45);
	imx290->hblank = v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops, V4L2_CID_HBLANK, 280, 0xffff - 1920, 1, 280);
	imx290->link_freq = v4l2_ctrl_new_int_menu(&imx290->ctrls, &imx290_ctrl_ops, V4L2_CID_LINK_FREQ, 0, 0, link_freq_2lanes);
	if (imx290->link_freq) imx290->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx290->sd.ctrl_handler = &imx290->ctrls;

	snprintf(imx290->sd.name, sizeof(imx290->sd.name), "m00_b_imx462 %s", dev_name(dev));
	ret = v4l2_async_register_subdev_sensor_common(&imx290->sd);
	if (ret < 0) goto err_clean;

	dev_info(dev, "IMX462 Dedicated 1080p60 Driver Probed Successfully\n");
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;

err_clean:
	media_entity_cleanup(&imx290->sd.entity);
	v4l2_ctrl_handler_free(&imx290->ctrls);
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	imx290_power_off(imx290);
	return ret;
}

static int imx290_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx290 *imx290 = to_imx290(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&imx290->sd.entity);
	v4l2_ctrl_handler_free(&imx290->ctrls);
	pm_runtime_disable(imx290->dev);
	if (!pm_runtime_status_suspended(imx290->dev))
		imx290_power_off(imx290);
	pm_runtime_set_suspended(imx290->dev);
	return 0;
}

static const struct of_device_id imx290_of_match[] = {
	{ .compatible = "sony,imx290" },
	{ .compatible = "sony,imx462" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx290_of_match);

static struct i2c_driver imx290_i2c_driver = {
	.probe = imx290_probe,
	.remove = imx290_remove,
	.driver = {
		.name = "imx290",
		.pm = pm_ptr(&imx290_pm_ops),
		.of_match_table = imx290_of_match,
	},
};

module_i2c_driver(imx290_i2c_driver);

MODULE_DESCRIPTION("Sony IMX462 Dedicated 1080p60 Driver");
MODULE_LICENSE("GPL v2");