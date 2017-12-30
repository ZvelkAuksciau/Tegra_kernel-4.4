/*
 * ov5693_v4l2.c - ov5693 sensor driver
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG 1

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/camera_common.h>
#include <media/ov5693.h>

#include "../platform/tegra/camera/camera_gpio.h"

//#include "ov5693_mode_tbls.h"

#define OV5693_MAX_COARSE_DIFF		6

#define OV5693_GAIN_SHIFT		8
#define OV5693_REAL_GAIN_SHIFT		4
#define OV5693_MIN_GAIN		(1 << OV5693_GAIN_SHIFT)
#define OV5693_MAX_GAIN		(16 << OV5693_GAIN_SHIFT)
#define OV5693_MAX_UNREAL_GAIN	(0x0F80)
#define OV5693_MIN_FRAME_LENGTH	(0x0)
#define OV5693_MAX_FRAME_LENGTH	(0x7fff)
#define OV5693_MIN_EXPOSURE_COARSE	(0x0002)
#define OV5693_MAX_EXPOSURE_COARSE	\
	(OV5693_MAX_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)
#define OV5693_DEFAULT_LINE_LENGTH	(0xA80)
#define OV5693_DEFAULT_PIXEL_CLOCK	(160)

#define OV5693_DEFAULT_GAIN		OV5693_MIN_GAIN
#define OV5693_DEFAULT_FRAME_LENGTH	(0x07C0)
#define OV5693_DEFAULT_EXPOSURE_COARSE	\
	(OV5693_DEFAULT_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)

#define OV5693_DEFAULT_WIDTH	1920
#define OV5693_DEFAULT_HEIGHT	1080
#define OV5693_DEFAULT_DATAFMT	MEDIA_BUS_FMT_UYVY8_2X8
#define OV5693_DEFAULT_CLK_FREQ	24000000

struct ov5693 {
	struct camera_common_power_rail	power;
	int				numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct camera_common_eeprom_data eeprom[OV5693_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[OV5693_EEPROM_SIZE];
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	int				reg_offset;

	s32				group_hold_prev;
	u32				frame_length;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static int ov5693_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ov5693_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops ov5693_ctrl_ops = {
	.g_volatile_ctrl = ov5693_g_volatile_ctrl,
	.s_ctrl		= ov5693_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV5693_MIN_GAIN,
		.max = OV5693_MAX_GAIN,
		.def = OV5693_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV5693_MIN_FRAME_LENGTH,
		.max = OV5693_MAX_FRAME_LENGTH,
		.def = OV5693_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV5693_MIN_EXPOSURE_COARSE,
		.max = OV5693_MAX_EXPOSURE_COARSE,
		.def = OV5693_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV5693_MIN_EXPOSURE_COARSE,
		.max = OV5693_MAX_EXPOSURE_COARSE,
		.def = OV5693_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.max = OV5693_EEPROM_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV5693_OTP_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov5693_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV5693_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static int test_mode;
module_param(test_mode, int, 0644);

static inline int ov5693_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	return 0;
}

static int ov5693_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	return 0;
}

static int ov5693_power_on(struct camera_common_data *s_data)
{
	return 0;
}

static int ov5693_power_off(struct camera_common_data *s_data)
{
	return 0;
}

static int ov5693_power_put(struct ov5693 *priv)
{
	return 0;
}

static int ov5693_power_get(struct ov5693 *priv)
{
	return 0;
}

static int ov5693_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct ov5693 *priv = (struct ov5693 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++\n", __func__);

	if (!enable) {
		return 0;
	}

	if (s_data->override_enable) {
		/*
		 * write list of override regs for the asking frame length,
		 * coarse integration time, and gain. Failures to write
		 * overrides are non-fatal
		 */
		control.id = V4L2_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);

		control.id = V4L2_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);

		control.id = V4L2_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);

		control.id = V4L2_CID_COARSE_TIME_SHORT;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	}

	if (test_mode) {

	}

	dev_dbg(&client->dev, "%s--\n", __func__);
	return 0;
}

static int ov5693_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct ov5693 *priv = (struct ov5693 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops ov5693_subdev_video_ops = {
	.s_stream	= ov5693_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = ov5693_g_input_status,
};

static struct v4l2_subdev_core_ops ov5693_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int ov5693_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int ov5693_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops ov5693_subdev_pad_ops = {
	.set_fmt = ov5693_set_fmt,
	.get_fmt = ov5693_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ov5693_subdev_ops = {
	.core	= &ov5693_subdev_core_ops,
	.video	= &ov5693_subdev_video_ops,
	.pad	= &ov5693_subdev_pad_ops,
};

static struct of_device_id ov5693_of_match[] = {
	{ .compatible = "kmti,kmtifpga", },
	{ },
};

static struct camera_common_sensor_ops ov5693_common_ops = {
	.power_on = ov5693_power_on,
	.power_off = ov5693_power_off,
	.write_reg = ov5693_write_reg,
	.read_reg = ov5693_read_reg,
};

static int ov5693_set_group_hold(struct ov5693 *priv)
{
	return 0;
}

static int ov5693_set_gain(struct ov5693 *priv, s32 val)
{
	return 0;
}

static int ov5693_set_frame_length(struct ov5693 *priv, s32 val)
{
	return 0;
}

static int ov5693_set_coarse_time(struct ov5693 *priv, s32 val)
{
	return 0;
}

static int ov5693_set_coarse_time_short(struct ov5693 *priv, s32 val)
{
	return 0;
}

static int ov5693_read_eeprom(struct ov5693 *priv,
				struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int ov5693_write_eeprom(struct ov5693 *priv,
				char *string)
{
	return 0;
}

static int ov5693_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5693 *priv =
		container_of(ctrl->handler, struct ov5693, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EEPROM_DATA:
		err = ov5693_read_eeprom(priv, ctrl);
		if (err)
			return err;
		break;
	default:
			pr_err("%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int ov5693_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5693 *priv =
		container_of(ctrl->handler, struct ov5693, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = ov5693_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = ov5693_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = ov5693_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = ov5693_set_coarse_time_short(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = ov5693_set_group_hold(priv);
		}
		break;
	case V4L2_CID_EEPROM_DATA:
		if (!ctrl->p_new.p_char[0])
			break;
		err = ov5693_write_eeprom(priv, ctrl->p_new.p_char);
		if (err)
			return err;
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int ov5693_ctrls_init(struct ov5693 *priv, bool eeprom_ctrl)
{
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_data *common_data = priv->s_data;
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {
		/* Skip control 'V4L2_CID_EEPROM_DATA' if eeprom inint err */
		if (ctrl_config_list[i].id == V4L2_CID_EEPROM_DATA) {
			if (!eeprom_ctrl) {
				common_data->numctrls -= 1;
				continue;
			}
		}

		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char)
				return -ENOMEM;
		}
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = numctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, ov5693_of_match);

static struct camera_common_pdata *ov5693_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;
	struct camera_common_pdata *ret = NULL;

	if (!node)
		return NULL;

	match = of_match_device(ov5693_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(client, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		dev_err(&client->dev, "pwdn gpios not in DT\n");
		goto error;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (gpio < 0) {
		/* reset-gpio is not absolutely needed */
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		dev_dbg(&client->dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	board_priv_pdata->use_cam_gpio =
		of_property_read_bool(node, "cam,use-cam-gpio");

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return ret;
}

static int ov5693_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5693_subdev_internal_ops = {
	.open = ov5693_open,
};

static const struct media_entity_operations ov5693_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

//=================From tbls header============================================

enum {
	OV5693_MODE_1920x1080,
};

static const int ov5693_30fps[] = {
	30,
};


static const struct camera_common_frmfmt ov5693_frmfmt[] = {
	{{1920, 1080},	ov5693_30fps,	1, 0,	OV5693_MODE_1920x1080},
};

static struct regmap_config ov5693_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int ov5693_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct ov5693 *priv;
	char debugfs_name[10];
	int err;

	pr_info("[KMTIFPGA]: probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data)
		return -ENOMEM;

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct ov5693) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(client, &ov5693_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = ov5693_parse_dt(client);
	if (PTR_ERR(priv->pdata) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &ov5693_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= ov5693_frmfmt;
	common_data->colorfmt		= camera_common_find_datafmt(
					  OV5693_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(ov5693_frmfmt);
	common_data->def_mode		= OV5693_MODE_1920x1080;
	common_data->def_width		= OV5693_DEFAULT_WIDTH;
	common_data->def_height		= OV5693_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= OV5693_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->s_data->dev		= &client->dev;

	err = ov5693_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(client, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "ov5693_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);
	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &ov5693_subdev_ops);

	err = ov5693_ctrls_init(priv, !err);
	if (err)
		return err;

	priv->subdev->internal_ops = &ov5693_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &ov5693_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_dbg(&client->dev, "Detected KMTIFpga sensor\n");


	return 0;
}

static int
ov5693_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct ov5693 *priv = (struct ov5693 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ov5693_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id ov5693_id[] = {
	{ "kmtifpga", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5693_id);

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "kmtifpga",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov5693_of_match),
	},
	.probe = ov5693_probe,
	.remove = ov5693_remove,
	.id_table = ov5693_id,
};

module_i2c_driver(ov5693_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for KMTI FPGA");
MODULE_AUTHOR("Matas Razgunas <matas.razgunas@gmail.com");
MODULE_LICENSE("GPL v2");

