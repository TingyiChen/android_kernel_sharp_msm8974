/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_camera_io_util.h"
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <sharp/sh_smem.h>
#include <mach/perflock.h>
#include <mach/socinfo.h>
#include <sharp/sh_boot_manager.h>

#define S5K4H5YB_SENSOR_NAME "s5k4h5yb"
DEFINE_MSM_MUTEX(s5k4h5yb_mut);

/* #define CONFIG_MSMB_CAMERA_DEBUG */
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static uint8_t *s5k4h5yb_diag_data = NULL;
static uint8_t *s5k4h5yb_otp_data = NULL;

static int32_t msm_s5k4h5yb_i2c_probe(
	struct i2c_client *, const struct i2c_device_id *);

static struct msm_sensor_power_setting s5k4h5yb_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 18750000,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_sensor_power_setting s5k4h5yb_power_setting_pp1[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 18750000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

// TODO: Update this info once you get settings
static struct v4l2_subdev_info s5k4h5yb_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static struct msm_camera_i2c_client s5k4h5yb_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


int32_t s5k4h5yb_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	void *data;
	
	CDBG("%s start\n", __func__);
	
	data = kmalloc(cdata->cfg.i2c_info.length, GFP_KERNEL);
	if(data == NULL){
		pr_err("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}
	
	CDBG("%s i2c_info.addr = 0x%0x\n", __func__, cdata->cfg.i2c_info.addr);
	CDBG("%s i2c_info.length = 0x%0x\n", __func__, cdata->cfg.i2c_info.length);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(s_ctrl->sensor_i2c_client, cdata->cfg.i2c_info.addr, data, cdata->cfg.i2c_info.length);
	if(rc < 0){
		pr_err("%s i2c_read_seq failed\n", __func__);
		kfree(data);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.i2c_info.data,
		data,
		cdata->cfg.i2c_info.length)){
		kfree(data);
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	kfree(data);
	
	return rc;
}

int32_t s5k4h5yb_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	struct msm_camera_i2c_reg_setting conf_array;
	struct msm_camera_i2c_reg_array *reg_setting = NULL;
	struct msm_camera_i2c_reg_array *pos_reg_setting = NULL;
	struct msm_camera_i2c_seq_reg_setting conf_sec_array;
	struct msm_camera_i2c_seq_reg_array *reg_sec_setting = NULL;
	struct msm_camera_i2c_seq_reg_array *pos_reg_sec_setting = NULL;
	int i;
	int cur_size, array_size;

	if (copy_from_user(&conf_array,
		(void *)cdata->cfg.setting,
		sizeof(struct msm_camera_i2c_reg_setting))) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}

	reg_setting = kzalloc(conf_array.size *
		(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
	if (!reg_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}
	reg_sec_setting = kzalloc(conf_array.size *
		(sizeof(struct msm_camera_i2c_seq_reg_array)),
		GFP_KERNEL);
	if (!reg_sec_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
		conf_array.size *
		sizeof(struct msm_camera_i2c_reg_array))) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		kfree(reg_setting);
		kfree(reg_sec_setting);
		return -EFAULT;
	}
	
	if(conf_array.data_type == MSM_CAMERA_I2C_BYTE_DATA){
	
		pos_reg_setting = reg_setting;
		pos_reg_sec_setting = reg_sec_setting;
		
		array_size = 1;
		conf_sec_array.addr_type = conf_array.addr_type;
		conf_sec_array.delay = conf_array.delay;
		conf_sec_array.reg_setting = reg_sec_setting;
		
		pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
		pos_reg_sec_setting->reg_data[0] = pos_reg_setting->reg_data;
		cur_size = 1;
		pos_reg_setting++;
		
		for (i = 1; i < conf_array.size; i++) {
			if(pos_reg_setting->reg_addr == (pos_reg_sec_setting->reg_addr + cur_size)){
				if(cur_size >= 7){
					pos_reg_sec_setting->reg_data_size = cur_size;
					
					pos_reg_sec_setting++;
					array_size++;
					
					cur_size = 0;
					pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
				}
				pos_reg_sec_setting->reg_data[cur_size] = pos_reg_setting->reg_data;
				cur_size++;
			} else {
				pos_reg_sec_setting->reg_data_size = cur_size;
				
				pos_reg_sec_setting++;
				array_size++;
				
				pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
				pos_reg_sec_setting->reg_data[0] = pos_reg_setting->reg_data;
				cur_size = 1;
			}
			pos_reg_setting++;
		}

		CDBG("%s array_size = %d", __func__, array_size);
		pos_reg_sec_setting->reg_data_size = cur_size;

		conf_sec_array.size = array_size;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_sec_array);
		
		CDBG("%s rc = %d", __func__, (int)rc);

	} else {

		CDBG("%s conf_array.data_type = %d", __func__, conf_array.data_type);
		
		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
			
		CDBG("%s rc = %d", __func__, (int)rc);
	}
	
	kfree(reg_setting);
	kfree(reg_sec_setting);

	CDBG("%s end", __func__);
	
	return rc;
}

int32_t s5k4h5yb_smem_read(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	int32_t camOtpData_size = 0;
	
	CDBG("%s start\n", __func__);
	
	CDBG("%s smem_info.addr = 0x%0x\n", __func__, cdata->cfg.smem_info.addr);
	CDBG("%s smem_info.length = 0x%0x\n", __func__, cdata->cfg.smem_info.length);
	
	p_sh_smem_common_type = sh_smem_get_common_address();
	if(p_sh_smem_common_type == NULL){
		pr_err("%s p_sh_smem_common_type == NULL\n",__func__);
		return -EFAULT;
	}
	
	camOtpData_size = sizeof(p_sh_smem_common_type->sh_camOtpData);
		
	if((cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length) > camOtpData_size){
		pr_err("%s length error %d : camOtpData_size = %d\n",__func__, (cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length), camOtpData_size);
		return -EFAULT;
	}
		
	if(s5k4h5yb_diag_data == NULL){
		pr_err("%s s5k4h5yb_diag_data == NULL\n",__func__);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.smem_info.data,
		&s5k4h5yb_diag_data[cdata->cfg.smem_info.addr],
		cdata->cfg.smem_info.length)){
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	
	return rc;
}

int32_t s5k4h5yb_smem_write(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	int32_t camOtpData_size = 0;
	
	CDBG("%s start\n", __func__);
	
	CDBG("%s smem_info.addr = 0x%0x\n", __func__, cdata->cfg.smem_info.addr);
	CDBG("%s smem_info.length = 0x%0x\n", __func__, cdata->cfg.smem_info.length);
	
	p_sh_smem_common_type = sh_smem_get_common_address();
	if(p_sh_smem_common_type == NULL){
		pr_err("%s p_sh_smem_common_type == NULL\n",__func__);
		return -EFAULT;
	}
	
	camOtpData_size = sizeof(p_sh_smem_common_type->sh_camOtpData);
		
	if((cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length) > camOtpData_size){
		pr_err("%s length error %d : camOtpData_size = %d\n",__func__, (cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length), camOtpData_size);
		return -EFAULT;
	}
		
	if(s5k4h5yb_diag_data == NULL){
		pr_err("%s s5k4h5yb_diag_data == NULL\n",__func__);
		return -EFAULT;
	}
	
	if (copy_from_user(&s5k4h5yb_diag_data[cdata->cfg.smem_info.addr],
		(void *)cdata->cfg.smem_info.data,
		cdata->cfg.smem_info.length)){
		pr_err("%s copy_from_user error\n",__func__);
		return -EFAULT;
	}
	
	return rc;
}


int32_t s5k4h5yb_otp_read(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	
	CDBG("%s start\n", __func__);
	
	CDBG("%s otp_info.addr = 0x%0x\n", __func__, cdata->cfg.otp_info.addr);
	CDBG("%s otp_info.length = 0x%0x\n", __func__, cdata->cfg.otp_info.length);
	
	if((cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length) > 1024){
		pr_err("%s length error %d\n",__func__, (cdata->cfg.smem_info.addr + cdata->cfg.smem_info.length));
		return -EFAULT;
	}
		
	if(s5k4h5yb_otp_data == NULL){
		pr_err("%s s5k4h5yb_otp_data == NULL\n",__func__);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.otp_info.data,
		&s5k4h5yb_otp_data[cdata->cfg.otp_info.addr],
		cdata->cfg.otp_info.length)){
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	
	return rc;
}

int32_t s5k4h5yb_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
		case SHCFG_GET_I2C_DATA:
			rc = s5k4h5yb_i2c_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k4h5yb_i2c_read failed", __func__);
			}
			break;
		case CFG_WRITE_I2C_ARRAY:
			rc = s5k4h5yb_i2c_write(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k4h5yb_i2c_write failed", __func__);
			}
			break;
		case SHCFG_GET_SMEM_DATA:
			rc = s5k4h5yb_smem_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k4h5yb_smem_read failed", __func__);
			}
			break;
		case SHCFG_SET_SMEM_DATA:
			rc = s5k4h5yb_smem_write(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k4h5yb_smem_write failed", __func__);
			}
			break;
		case SHCFG_GET_OTP_DATA:
			rc = s5k4h5yb_otp_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s s5k4h5yb_otp_read failed", __func__);
			}
			break;
		default:
			mutex_unlock(s_ctrl->msm_sensor_mutex);
			rc = msm_sensor_config(s_ctrl, argp);
			return rc;
			break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t s5k4h5yb_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s\n", __func__);
	
	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n",
			__func__);
		return rc;
	}

	return rc;
}

int32_t s5k4h5yb_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s\n", __func__);
	
	rc = msm_sensor_power_down(s_ctrl);

	return rc;
}


static struct msm_sensor_fn_t s5k4h5yb_sensor_func_tbl = {
	.sensor_config = s5k4h5yb_sensor_config,
	.sensor_power_up = s5k4h5yb_sensor_power_up,
	.sensor_power_down = s5k4h5yb_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t s5k4h5yb_s_ctrl = {
	.sensor_i2c_client = &s5k4h5yb_sensor_i2c_client,
	.power_setting_array.power_setting = s5k4h5yb_power_setting,
	.power_setting_array.size = ARRAY_SIZE(s5k4h5yb_power_setting),
	.msm_sensor_mutex = &s5k4h5yb_mut,
	.sensor_v4l2_subdev_info = s5k4h5yb_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k4h5yb_subdev_info),
	.func_tbl = &s5k4h5yb_sensor_func_tbl,
};

static const struct i2c_device_id s5k4h5yb_i2c_id[] = {
	{S5K4H5YB_SENSOR_NAME, (kernel_ulong_t)&s5k4h5yb_s_ctrl},
	{ }
};

static struct i2c_driver s5k4h5yb_i2c_driver = {
	.id_table = s5k4h5yb_i2c_id,
	.probe  = msm_s5k4h5yb_i2c_probe,
	.driver = {
		.name = S5K4H5YB_SENSOR_NAME,
	},
};

MODULE_DEVICE_TABLE(of, s5k4h5yb_dt_match);

static const struct of_device_id s5k4h5yb_dt_match[] = {
	{.compatible = "qcom,s5k4h5yb", .data = &s5k4h5yb_s_ctrl},
	{}
};

static struct platform_driver s5k4h5yb_platform_driver = {
	.driver = {
		.name = "qcom,s5k4h5yb",
		.owner = THIS_MODULE,
		.of_match_table = s5k4h5yb_dt_match,
	},
};

static int32_t msm_s5k4h5yb_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &s5k4h5yb_s_ctrl);
}

static int32_t s5k4h5yb_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	struct msm_sensor_ctrl_t *s_ctrl;
	uint16_t	rev = 0;
	
	match = of_match_device(s5k4h5yb_dt_match, &pdev->dev);
	
	s_ctrl = (struct msm_sensor_ctrl_t *)match->data;
	rev = sh_boot_get_hw_revision();
	
#if defined(CONFIG_MACH_LYNX_GP9D)
	CDBG("%s rev=0x%0x\n", __func__, rev);
	if((rev & 0x01) == 0){
		s_ctrl->power_setting_array.power_setting = s5k4h5yb_power_setting_pp1;
		s_ctrl->power_setting_array.size = ARRAY_SIZE(s5k4h5yb_power_setting_pp1);
	}
#endif
	rc = msm_sensor_platform_probe(pdev, match->data);
	
	if(rc < 0){
		return rc;
	} else {
		/* OTP read */
		int i = 0;
		long before_config_val = 0;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array reg_setting;
		struct msm_camera_i2c_reg_array init_reg_setting[] = {
			{ 0x3b03, 0x00 , 0x00},
			{ 0x3b04, 0x66 , 0x00 },
			{ 0x3b05, 0x01 , 0x00 },
			{ 0x3b06, 0xe6 , 0x00 },
			{ 0x3b07, 0x06 , 0x00 },
			{ 0x3b08, 0x08 , 0x00 },
			{ 0x3b0a, 0x02 , 0x00 },
			{ 0x3b0b, 0x76 , 0x00 },
			{ 0x3b0d, 0x0a , 0x00 },
			{ 0x3b0e, 0x1d , 0x00 },
			{ 0x3b0f, 0x01 , 0x00 },
			{ 0x3b10, 0xfd , 0x00 },
		};
		
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
		conf_array.delay = 0;
		
		for(i=0; i < s_ctrl->power_setting_array.size; i++){
			if(s_ctrl->power_setting_array.power_setting[i].seq_type == SENSOR_CLK){
				before_config_val = s_ctrl->power_setting_array.power_setting[i].config_val;
				CDBG("%s before config_val = %ld\n", __func__, before_config_val);
				s_ctrl->power_setting_array.power_setting[i].config_val = 24000000;
				CDBG("%s after config_val = %ld\n", __func__, s_ctrl->power_setting_array.power_setting[i].config_val);
				break;
			}
		}
		
		rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("%s %s power up failed\n", __func__,s_ctrl->sensordata->sensor_name);
			return rc;
		}
		
		s5k4h5yb_otp_data = kmalloc(1024, GFP_KERNEL);
	
		if(s5k4h5yb_otp_data != NULL){
			conf_array.size = ARRAY_SIZE(init_reg_setting);
			conf_array.reg_setting = init_reg_setting;
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
			
			conf_array.size = 1;
			conf_array.reg_setting = &reg_setting;
			reg_setting.reg_addr = 0x3a02;
			reg_setting.reg_data = 0x00;
			reg_setting.delay = 0x00;
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
			
			reg_setting.reg_addr = 0x3a00;
			reg_setting.reg_data = 0x01;
			reg_setting.delay = 0x00;
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
			
			for(i = 0; i < 16; i++){
				if(i != 0){
					reg_setting.reg_addr = 0x3a02;
					reg_setting.reg_data = i;
					reg_setting.delay = 0x00;
					s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
						s_ctrl->sensor_i2c_client, &conf_array);
				}
				
				usleep_range(350,1350);
				
				s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(s_ctrl->sensor_i2c_client, 0x3a04, &s5k4h5yb_otp_data[i * 64], 64);
				
				
			}
			reg_setting.reg_addr = 0x3a00;
			reg_setting.reg_data = 0x00;
			reg_setting.delay = 0x00;
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
		}
		
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		
		for(i=0; i < s_ctrl->power_setting_array.size; i++){
			if(s_ctrl->power_setting_array.power_setting[i].seq_type == SENSOR_CLK){
				s_ctrl->power_setting_array.power_setting[i].config_val = before_config_val;
				CDBG("%s after config_val = %ld\n", __func__, s_ctrl->power_setting_array.power_setting[i].config_val);
				break;
			}
		}
		
		CDBG("%s s5k4h5yb_otp_data\n", __func__);
		for(i=0;i < 127;i++){
			CDBG("%s %02x %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,", __func__, i
					, *(s5k4h5yb_otp_data + i*8 + 0)
					, *(s5k4h5yb_otp_data + i*8 + 1)
					, *(s5k4h5yb_otp_data + i*8 + 2)
					, *(s5k4h5yb_otp_data + i*8 + 3)
					, *(s5k4h5yb_otp_data + i*8 + 4)
					, *(s5k4h5yb_otp_data + i*8 + 5)
					, *(s5k4h5yb_otp_data + i*8 + 6)
					, *(s5k4h5yb_otp_data + i*8 + 7));
		}
		
	}
	return rc;
}

static int __init s5k4h5yb_init_module(void)
{
	int32_t rc = 0;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	int32_t camOtpData_size = 0;
	
	pr_info("%s:%d\n", __func__, __LINE__);
	
	p_sh_smem_common_type = sh_smem_get_common_address();
	if(p_sh_smem_common_type != NULL){
		camOtpData_size = sizeof(p_sh_smem_common_type->sh_camOtpData);
	
		CDBG("%s camOtpData_size = %d\n", __func__, camOtpData_size);
	
		s5k4h5yb_diag_data = kmalloc(camOtpData_size, GFP_KERNEL);
	
		if(s5k4h5yb_diag_data != NULL){
			memcpy(s5k4h5yb_diag_data, &p_sh_smem_common_type->sh_camOtpData[0], camOtpData_size);
		}
	}
	
	rc = platform_driver_probe(&s5k4h5yb_platform_driver,
		s5k4h5yb_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&s5k4h5yb_i2c_driver);
}

static void __exit s5k4h5yb_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (s5k4h5yb_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&s5k4h5yb_s_ctrl);
		platform_driver_unregister(&s5k4h5yb_platform_driver);
	} else
		i2c_del_driver(&s5k4h5yb_i2c_driver);

	kfree(s5k4h5yb_diag_data);
	s5k4h5yb_diag_data = NULL;

	return;
}

module_init(s5k4h5yb_init_module);
module_exit(s5k4h5yb_exit_module);
MODULE_DESCRIPTION("s5k4h5yb");
MODULE_LICENSE("GPL v2");
