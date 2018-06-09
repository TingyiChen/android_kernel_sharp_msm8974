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
#include <mach/socinfo.h>

#define IMX214_SENSOR_NAME "imx214"
DEFINE_MSM_MUTEX(imx214_mut);

/* #define CONFIG_MSMB_CAMERA_DEBUG */
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static uint8_t *imx214_diag_data = NULL;

static struct msm_sensor_ctrl_t imx214_s_ctrl;

//static struct perf_lock imx214_perf_lock;

static struct msm_sensor_power_setting imx214_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
#if defined(CONFIG_MACH_LYNX_DL50)
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 9876500,
		.delay = 2,
	},
#elif defined(CONFIG_MACH_DECKARD_AS99)
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 6000000,
		.delay = 2,
	},
#elif defined(CONFIG_MACH_EBZ)
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
//		.config_val = 8000000,
		.config_val = 7400000,
		.delay = 2,
	},
#else
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 9876500,
		.delay = 2,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_sensor_power_setting imx214_power_off_setting[ARRAY_SIZE(imx214_power_setting)];

static struct v4l2_subdev_info imx214_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

int32_t imx214_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s", __func__);
	
	rc = msm_sensor_i2c_probe(client, id, &imx214_s_ctrl);
	if (rc < 0) {
		CDBG("%s msm_sensor_i2c_probe failed\n", __func__);
		return rc;
	}
	
	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (!s_ctrl) {
		pr_err("%s:%d sensor ctrl structure NULL\n", __func__,
			__LINE__);
		return -EINVAL;
	}
	
	return rc;
}

static const struct i2c_device_id imx214_i2c_id[] = {
	{IMX214_SENSOR_NAME, (kernel_ulong_t)&imx214_s_ctrl},
	{ }
};

static struct i2c_driver imx214_i2c_driver = {
	.id_table = imx214_i2c_id,
	.probe  = imx214_sensor_i2c_probe,
	.driver = {
		.name = IMX214_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx214_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx214_dt_match[] = {
	{.compatible = "qcom,imx214", .data = &imx214_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx214_dt_match);

static struct platform_driver imx214_platform_driver = {
	.driver = {
		.name = "qcom,imx214",
		.owner = THIS_MODULE,
		.of_match_table = imx214_dt_match,
	},
};

static int32_t imx214_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx214_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx214_init_module(void)
{
	int32_t rc = 0;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	int32_t camOtpData_size = 0;

	CDBG("%s:%d\n", __func__, __LINE__);
	
#if defined(CONFIG_MACH_EBZ)
	{
		enum msm_cpu cur_cpu = socinfo_get_msm_cpu();
		int i = 0;
		
		CDBG("%s cur_cpu = %d\n", __func__, cur_cpu);
		CDBG("%s socinfo_get_id() = %d\n", __func__, socinfo_get_id());
		if(cur_cpu == MSM_CPU_8974PRO_AA){
			pr_err("%s CPU is MSM_CPU_8974PRO_AA\n", __func__);
			for(i=0; i < ARRAY_SIZE(imx214_power_setting); i++){
				if(imx214_power_setting[i].seq_type == SENSOR_CLK){
					pr_err("%s before config_val = %ld\n", __func__, imx214_power_setting[i].config_val);
					imx214_power_setting[i].config_val = 8000000;
					pr_err("%s after config_val = %ld\n", __func__, imx214_power_setting[i].config_val);
					break;
				}
			}
		}
	}
#endif
	
	p_sh_smem_common_type = sh_smem_get_common_address();
	if(p_sh_smem_common_type != NULL){
		camOtpData_size = sizeof(p_sh_smem_common_type->sh_camOtpData);
	
		CDBG("%s camOtpData_size = %d\n", __func__, camOtpData_size);
	
		imx214_diag_data = kmalloc(camOtpData_size, GFP_KERNEL);
	
		if(imx214_diag_data != NULL){
			memcpy(imx214_diag_data, &p_sh_smem_common_type->sh_camOtpData[0], camOtpData_size);
		}
	}
	
//	perf_lock_init(&imx214_perf_lock, PERF_LOCK_729600KHz, "camera_imx214");
	
	rc = platform_driver_probe(&imx214_platform_driver,
		imx214_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx214_i2c_driver);
}

static void __exit imx214_exit_module(void)
{
	CDBG("%s:%d\n", __func__, __LINE__);
	if (imx214_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx214_s_ctrl);
		platform_driver_unregister(&imx214_platform_driver);
	} else
		i2c_del_driver(&imx214_i2c_driver);
		
	kfree(imx214_diag_data);
	imx214_diag_data = NULL;
	
	return;
}

int32_t imx214_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
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

int32_t imx214_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
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

int32_t imx214_smem_read(struct msm_sensor_ctrl_t *s_ctrl,
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
		
	if(imx214_diag_data == NULL){
		pr_err("%s imx214_diag_data == NULL\n",__func__);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.smem_info.data,
		&imx214_diag_data[cdata->cfg.smem_info.addr],
		cdata->cfg.smem_info.length)){
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	
	return rc;
}

int32_t imx214_smem_write(struct msm_sensor_ctrl_t *s_ctrl,
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
		
	if(imx214_diag_data == NULL){
		pr_err("%s imx214_diag_data == NULL\n",__func__);
		return -EFAULT;
	}
	
	if (copy_from_user(&imx214_diag_data[cdata->cfg.smem_info.addr],
		(void *)cdata->cfg.smem_info.data,
		cdata->cfg.smem_info.length)){
		pr_err("%s copy_from_user error\n",__func__);
		return -EFAULT;
	}
	
	return rc;
}

int32_t imx214_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
		case SHCFG_GET_I2C_DATA:
			rc = imx214_i2c_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx081_i2c_read failed", __func__);
			}
			break;
		case CFG_WRITE_I2C_ARRAY:
			rc = imx214_i2c_write(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx081_i2c_write failed", __func__);
			}
			break;
		case SHCFG_GET_SMEM_DATA:
			rc = imx214_smem_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx081_smem_read failed", __func__);
			}
			break;
		case SHCFG_SET_SMEM_DATA:
			rc = imx214_smem_write(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx081_smem_read failed", __func__);
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

int32_t imx214_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s\n", __func__);
	
	s_ctrl->power_setting_array.power_setting = imx214_power_setting;
	s_ctrl->power_setting_array.size = ARRAY_SIZE(imx214_power_setting);
	
	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n",
			__func__);
		return rc;
	}

//	perf_lock(&imx214_perf_lock);
	return rc;
}

int32_t imx214_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	int32_t clk_index = 0;
	int32_t power_off_index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;

	CDBG("%s\n", __func__);
	
	power_setting_array = &s_ctrl->power_setting_array;
	
	for (index = 0; index < power_setting_array->size; index++) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
			case SENSOR_CLK:
				clk_index = index;
				CDBG("%s clk_index = %d\n", __func__, clk_index);
				break;
			case SENSOR_GPIO:
				memcpy(&imx214_power_off_setting[power_off_index], power_setting, sizeof(struct msm_sensor_power_setting));
				power_off_index ++;
				break;
			case SENSOR_VREG:
				memcpy(&imx214_power_off_setting[power_off_index], power_setting, sizeof(struct msm_sensor_power_setting));
				power_off_index ++;
				break;
			case SENSOR_I2C_MUX:
				memcpy(&imx214_power_off_setting[power_off_index], &power_setting_array->power_setting[clk_index], sizeof(struct msm_sensor_power_setting));
				power_off_index ++;
				memcpy(&imx214_power_off_setting[power_off_index], power_setting, sizeof(struct msm_sensor_power_setting));
				power_off_index ++;
				break;
			default:
				pr_err("%s error power seq type %d\n", __func__,
					power_setting->seq_type);
				break;
		}
	}
	
	if(index == power_off_index){
		CDBG("%s exchang power_off setting\n", __func__);
		s_ctrl->power_setting_array.power_setting = imx214_power_off_setting;
	}
	
	rc = msm_sensor_power_down(s_ctrl);
	
//	perf_unlock(&imx214_perf_lock);

	return rc;
}

static struct msm_sensor_fn_t imx214_sensor_func_tbl = {
	.sensor_config = imx214_sensor_config,
	.sensor_power_up = imx214_sensor_power_up,
	.sensor_power_down = imx214_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t imx214_s_ctrl = {
	.sensor_i2c_client = &imx214_sensor_i2c_client,
	.power_setting_array.power_setting = imx214_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx214_power_setting),
	.msm_sensor_mutex = &imx214_mut,
	.sensor_v4l2_subdev_info = imx214_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx214_subdev_info),
	.func_tbl = &imx214_sensor_func_tbl,
};

module_init(imx214_init_module);
module_exit(imx214_exit_module);
MODULE_DESCRIPTION("imx214");
MODULE_LICENSE("GPL v2");
