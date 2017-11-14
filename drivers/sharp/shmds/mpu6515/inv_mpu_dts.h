/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef _INV_MPU_DTS_H_
#define _INV_MPU_DTS_H_

#include <linux/i2c.h>
/* SHMDS_ID_0103_01 mod S */
//#include <linux/mpu.h>
#include <sharp/mpu6515.h>
/* SHMDS_ID_0103_01 mod E */

int inv_mpu_power_on(struct mpu_platform_data *pdata);
int inv_mpu_power_off(struct mpu_platform_data *pdata);
int inv_parse_orientation_matrix(struct device *dev, s8 *orient);
int inv_parse_secondary_orientation_matrix(struct device *dev,
							s8 *orient);
int inv_parse_secondary(struct device *dev, struct mpu_platform_data *pdata);
int inv_parse_aux(struct device *dev, struct mpu_platform_data *pdata);
int invensense_mpu_parse_dt(struct device *dev,
			    struct mpu_platform_data *pdata);

#endif  /* #ifndef _INV_MPU_DTS_H_ */
