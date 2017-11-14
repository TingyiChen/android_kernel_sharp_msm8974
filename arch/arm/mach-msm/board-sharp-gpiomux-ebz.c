/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>

#define WLAN_CLK	40
#define WLAN_SET	39
#define WLAN_DATA0	38
#define WLAN_DATA1	37
#define WLAN_DATA2	36

static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gpio_suspend_config[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,  /* IN-NP */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,  /* O-LOW */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
};

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting wcnss_5gpio_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5gpio_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting taiko_reset = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting taiko_int = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_uart7_active_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_uart7_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_blsp2_uart7_configs[] __initdata = {
	{
		.gpio	= 43,	/* BLSP2 UART7 CTS */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 44,	/* BLSP2 UART7 RFR */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8974_slimbus_config[] __initdata = {
	{
		.gpio	= 70,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 71,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
	{
		.func = GPIOMUX_FUNC_1, /*i2c suspend*/ /* 2 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 0*/ /* 3 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend 0*/ /* 4 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 24, /* FLASH_LED_NOW */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config wcnss_5gpio_interface[] = {
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_taiko_config[] __initdata = {
	{
		.gpio	= 63,		/* SYS_RST_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_reset,
		},
	},
	{
		.gpio	= 72,		/* CDC_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_int,
		},
	},
};

/* suspended */
static struct gpiomux_setting sh_sus_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_sus_func1_pd_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_sus_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func3_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_sus_func4_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func5_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_np_4ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_np_6ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_pd_10ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/* active */
static struct gpiomux_setting sh_act_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_4ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_act_func1_pd_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_act_func2_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func2_pu_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting sh_act_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func4_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func5_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_act_gpio_pd_10ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_10MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_act_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sh_msm8974_gpio_configs[] __initdata = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 8,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_4ma_out_low_cfg,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func4_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func4_np_2ma_cfg,
		},
	},
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func5_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func5_np_2ma_cfg,
		},
	},
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func4_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func4_np_2ma_cfg,
		},
	},
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_pd_2ma_cfg,
		},
	},
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func2_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func2_pu_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_10ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_10ma_out_low_cfg,
		},
	},
	{
		.gpio = 50,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 57,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 59,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 61,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 62,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 74,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 75,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 79,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 82,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 83,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_2ma_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_2ma_cfg,
		},
	},
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 104,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 105,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 110,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 111,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 118,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 121,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 123,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 129,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 130,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 131,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 132,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 135,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 137,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 144,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 145,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
};

void __init msm_8974_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}
	pr_err("%s:%d socinfo_get_version %x\n", __func__, __LINE__,
		socinfo_get_version());
	if (socinfo_get_version() >= 0x20000)
		msm_tlmm_misc_reg_write(TLMM_SPARE_REG, 0xf);


	msm_gpiomux_install(msm_blsp2_uart7_configs,
			 ARRAY_SIZE(msm_blsp2_uart7_configs));
	msm_gpiomux_install(wcnss_5wire_interface,
				ARRAY_SIZE(wcnss_5wire_interface));

	msm_gpiomux_install(msm8974_slimbus_config,
			ARRAY_SIZE(msm8974_slimbus_config));

	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));

	msm_gpiomux_install(msm_taiko_config, ARRAY_SIZE(msm_taiko_config));

	sh_msm_gpiomux_install(sh_msm8974_gpio_configs,
			ARRAY_SIZE(sh_msm8974_gpio_configs));
}

static void wcnss_switch_to_gpio(void)
{
	/* Switch MUX to GPIO */
	msm_gpiomux_install(wcnss_5gpio_interface,
			ARRAY_SIZE(wcnss_5gpio_interface));

	/* Ensure GPIO config */
	gpio_direction_input(WLAN_DATA2);
	gpio_direction_input(WLAN_DATA1);
	gpio_direction_input(WLAN_DATA0);
	gpio_direction_output(WLAN_SET, 0);
	gpio_direction_output(WLAN_CLK, 0);
}

static void wcnss_switch_to_5wire(void)
{
	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));
}

u32 wcnss_rf_read_reg(u32 rf_reg_addr)
{
	int count = 0;
	u32 rf_cmd_and_addr = 0;
	u32 rf_data_received = 0;
	u32 rf_bit = 0;

	wcnss_switch_to_gpio();

	/* Reset the signal if it is already being used. */
	gpio_set_value(WLAN_SET, 0);
	gpio_set_value(WLAN_CLK, 0);

	/* We start with cmd_set high WLAN_SET = 1. */
	gpio_set_value(WLAN_SET, 1);

	gpio_direction_output(WLAN_DATA0, 1);
	gpio_direction_output(WLAN_DATA1, 1);
	gpio_direction_output(WLAN_DATA2, 1);

	gpio_set_value(WLAN_DATA0, 0);
	gpio_set_value(WLAN_DATA1, 0);
	gpio_set_value(WLAN_DATA2, 0);

	/* Prepare command and RF register address that need to sent out.
	 * Make sure that we send only 14 bits from LSB.
	 */
	rf_cmd_and_addr  = (((WLAN_RF_READ_REG_CMD) |
		(rf_reg_addr << WLAN_RF_REG_ADDR_START_OFFSET)) &
		WLAN_RF_READ_CMD_MASK);

	for (count = 0; count < 5; count++) {
		gpio_set_value(WLAN_CLK, 0);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA0, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA1, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA2, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		/* Send the data out WLAN_CLK = 1 */
		gpio_set_value(WLAN_CLK, 1);
	}

	/* Pull down the clock signal */
	gpio_set_value(WLAN_CLK, 0);

	/* Configure data pins to input IO pins */
	gpio_direction_input(WLAN_DATA0);
	gpio_direction_input(WLAN_DATA1);
	gpio_direction_input(WLAN_DATA2);

	for (count = 0; count < 2; count++) {
		gpio_set_value(WLAN_CLK, 1);
		gpio_set_value(WLAN_CLK, 0);
	}

	rf_bit = 0;
	for (count = 0; count < 6; count++) {
		gpio_set_value(WLAN_CLK, 1);
		gpio_set_value(WLAN_CLK, 0);

		rf_bit = gpio_get_value(WLAN_DATA0);
		rf_data_received |= (rf_bit << (count * 3 + 0));

		if (count != 5) {
			rf_bit = gpio_get_value(WLAN_DATA1);
			rf_data_received |= (rf_bit << (count * 3 + 1));

			rf_bit = gpio_get_value(WLAN_DATA2);
			rf_data_received |= (rf_bit << (count * 3 + 2));
		}
	}

	gpio_set_value(WLAN_SET, 0);
	wcnss_switch_to_5wire();

	return rf_data_received;
}
