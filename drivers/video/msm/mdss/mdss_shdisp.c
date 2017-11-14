/* drivers/video/msm/mdss/mdss_shdisp.c  (Display Driver)
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <mach/board.h>
#include <sharp/shdisp_kerl.h>
#include "mdss_fb.h"

static int lcd_disp_on = 0;

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_disp_status(void)
{
	shdisp_api_get_boot_context();

	if( shdisp_api_get_boot_disp_status() ) {
		return true;
	}

	return false;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_on( void )
{
	shdisp_api_lcd_on();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_off( void )
{
    int mode;
    mode = mdss_fb_shutdown_in_progress();
    shdisp_api_lcd_off(mode);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bkl_ctl( u32 bl_level )
{
	struct shdisp_main_bkl_ctl bkl;

	pr_debug("LCDDBG: %s called bl_level=%u\n", __func__, bl_level);

	if( bl_level == 0 ) {
		shdisp_api_main_bkl_off();
	} else {
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
int mdss_shdisp_is_disp_on( void )
{
	return lcd_disp_on;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_start_display( void )
{
	shdisp_api_start_display();
	lcd_disp_on = 1;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lcd_power_on( void )
{
//	shdisp_api_main_lcd_power_on();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lcd_disp_off( void )
{
//	shdisp_api_main_disp_off();
	lcd_disp_on = 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_pll_ctl(int ctl)
{
	return shdisp_api_main_pll_ctl(ctl);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_shutdown( void )
{
	shdisp_api_shutdown();
}

