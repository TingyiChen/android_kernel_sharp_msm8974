/* drivers/sharp/shdisp/shdisp_bl71y6.c  (Display Driver)
 *
 * Copyright (C) 2013-2014 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_bdic.h"
#include "shdisp_dbg.h"

#if defined(CONFIG_MACH_LYNX_GP9D)
  #include "./data/shdisp_bl71y6_data_gp9d.h"
#elif defined(CONFIG_MACH_LYNX_DL50)
  #include "./data/shdisp_bl71y6_data_dl50.h"
#elif defined(CONFIG_MACH_DECKARD_AS99)
  #include "./data/shdisp_bl71y6_data_as99.h"
#elif defined(CONFIG_MACH_EBZ)
  #include "./data/shdisp_bl71y6_data_pa24.h"
#else
  #include "./data/shdisp_bl71y6_data_default.h"
#endif

#include "shdisp_pm.h"
#include <sharp/sh_boot_manager.h>

#define SHDISP_BL71Y6_CTRL_DECLARE
#include "data/shdisp_bl71y6_ctrl.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_BKL_MODE_OFF                (0)
#define SHDISP_BDIC_BKL_MODE_FIX                (1)
#define SHDISP_BDIC_BKL_MODE_AUTO               (2)

#define SHDISP_BDIC_BKL_DTV_OFF                 (0)
#define SHDISP_BDIC_BKL_DTV_ON                  (1)

#define SHDISP_BDIC_BKL_EMG_OFF                 (0)
#define SHDISP_BDIC_BKL_EMG_ON                  (1)

#define SHDISP_BDIC_BKL_ECO_OFF                 (0)
#define SHDISP_BDIC_BKL_ECO_ON                  (1)

#define SHDISP_BDIC_BKL_CHG_OFF                 (0)
#define SHDISP_BDIC_BKL_CHG_ON                  (1)

#define SHDISP_BDIC_TRI_LED_MODE_OFF           (-1)
#define SHDISP_BDIC_TRI_LED_MODE_NORMAL         (0)
#define SHDISP_BDIC_TRI_LED_MODE_BLINK          (1)
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY        (2)
#define SHDISP_BDIC_TRI_LED_MODE_HISPEED        (3)
#define SHDISP_BDIC_TRI_LED_MODE_STANDARD       (4)
#define SHDISP_BDIC_TRI_LED_MODE_BREATH         (5)
#define SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH    (6)
#define SHDISP_BDIC_TRI_LED_MODE_WAVE           (7)
#define SHDISP_BDIC_TRI_LED_MODE_FLASH          (8)
#define SHDISP_BDIC_TRI_LED_MODE_AURORA         (9)
#define SHDISP_BDIC_TRI_LED_MODE_RAINBOW       (10)

#define SHDISP_BDIC_I2C_ADO_UPDATE_MASK         (0x60)
#define SHDISP_BDIC_I2C_SEND_WAIT               (1000)
#define SHDISP_BDIC_I2C_SEND_RETRY              (200)


#define SHDISP_BDIC_I2C_SEND_RETRY_WAIT         (100)

#define SHDISP_FW_STACK_EXCUTE

#define SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE    (ARRAY_SIZE(shdisp_bdic_bkl_ado_tbl))
#define SHDISP_BDIC_LUX_DIVIDE_COFF         (100)

#define SHDISP_BDIC_REGSET(x)         (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x), 0, 0))
#define SHDISP_PSALS_REGSET(x)        (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x), bkl_mode, lux_mode))

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_get_lux_data(void);
static void shdisp_bdic_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk);
static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_seq_backlight_off(void);
static void shdisp_bdic_seq_backlight_fix_on(int param);
static void shdisp_bdic_seq_backlight_auto_on(int param);
static int  shdisp_bdic_seq_led_off(void);
static int  shdisp_bdic_seq_led_normal_on(unsigned char color);
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_als(unsigned short *clear, unsigned short *ir);
static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_seq_psals_active(unsigned long dev_type);
static int shdisp_bdic_seq_psals_standby(unsigned long dev_type);
static int shdisp_bdic_seq_ps_background(unsigned long state);
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void);
static void shdisp_bdic_LD_LCD_BKL_emg_on(void);
static void shdisp_bdic_LD_LCD_BKL_emg_off(void);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);
static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned short *param);
static void shdisp_bdic_LD_LCD_BKL_get_auto_param(int mode, unsigned short *param);
static void shdisp_bdic_LD_LCD_BKL_get_auto_fix_param(int mode, int level, unsigned short *param);
static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned short *pwm_val);
static void shdisp_bdic_LD_LCD_BKL_eco_on(void);
static void shdisp_bdic_LD_LCD_BKL_eco_off(void);
static void shdisp_bdic_LD_LCD_BKL_chg_on(void);
static void shdisp_bdic_LD_LCD_BKL_chg_off(void);

static void shdisp_bdic_PD_LCD_POS_PWR_on(void);
static void shdisp_bdic_PD_LCD_POS_PWR_off(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_on(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_off(void);
static void shdisp_bdic_PD_LCD_VO2_ON(void);
static void shdisp_bdic_PD_LCD_VO2_OFF(void);
static void shdisp_bdic_seq_bdic_active_for_led(int);
static void shdisp_bdic_seq_bdic_standby_for_led(int);
static void shdisp_bdic_seq_clmr_off_for_led(int);
static int  shdisp_bdic_seq_regset(
            const shdisp_bdicRegSetting_t *regtable, int size, unsigned char bkl_mode, unsigned char lux_mode);
static int  shdisp_bdic_PD_set_active(int power_status);
static int  shdisp_bdic_PD_set_standby(void);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
static void shdisp_bdic_PD_TRI_LED_set_anime(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig(void);
static void shdisp_bdic_PWM_set_value(int pwm_param);
static void shdisp_bdic_bkl_adj_clmr_write_cmd(int mode, unsigned char *value, int len );
static int  shdisp_bdic_PD_psals_power_on(void);
static int  shdisp_bdic_PD_psals_power_off(void);
static int  shdisp_bdic_PD_psals_ps_init_als_off(void);
static int  shdisp_bdic_PD_psals_ps_init_als_on(void);
static int  shdisp_bdic_PD_psals_ps_deinit_als_off(void);
static int  shdisp_bdic_PD_psals_ps_deinit_als_on(void);
static int  shdisp_bdic_PD_psals_als_init_ps_off(void);
static int  shdisp_bdic_PD_psals_als_init_ps_on(void);
static int  shdisp_bdic_PD_psals_als_deinit_ps_off(void);
static int  shdisp_bdic_PD_psals_als_deinit_ps_on(void);

static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value);
static void shdisp_bdic_PD_clmr_get_ado_cmd(unsigned short *ado_value);

static int  shdisp_bdic_PD_wait4i2ctimer_stop(void);

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_bdic_IO_bank_set(unsigned char val);
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_bdic_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val);
#endif /* CONFIG_ANDROID_ENGINEERING */
static int shdisp_bdic_als_clmr_write_cmd(unsigned char reg, unsigned char val);
static int shdisp_bdic_als_clmr_msk_write_cmd(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_bdic_als_clmr_read_cmd(unsigned char reg, unsigned char *val);
static int shdisp_bdic_als_clmr_burst_write_cmd(unsigned char addr, unsigned char *val, unsigned char size);

static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params);
static int shdisp_bdic_register_driver(void);
static int shdisp_bdic_PD_get_auto_bkl_level(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_bdic_state_str s_state_str;

static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_bkl_param;
static int shdisp_bdic_bkl_param_auto;
static int shdisp_bdic_bkl_before_mode;

static int shdisp_bdic_dtv;

static int shdisp_bdic_emg;

static int shdisp_bdic_eco;

static int shdisp_bdic_chg;

static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_before_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;
static int shdisp_bdic_tri_led_count;

static int shdisp_bdic_i2c_errflag;

static int const mp_lux_th    = 3;

static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_OFF },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_OFF }
};

static unsigned int shdisp_bdic_irq_fac = 0;
static unsigned int shdisp_bdic_irq_fac_exe = 0;

static int  shdisp_bdic_irq_prioriy[SHDISP_IRQ_MAX_KIND];

static unsigned char shdisp_backup_irq_photo_req[3];

static unsigned char lux_mode_recovery;
static unsigned char bkl_mode_recovery;
static int psals_recovery_flag = 0;

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_boot_init( void )
{
#ifdef SHDISP_NOT_SUPPORT_NO_OS
    int ret;
    unsigned char version;
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

    SHDISP_TRACE("in\n")
    shdisp_bdic_bkl_mode        = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_param       = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_bdic_bkl_param_auto  = SHDISP_MAIN_BKL_PARAM_OFF;

    shdisp_bdic_dtv             = SHDISP_BDIC_BKL_DTV_OFF;

    shdisp_bdic_emg             = SHDISP_BDIC_BKL_EMG_OFF;

    shdisp_bdic_eco             = SHDISP_BDIC_BKL_ECO_OFF;

    shdisp_bdic_chg             = SHDISP_BDIC_BKL_CHG_OFF;

    shdisp_bdic_tri_led_color    = 0;
    shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_ontime   = 0;
    shdisp_bdic_tri_led_interval = 0;
    shdisp_bdic_tri_led_count    = 0;

    shdisp_bdic_register_driver();

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    s_state_str.bdic_chipver = 0x00;
    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
    shdisp_bdic_api_hw_reset();

    ret = shdisp_bdic_IO_read_check_reg(BDIC_REG_VERSION, &version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_api_set_hw_reset();
        return SHDISP_BDIC_IS_NOT_EXIST;
    }

    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
    if (ret != SHDISP_RESULT_SUCCESS || (version & 0x08) != 0x08) {
        SHDISP_ERR("Version Read. ret=%d Version=%02X\n", ret, version);
        shdisp_bdic_api_set_hw_reset();
        return SHDISP_BDIC_IS_NOT_EXIST;
    }
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

    SHDISP_TRACE("out\n")
    return SHDISP_BDIC_IS_EXIST;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_hw_reset                                                  */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_api_hw_reset(void)
{
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_hw_reset                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_api_set_hw_reset(void)
{
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_bdic_exist                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_bdic_exist(int *bdic_is_exist)
{
    *bdic_is_exist = s_state_str.bdic_is_exist;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_bdic_chipver                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_get_bdic_chipver(int *chipver)
{
    *chipver = s_state_str.bdic_chipver;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str *state_str)
{
    s_state_str.bdic_is_exist                   = state_str->bdic_is_exist;
    s_state_str.bdic_main_bkl_opt_mode_output   = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.bdic_main_bkl_opt_mode_ado      = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.shdisp_lux_change_level1        = SHDISP_LUX_CHANGE_LEVEL1;
    s_state_str.shdisp_lux_change_level2        = SHDISP_LUX_CHANGE_LEVEL2;
    s_state_str.clmr_is_exist                   = state_str->clmr_is_exist;

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    if (s_state_str.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
        (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
    }
#else
    s_state_str.bdic_chipver = state_str->bdic_chipver;
#endif
    s_state_str.bdic_clrvari_index              = state_str->bdic_clrvari_index;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_on                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_on(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_on();
    SHDISP_TRACE("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_off                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_off();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_on(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_on();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_off();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_on                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_on(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_VO2_ON();
    SHDISP_TRACE("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_VO2_OFF();
    SHDISP_TRACE("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_off();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_active                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_active(int power_status)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret =  shdisp_bdic_PD_set_active(power_status);
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_standby                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_set_standby();
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_fix_on(param);
    SHDISP_TRACE("out\n")
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_auto_on                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_auto_on(int param)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_auto_on(param);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int *param)
{
    int mode = 0;
    unsigned short pwm_table[SHDISP_BKL_AUTO_PWM_TBL_NUM];

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, pwm_table);
        *param = pwm_table[1];
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        *param = 0x100;
        break;

    default:
        *param = 0;
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;

    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode        = tmp->led_mode;
    shdisp_bdic_tri_led_before_mode = tmp->led_mode;
    shdisp_bdic_tri_led_color       = color;
    shdisp_bdic_tri_led_ontime      = tmp->ontime;
    shdisp_bdic_tri_led_interval    = tmp->interval;
    shdisp_bdic_tri_led_count       = tmp->count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;


    SHDISP_DEBUG("tmp->mode %d, tmp->param %d \n", tmp->mode, tmp->param)

    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if ((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
               (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_WEAK)) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param != SHDISP_MAIN_BKL_PARAM_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param;
    } else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_on(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_off(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_on(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_off(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_off(void)
{
    int ret;
    ret = shdisp_bdic_seq_led_off();
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */

unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led )
{
    int i;
    unsigned char color = 0xFF;

    for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
        if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
            shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
            shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue) {
            color = shdisp_triple_led_color_index_tbl[i].color;
            break;
        }
    }

    if (color == 0xFF) {
        if (tri_led->red > 1) {
            tri_led->red = 1;
        }
        if (tri_led->green > 1) {
            tri_led->green = 1;
        }
        if (tri_led->blue > 1) {
            tri_led->blue = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
            if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue) {
                color = shdisp_triple_led_color_index_tbl[i].color;
                break;
            }
        }
        if (color == 0xFF) {
            color = 0;
        }
    }
    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    int ret;
    ret = shdisp_bdic_seq_led_normal_on(color);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_blink_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_firefly_on                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_firefly_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_clrvari_index                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari )
{
    int i = 0;

    for (i = 0; i < SHDISP_COL_VARI_KIND; i++) {
        if ((int)shdisp_clrvari_index[i] == clrvari) {
            break;
        }
    }
    if (i >= SHDISP_COL_VARI_KIND) {
        i = 0;
    }
    return i;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_lux                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(value, lux);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind                               */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode)
{
    if (s_state_str.bdic_main_bkl_opt_mode_ado == SHDISP_BDIC_MAIN_BKL_OPT_LOW) {
        *mode = SHDISP_LUX_MODE_LOW;
    } else {
        *mode = SHDISP_LUX_MODE_HIGH;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_als                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_als(unsigned short *clear, unsigned short *ir)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_als(clear, ir);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char value = 0x00;
    int i2c_rtimer_restart_flg = 0;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_read_reg(BDIC_REG_SYSTEM8, &value);

    if (value & SHDISP_BDIC_I2C_R_START) {
        SHDISP_ERR("<OTHER> i2c read start.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (value & SHDISP_BDIC_I2C_R_TIMRE_START) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : stop i2c rtimer.\n");
#endif
        i2c_rtimer_restart_flg = 1;
        shdisp_bdic_API_I2C_start_ctl(SHDISP_BDIC_POLLING_OFF);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_SYS_delay_us(1000);
    }

    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
        ret = shdisp_bdic_LD_i2c_write(msg);
        break;
    case SHDISP_BDIC_I2C_M_R:
        ret = shdisp_bdic_LD_i2c_read_mode0(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
        ret = shdisp_bdic_LD_i2c_read_mode1(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE2:
        ret = shdisp_bdic_LD_i2c_read_mode2(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = shdisp_bdic_LD_i2c_read_mode3(msg);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    if (i2c_rtimer_restart_flg == 1) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : restart i2c rtimer.\n");
#endif
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        } else {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x30);
        }
        shdisp_bdic_API_I2C_start_ctl(SHDISP_BDIC_POLLING_ON);
    } else {
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_I2C_start_judge                                           */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_bdic_API_I2C_start_judge(void)
{
    unsigned char value = 0x00;

    shdisp_bdic_IO_read_reg(BDIC_REG_SYSTEM8, &value);
    value &= SHDISP_BDIC_I2C_R_TIMER_MASK;

    return value;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_I2C_start_ctl                                             */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_I2C_start_ctl(int flg)
{
    if (flg == SHDISP_BDIC_POLLING_ON) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM8, SHDISP_BDIC_I2C_THROUGH_MODE);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM8,
                SHDISP_BDIC_I2C_R_TIMRE_START, SHDISP_BDIC_I2C_R_TIMER_MASK);
    } else {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM8,
                SHDISP_BDIC_I2C_R_TIMRE_STOP, SHDISP_BDIC_I2C_R_TIMER_MASK);
        shdisp_bdic_PD_wait4i2ctimer_stop();
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM8, SHDISP_BDIC_I2C_THROUGH_MODE);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_ALS_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_ALS_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }

    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
          ret = shdisp_bdic_als_clmr_write_cmd( msg->wbuf[0] , msg->wbuf[1] );
        break;
    case SHDISP_BDIC_I2C_M_R:
          ret = shdisp_bdic_als_clmr_read_cmd( msg->wbuf[0] , msg->rbuf );
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
        ret = shdisp_bdic_LD_i2c_read_mode1(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE2:
        ret = shdisp_bdic_LD_i2c_read_mode2(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = shdisp_bdic_LD_i2c_read_mode3(msg);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    ret = shdisp_bdic_IO_write_reg(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    } else {
        shdisp_SYS_bdic_i2c_cmd_reset();
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    ret = shdisp_bdic_IO_read_reg(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    } else {
        shdisp_SYS_bdic_i2c_cmd_reset();
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    unsigned char dummy = 0;

    shdisp_bdic_IO_read_reg(BDIC_REG_GINF4, &dummy);

    if (dummy & 0x04) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        return SHDISP_RESULT_FAILURE;
    }
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned char   shdisp_log_lv_bk;

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc(256 * 2, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", 256 * 2);
        return;
    }
    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    p = pbuf;
    shdisp_bdic_IO_bank_set(0x00);
    for (idx = 0x00; idx <= 0xFF; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_IO_bank_set(0x01);
    for (idx = 0x00; idx <= 0xFF; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_IO_bank_set(0x00);
    shdisp_log_lv = shdisp_log_lv_bk;
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_OFF);

    printk("[SHDISP] BDIC INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist              = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] s_state_str.bdic_chipver               = %d.\n", s_state_str.bdic_chipver);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_output = %d. (0:low/1:high)\n",
                                                                    s_state_str.bdic_main_bkl_opt_mode_output);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_ado = %d. (0:low/1:high)\n",
                                                                    s_state_str.bdic_main_bkl_opt_mode_ado);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level1   = %d.\n", s_state_str.shdisp_lux_change_level1);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level2   = %d.\n", s_state_str.shdisp_lux_change_level2);
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_dtv                        = %d.\n", shdisp_bdic_dtv);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_eco                        = %d.\n", shdisp_bdic_eco);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);
    printk("[SHDISP] shdisp_bdic_tri_led_color              = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode               = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_before_mode        = %d.\n", shdisp_bdic_tri_led_before_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime             = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval           = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count              = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] bdic_clrvari_index                     = %d.\n", s_state_str.bdic_clrvari_index );

    for (idx = 0; idx < NUM_SHDISP_MAIN_BKL_DEV_TYPE; idx++) {
        printk("[SHDISP] shdisp_bkl_priority_table[%d]       = (mode:%d, param:%d).\n",
                                    idx, shdisp_bkl_priority_table[idx].mode, shdisp_bkl_priority_table[idx].param);
    }

    p = pbuf;
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK0 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
        p += 8;
    }
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK1 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
        p += 8;
    }
    printk("[SHDISP] BDIC INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_INFO_output                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    pbuf = (unsigned char*)kzalloc((BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1));
        return;
    }
    p = pbuf;
    for (idx = BDIC_REG_SEQ_ANIME; idx <= BDIC_REG_CH2_C; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] TRI-LED INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist      = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] shdisp_bdic_tri_led_color      = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode       = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_before_mode= %d.\n", shdisp_bdic_tri_led_before_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime     = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval   = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count      = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] shdisp_bdic_clrvari_index      = %d.\n", s_state_str.bdic_clrvari_index);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x\n", BDIC_REG_SEQ_ANIME, *p, *(p + 1), *(p + 2));
    p += 3;
    printk("[SHDISP] BDIC_REG_LED_SETTING   0x%2X: %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH0_SET1, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED_CURRENT   0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH0_A, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p  + 7), *(p + 8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED INFO <<-\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PSALS_INFO_output                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_PSALS_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc((((SENSOR_REG_D2_MSB + 7) / 8) * 8), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (((SENSOR_REG_D2_MSB + 7) / 8) * 8));
        return;
    }

    shdisp_SYS_delay_us(1000 * 1000);

    p = pbuf;
    for (idx = SENSOR_REG_COMMAND1; idx <= SENSOR_REG_D2_MSB; idx++) {
        *p = 0x00;
        shdisp_bdic_photo_sensor_IO_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] PSALS SENSOR INFO ->>\n");
    p = pbuf;
    printk("[SHDISP] SENSOR_REG_DUMP 0x00: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                        *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x08: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                        *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x10: %02x %02x                              \n", *p, *(p + 1));
    printk("[SHDISP] PSALS SENSOR INFO <<-\n");

    kfree(pbuf);
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DIAG, SHDISP_DEV_STATE_OFF);
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_type( int irq_type )
{
    if ((irq_type < SHDISP_IRQ_TYPE_PALS) || (irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    if (irq_type == SHDISP_IRQ_TYPE_DET) {
        if (!(SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) {
           return SHDISP_RESULT_FAILURE;
        }
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_save_fac                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_save_fac(void)
{
    unsigned char value1 = 0, value2 = 0, value3 = 0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC1, &value1);
    value1 &= 0x7F;
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC3, &value2);
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC4, &value3);
    SHDISP_DEBUG("GFAC4=%02x GFAC3=%02x GFAC1=%02x\n", value3, value2, value1);
    shdisp_bdic_irq_fac = (unsigned int)value1 | ((unsigned int)value2 << 8 ) | ((unsigned int)value3 << 16);

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        SHDISP_ERR("[SHDISP] ps_als error : INT_PS_REQ(GFAC3[1]) detect\n" );
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PS_REQ;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        SHDISP_ERR("[SHDISP] ps_als error : INT_I2C_ERR_REQ(GFAC4[3]) detect\n" );
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_I2C_ERROR_BDIC;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
    }

    if (shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_DET | SHDISP_BDIC_INT_GFAC_PS2 | SHDISP_BDIC_INT_GFAC_I2C_ERR)) {
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMR3, &shdisp_backup_irq_photo_req[0]);
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMF3, &shdisp_backup_irq_photo_req[1]);
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMR4, &shdisp_backup_irq_photo_req[2]);
    }

    if ((shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_DET(void)
{
    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        return SHDISP_BDIC_IRQ_TYPE_DET;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_fac(void)
{
    int i;
    if (shdisp_bdic_irq_fac == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_irq_fac_exe = (shdisp_bdic_irq_fac & SHDISP_INT_ENABLE_GFAC);
    if (shdisp_bdic_irq_fac_exe == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_NONE;
    }

    i = 0;
    if ((shdisp_bdic_irq_fac_exe &
         (SHDISP_BDIC_INT_GFAC_PS | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_PS2))
         == SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_PS;
        i++;
    }

    if ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_DET) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_DET;
        i++;
    } else if (((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_I2C_ERR) != 0) ||
               ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_PS2) != 0)) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
        i++;
    } else if ((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS;
        i++;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_get_fac( int iQueFac )
{
    if (iQueFac >= SHDISP_IRQ_MAX_KIND) {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
    return shdisp_bdic_irq_prioriy[iQueFac];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_Clear                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_Clear(void)
{
    unsigned char out1, out2, out3;

    if (shdisp_bdic_irq_fac == 0) {
        return;
    }

    out1 = (unsigned char)(shdisp_bdic_irq_fac & 0x000000FF);
    out2 = (unsigned char)((shdisp_bdic_irq_fac >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((shdisp_bdic_irq_fac >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);

    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) && (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS)) {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR))) {
        if (shdisp_backup_irq_photo_req[0] & 0x01) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[1] & 0x01) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[2] & 0x20) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR4, 0x20);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_i2c_error_Clear(void)
{
    unsigned char out2 = 0, out3 = 0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        out3 = 0x08;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
        out2 = 0x02;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }
    psals_recovery_flag = 1;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_fac_Clear                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_fac_Clear(void)
{
    unsigned char out3 = 0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET) {
        out3 = 0x04;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_ctrl                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl)
{
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_LCD);
    shdisp_bdic_IO_bank_set(0x00);
    if (ctrl) {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF4, 0x04);
    } else {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }
    shdisp_FWCMD_safe_finishanddoKick();
    shdisp_FWCMD_set_apino(SHDISP_CLMR_FWCMD_APINO_NOTHING);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_Clear_All                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_Clear_All(void)
{
    unsigned char out1, out2, out3;

    out1 = (unsigned char)(SHDISP_INT_ENABLE_GFAC & 0x000000FF);
    out2 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_set_fac                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC)
{
    shdisp_bdic_irq_fac = nGFAC;

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_OPTSEL) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_photo_param                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2)
{
    s_state_str.shdisp_lux_change_level1 = (unsigned char)( level1 & 0x00FF );
    s_state_str.shdisp_lux_change_level2 = (unsigned char)( level2 & 0x00FF );
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_als_sensor_pow_ctl                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_api_als_sensor_pow_ctl(int dev_type, int power_mode)
{
    unsigned long type = 0;
    int param_chk = 0;

    switch (dev_type) {
    case SHDISP_PHOTO_SENSOR_TYPE_APP:
        type = SHDISP_DEV_TYPE_ALS_APP;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_CAMERA:
        type = SHDISP_DEV_TYPE_ALS_CAMERA;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_KEYLED:
        type = SHDISP_DEV_TYPE_ALS_KEYLED;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_DIAG:
        type = SHDISP_DEV_TYPE_ALS_DIAG;
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->type(%d).\n", dev_type);
        break;
    }

    switch (power_mode) {
    case SHDISP_PHOTO_SENSOR_DISABLE:
        shdisp_bdic_seq_psals_standby(type);
        break;
    case SHDISP_PHOTO_SENSOR_ENABLE:
        shdisp_bdic_seq_psals_active(type);
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->power(%d).\n", power_mode);
        break;
    }

    if (param_chk == 1) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_default_sensor_param                                  */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_api_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj)
{
    unsigned long tmp_param1;
    unsigned long tmp_param2;

    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ0_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ0_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ1_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ1_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[0].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_A;
    tmp_adj->als_adjust[0].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_A;
    tmp_adj->als_adjust[0].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_A;

    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ0_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ0_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ1_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ1_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[1].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_B;
    tmp_adj->als_adjust[1].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_B;
    tmp_adj->als_adjust[1].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_B;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_prox_sensor_param                                     */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_api_set_prox_sensor_param(struct shdisp_prox_params *prox_params)
{
    shdisp_bdic_PD_psals_write_threshold(prox_params);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_get_LC_MLED01                                             */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_bdic_api_get_LC_MLED01(void)
{
    return ((BDIC_REG_M1LED_VAL << 8) | BDIC_REG_M2LED_VAL);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_get_lux_data                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_get_lux_data(void)
{
    unsigned char para[1] = { 0x05 };
    unsigned char rtnbuf[4] = {0};

    SHDISP_TRACE("in\n");

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_safe_finishanddoKick();
    }

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXPARAM_READ, 1, para);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 4, rtnbuf);

    SHDISP_TRACE("out\n");
    return (rtnbuf[3] << 24) | (rtnbuf[2] << 16) | (rtnbuf[1] << 8) | rtnbuf[0];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_bkl_mode                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk)
{
    unsigned char value[1];

    SHDISP_TRACE("in\n");

    value[0] = bkl_mode;
    value[0] = value[0] & ~msk;
    value[0] |= data;

    SHDISP_DEBUG("bkl_mode=0x%02x\n", value[0]);

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, 1, value);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, 1, value);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }

    bkl_mode_recovery = value[0];

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_lux_mode                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk)
{
    unsigned char value[1];

    SHDISP_TRACE("in\n");

    value[0] = lux_mode;
    value[0] = value[0] & ~msk;
    value[0] |= data;

    SHDISP_DEBUG("lux_mode=0x%02x\n", value[0]);

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, value);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, value);
        shdisp_FWCMD_buf_finish();
        shdisp_FWCMD_doKick(1, 0, 0);
    }

    lux_mode_recovery = value[0];

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_lux_mode_modify                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk)
{
    unsigned char lux_mode[1];
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    lux_mode[0] = (( lux_data  & 0x0000ff00 ) >> 8);
    SHDISP_DEBUG("before lux_mode=0x%02x\n", lux_mode[0]);

    lux_mode[0] = lux_mode[0] & ~msk;
    lux_mode[0] |= data;

    SHDISP_DEBUG("after lux_mode=0x%02x\n", lux_mode[0]);

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, 1, lux_mode);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 0, 0);

    lux_mode_recovery = lux_mode[0];

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_lux_data_backup                                  */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_RECOVERY_lux_data_backup(void)
{
    int lux_data;
    unsigned char bkl_mode;
    unsigned char lux_mode;

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = (( lux_data  & 0x0000ff00 ) >> 8);

    shdisp_bdic_set_bkl_mode(0x00 , 0x00 , 0xff);
    shdisp_bdic_API_set_lux_mode(0x00 , 0x00 , 0xff);

    bkl_mode_recovery = bkl_mode;
    lux_mode_recovery = lux_mode;

    SHDISP_DEBUG("backup bkl_mode_recovery = 0x%02x\n", bkl_mode_recovery);
    SHDISP_DEBUG("backup lux_mode_recovery = 0x%02x\n", lux_mode_recovery);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_lux_data_restore                                 */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_RECOVERY_lux_data_restore(void)
{
    SHDISP_DEBUG("restore bkl_mode_recovery = 0x%02x\n", bkl_mode_recovery);
    SHDISP_DEBUG("restore lux_mode_recovery = 0x%02x\n", lux_mode_recovery);
    shdisp_bdic_set_bkl_mode(0x00, bkl_mode_recovery , 0xff);
    shdisp_bdic_API_set_lux_mode(0x00, lux_mode_recovery , 0xff);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_on                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_off                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_active                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_psals_active(unsigned long dev_type)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_seq_psals_active(dev_type);
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_standby                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_psals_standby(unsigned long dev_type)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_seq_psals_standby(dev_type);
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_ps_background                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_ps_background(unsigned long state)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_seq_ps_background(state);
    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_init_als_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_init_als_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_init_ps_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_init_ps_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD4;
        break;

    default:
        return;;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_off                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_off(void)
{
    int ret;

    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_fix_on                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_fix_on(int param)
{
    int ret;

    SHDISP_TRACE("in param:%d\n", param);
    (void)shdisp_pm_clmr_power_manager((SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager((SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 START\n");
    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 END\n");

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_POST_START_FIX, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    SHDISP_TRACE("out Completed.\n");
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_auto_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_auto_on(int param)
{
    int ret;

    (void)shdisp_pm_clmr_power_manager((SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager((SHDISP_DEV_TYPE_BKL | SHDISP_DEV_TYPE_ALS_BKL), SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 START\n");
    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_PRE_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 END\n");

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_POST_START, 0);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_off                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_off(void)
{
    int ret;

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_STOP, 0);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_normal_on                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_normal_on(unsigned char color)
{
    int ret;

    SHDISP_TRACE("in color:%d\n", color);
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED);
    SHDISP_TRACE("out ret:(%d)\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_blink_on                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d\n", color);
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    } else {
        shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED);
    }
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_firefly_on                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d\n", color);
    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);
    shdisp_SYS_bdic_i2c_doKick_if_exist();

    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    } else {
        shdisp_bdic_seq_clmr_off_for_led(SHDISP_DEV_TYPE_LED);
    }
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lux                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int i;
    int ret;
    unsigned long ret_lux;
    unsigned long ado;

    SHDISP_TRACE("in\n");

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_PD_REG_ADO_get_opt(value);

    ado = (unsigned long)(*value);
    ret_lux = 0;
    if (ado != 0) {
        for (i = 0; i < SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE; i++) {
            if ((ado >= shdisp_bdic_bkl_ado_tbl[i].range_low) &&
                (ado < shdisp_bdic_bkl_ado_tbl[i].range_high)) {
                ret_lux  = (unsigned long)((unsigned short)ado * (unsigned short)shdisp_bdic_bkl_ado_tbl[i].param_a);
                ret_lux += shdisp_bdic_bkl_ado_tbl[i].param_b;
                ret_lux += (SHDISP_BDIC_LUX_DIVIDE_COFF / 2);
                ret_lux /= SHDISP_BDIC_LUX_DIVIDE_COFF;
                break;
            }
        }
    }

   *lux = ret_lux;

    SHDISP_TRACE("out ado=0x%04X, lux=%lu\n", (unsigned int)ado, ret_lux);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_als                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_als(unsigned short *clear, unsigned short *ir)
{
    int ret;
    int InfoReg1;

    SHDISP_TRACE("in\n");

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }

    InfoReg1 = shdisp_SYS_getInfoReg1();

    *clear = (unsigned short)(InfoReg1 & 0x0000ffff);
    *ir = (unsigned short)((InfoReg1 >> 16) & 0x0000ffff);

    SHDISP_TRACE("out clear=0x%04x, ir=0x%04x\n", *clear, *ir);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_write                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i;

#ifdef SHDISP_SW_BDIC_I2C_RWLOG
    int t;
    if (msg->wlen <= 0) {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d)\n", msg->addr, msg->wlen);
    } else {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d Wbuf=", msg->addr, msg->wlen);
        for (t = 0; t < (msg->wlen - 1); t++) {
            printk("0x%02X,", (msg->wbuf[t] & 0xFF));
        }
        printk("0x%02X)\n", (msg->wbuf[t] & 0xFF));
    }
#endif

    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->wlen > SHDISP_BDIC_I2C_WBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));

    for (i = 0; i < msg->wlen; i++) {
        shdisp_bdic_IO_write_reg((BDIC_REG_I2C_DATA1 + i), msg->wbuf[i]);
    }

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, msg->wlen);

    shdisp_bdic_i2c_errflag = 0;

    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM8, SHDISP_BDIC_I2C_W_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);

    for (i = 0; i <= SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else {
            shdisp_bdic_IO_read_reg(BDIC_REG_SYSTEM8, &value);
            if ((value & SHDISP_BDIC_I2C_W_START) == 0x00) {
                return SHDISP_RESULT_SUCCESS;
            } else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }

    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode0                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i, t;

    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->wlen != 0x01) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->rbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->rlen > SHDISP_BDIC_I2C_RBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->rlen(%d).\n", msg->rlen);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SYS, 0x00);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, msg->wbuf[0]);

    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, (unsigned char)(msg->rlen << 4));

    shdisp_bdic_i2c_errflag = 0;

    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM8, SHDISP_BDIC_I2C_R_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);

    for (i = 0; i <= SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else {
            shdisp_bdic_IO_read_reg(BDIC_REG_SYSTEM8, &value);
            if ((value & SHDISP_BDIC_I2C_R_START) == 0x00) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP] bdic i2c Read(addr=0x%02x, reg=0x%02X, size=%d, ",
                                                                                msg->addr, msg->wbuf[0], msg->rlen);
#endif
                for (t = 0; t < msg->rlen; t++) {
                    shdisp_bdic_IO_read_reg((BDIC_REG_I2C_RDATA0 + t), &value);
                    msg->rbuf[t] = value;
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                    if (t < (msg->rlen - 1)) {
                        printk("0x%02X,", value);
                    } else {
                        printk("0x%02X)\n", value);
                    }
#endif
                }
                return SHDISP_RESULT_SUCCESS;
            } else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }

    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode1                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode2                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode3                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_active                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_psals_active(unsigned long dev_type)
{
    if (shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_ON) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_ON) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_standby                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_psals_standby(unsigned long dev_type)
{
    if (shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_OFF) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF) !=  SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF) !=  SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_ps_background                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_ps_background(unsigned long state)
{
    int ret;
    ret = shdisp_pm_clmr_power_manager(SHDISP_DEV_TYPE_PS, state);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_on(void)
{
    int ret;

    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_off(void)
{
    int ret;

    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_on(void)
{
    int ret;

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_off(void)
{
    int ret;

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY;
        SHDISP_INFO("Current brightness mode: EMERGENCY\n");
    } else if ((shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_FIX) && (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON)) {
        *mode = SHDISP_BKL_TBL_MODE_CHARGE;
        SHDISP_INFO("Current brightness mode: CHARGE\n");
    } else if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        *mode = SHDISP_BKL_TBL_MODE_ECO;
        SHDISP_INFO("Current brightness mode: ECO\n");
    } else {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
        SHDISP_INFO("Current brightness mode: NORMAL\n");
    }

    if (*mode >= NUM_SHDISP_BKL_TBL_MODE) {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_fix_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned short *param)
{
    unsigned short pwm_val;

    SHDISP_TRACE("in\n");

    if (param == NULL) {
        return;
    }

    *param = 0x5D00;

    pwm_val = shdisp_main_bkl_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_ECO:
        break;
    case SHDISP_BKL_TBL_MODE_EMERGENCY:
        if (pwm_val > SHDISP_BKL_EMERGENCY_LIMIT_FIX) {
            pwm_val = SHDISP_BKL_EMERGENCY_LIMIT_FIX;
        }
        break;
    case SHDISP_BKL_TBL_MODE_CHARGE:
    default:
        break;
    }

    *(param + 1) = pwm_val;

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_auto_param                                     */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_auto_param(int mode, unsigned short *param)
{
    unsigned short pwm_val;
    int i;

    SHDISP_TRACE("in\n");

    if (param == NULL) {
        return;
    }

    *param = shdisp_main_bkl_min_tbl[0];

    for (i = 1; i < SHDISP_BKL_AUTO_PWM_TBL_NUM; i++) {
        shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &pwm_val);
        *(param + i) = pwm_val;
    }

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_auto_fix_param                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_auto_fix_param(int mode, int level, unsigned short *param)
{
    unsigned short pwm_val;

    SHDISP_TRACE("in\n");

    if (param == NULL) {
        return;
    }

    shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, level, &pwm_val);

    *param = 0x5D00;
    *(param + 1) = pwm_val;

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_pwm_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned short *pwm_val)
{
    unsigned long val;
    unsigned short min_val;
    unsigned short max_val;

    if (level < 1) {
        *pwm_val = (unsigned short)SHDISP_BKL_PWM_LOWER_LIMIT;
        SHDISP_TRACE("out1\n");
        return;
    }

    if (level >= SHDISP_BKL_AUTO_PWM_TBL_NUM) {
        level = SHDISP_BKL_AUTO_PWM_TBL_NUM - 1;
    }

    min_val = shdisp_main_bkl_min_tbl[level];
    max_val = shdisp_main_bkl_max_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_ECO:
    case SHDISP_BKL_TBL_MODE_EMERGENCY:
        if (shdisp_bdic_bkl_param_auto <= SHDISP_MAIN_BKL_PARAM_MIN_AUTO) {
            val = (unsigned long)min_val;
        } else if (shdisp_bdic_bkl_param_auto >= SHDISP_MAIN_BKL_PARAM_MAX_AUTO) {
            val = (unsigned long)max_val;
        } else {
            val = (unsigned long)((max_val - min_val) * (shdisp_bdic_bkl_param_auto - 2));
            val /= (unsigned char)SHDISP_BKL_AUTO_STEP_NUM;
            val += (unsigned long)min_val;
        }
        if ((mode == SHDISP_BKL_TBL_MODE_EMERGENCY) && (val > SHDISP_BKL_EMERGENCY_LIMIT_AUTO)) {
            val = SHDISP_BKL_EMERGENCY_LIMIT_AUTO;
        }
        break;
    case SHDISP_BKL_TBL_MODE_CHARGE:
        val = (unsigned long)max_val;
        break;
    default:
        val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT;
        break;
    }

    if (val < (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT) {
        val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT;
    } else if (val > (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT) {
        val = (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT;
    } else {
        ;
    }
    *pwm_val = (unsigned short)val;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_on(void)
{
    int ret;

    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_off(void)
{
    int ret;

    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_on(void)
{
    int ret;

    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_off(void)
{
    int ret;

    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_OFF) {
        return;
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_doKick_if_exist.\n");
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_on(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_on);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_off(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_off);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_on(void)
{
    SHDISP_TRACE("in\n")
    if (s_state_str.bdic_chipver == SHDISP_BDIC_CHIPVER_0) {
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on_ts1);
    } else {
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on);
    }
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_off(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsn_off);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_VO2_ON                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_VO2_ON(void)
{
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_VO2_OFF                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_VO2_OFF(void)
{
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_active_for_led                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_active_for_led(int dev_type)
{
    SHDISP_TRACE("in dev_type:%d\n", dev_type);
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    SHDISP_TRACE("out Completed\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_standby_for_led                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_standby_for_led(int dev_type)
{
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_clmr_off_for_led                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_clmr_off_for_led(int dev_type)
{
    (void)shdisp_pm_clmr_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_bdic_seq_regset                                               */
/*---------------------------------------------------------------------------*/
static int shdisp_bdic_seq_regset(
                const shdisp_bdicRegSetting_t *regtable, int size, unsigned char bkl_mode, unsigned char lux_mode)
{
    int i, cnt_bdic, cnt_als;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t *tbl;
    unsigned char top_addr_bdic;
    unsigned char top_addr_als;
    unsigned char bBuf_bdic[16];
    unsigned char bBuf_als[8];

    cnt_bdic = 0;
    cnt_als  = 0;
    top_addr_bdic = 0x00;
    top_addr_als  = 0x00;

    tbl = (shdisp_bdicRegSetting_t *)regtable;
    for (i = 0; i < size; i++) {
        if (((cnt_bdic > 0) && (tbl->flg != SHDISP_BDIC_STRM)) || (cnt_bdic == sizeof(bBuf_bdic))) {
            ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            cnt_bdic = 0;
            top_addr_bdic = 0x00;
        }
        if (((cnt_als > 0) && (tbl->flg != SHDISP_ALS_STRM)) || (cnt_als == sizeof(bBuf_als))) {
            ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
            cnt_als = 0;
            top_addr_als = 0x00;
        }
        switch (tbl->flg) {
        case SHDISP_BDIC_STR:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
            ret = shdisp_bdic_IO_set_bit_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CLR:
            ret = shdisp_bdic_IO_clr_bit_reg(tbl->addr, tbl->mask);
            break;
        case SHDISP_BDIC_RMW:
            ret = shdisp_bdic_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BDIC_STRM:
            if (cnt_bdic == 0) {
                top_addr_bdic = tbl->addr;
            }
            bBuf_bdic[cnt_bdic] = tbl->data;
            cnt_bdic++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
            ret = shdisp_bdic_als_clmr_write_cmd(tbl->addr, tbl->data);
            break;
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            if (cnt_als == 0) {
                top_addr_als = tbl->addr;
            }
            bBuf_als[cnt_als] = tbl->data;
            cnt_als++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
                cnt_als = 0;
                top_addr_als = 0x00;
            }
            break;
        case SHDISP_ALS_RMW:
            ret = shdisp_bdic_als_clmr_msk_write_cmd(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BKL_MODE:
            shdisp_bdic_set_bkl_mode(bkl_mode, tbl->data , tbl->mask);
            break;
        case SHDISP_LUX_MODE:
            shdisp_bdic_API_set_lux_mode(lux_mode, tbl->data , tbl->mask);
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic R/W Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            return ret;
        }
        if (tbl->wait > 0) {
            if ((cnt_bdic > 0) && (tbl->flg == SHDISP_BDIC_STRM)) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            if ((cnt_als > 0) && (tbl->flg == SHDISP_ALS_STRM)) {
                ret = shdisp_bdic_als_clmr_burst_write_cmd(top_addr_als, bBuf_als, cnt_als);
                cnt_als = 0;
                top_addr_als = 0x00;
            }
            shdisp_SYS_cmd_delay_us(tbl->wait);
        }
        tbl++;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_active(int power_status)
{
    int ret;
    unsigned char version;

    SHDISP_TRACE("in\n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    if (power_status == SHDISP_DEV_STATE_NOINIT) {
        SHDISP_BDIC_REGSET(shdisp_bdic_init1);
        shdisp_SYS_bdic_i2c_doKick_if_exist();

        ret = shdisp_bdic_IO_read_check_reg(BDIC_REG_VERSION, &version);
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
        s_state_str.bdic_chipver = (int)SHDISP_BDIC_GET_CHIPVER(version);

        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
        SHDISP_BDIC_REGSET(shdisp_bdic_init2);
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_active);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);

    SHDISP_BDIC_REGSET(shdisp_bdic_standby);

    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_bkl_on                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_bkl_on(void)
{
    int ret;
    unsigned char val = 0x00;

    SHDISP_TRACE("in\n");

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_current);
    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);

    ret = shdisp_bdic_IO_read_reg(BDIC_REG_GINF3 , &val);
    SHDISP_DEBUG("DCDC1 Err Chk. ret=%d. val=0x%02x\n", ret, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        if ((val & SHDISP_BDIC_GINF3_DCDC1_OVD) == SHDISP_BDIC_GINF3_DCDC1_OVD) {
            SHDISP_ERR("DCDC1_OVD bit ON.\n");
            SHDISP_BDIC_REGSET(shdisp_bdic_dcdc1_err);
        }
    } else {
        SHDISP_ERR("BDIC GINF3 read error.\n");
    }

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{
    unsigned char value[1];

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            shdisp_bdic_PD_bkl_on();

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);
        } else if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
        } else if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO) {
            if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX) {
                shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

                value[0] = 0x01;
                shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);
            } else if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO) {
                shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

                value[0] = 0x01;
                shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
            }
        }
        break;

    case SHDISP_BDIC_REQ_PRE_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            shdisp_bdic_PD_bkl_on();

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_FIX_PARAM);

            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

            value[0] = 0x03;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);
        }
        break;

    case SHDISP_BDIC_REQ_POST_START_FIX:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_post_start);
        }
        break;

    case SHDISP_BDIC_REQ_POST_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_post_start);
        } else if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

            value[0] = 0x03;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value , 1);
        } else if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO) {
            shdisp_bdic_PWM_set_value(SHDISP_BDIC_BL_PWM_AUTO_PARAM);

            value[0] = 0x01;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_ALS_PARAM_SET, value , 1);
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        value[0] = 0x00;
        shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_MODE_SET, value, 1);

        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = SHDISP_MAIN_BKL_PARAM_OFF;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_off);
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_AUTO;
        shdisp_bdic_bkl_param_auto = param;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_ON;
        break;

    default:
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char    reg;
    unsigned char    bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPIO_0;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPIO_1;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPIO_2;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPIO_3;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPIO_4;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD5:
        reg = BDIC_REG_GPIO_5;
        bit = 0x01;
        break;
    default:
        return;
    }
    if (status == SHDISP_BDIC_GPIO_HIGH) {
        shdisp_bdic_IO_set_bit_reg( reg, bit );
    } else {
        shdisp_bdic_IO_clr_bit_reg( reg, bit );
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_api_set_led_fix_on_table                                      */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_api_set_led_fix_on_table(int clr_vari, int color)
{
    unsigned char *pTriLed;

    pTriLed = (unsigned char*)(&(shdisp_triple_led_tbl[clr_vari][color]));

    shdisp_bdic_led_fix_on[0].data = *(pTriLed + 0);
    shdisp_bdic_led_fix_on[1].data = *(pTriLed + 1);
    shdisp_bdic_led_fix_on[2].data = *(pTriLed + 2);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_START.tri_led_mode=%d.led_before_mode=%d.\n"
                       , shdisp_bdic_tri_led_mode, shdisp_bdic_tri_led_before_mode);
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        switch (shdisp_bdic_tri_led_before_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            if (shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off_fix);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            }
            break;
        case SHDISP_BDIC_TRI_LED_MODE_OFF:
            break;
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        default:
            SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            break;
        }

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_api_set_led_fix_on_table(s_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_TRI_LED_set_chdig();
            shdisp_bdic_PD_TRI_LED_set_anime();
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);
        }
        shdisp_bdic_tri_led_before_mode = shdisp_bdic_tri_led_mode;
        break;

    case SHDISP_BDIC_REQ_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        shdisp_bdic_tri_led_mode        = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char timeer1_val;
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;

    timeer1_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
    timeer1_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER, (unsigned char)timeer1_val, 0xF7);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        ch_set1_val = 0x06;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER, (unsigned char)timeer1_val, 0xF7);
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    int clrvari = s_state_str.bdic_clrvari_index;
    unsigned char wBuf[9];

    memset( wBuf, 0, sizeof( wBuf ));

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        wBuf[0] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[1] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[2] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        wBuf[3] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][0];
        wBuf[4] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][1];
        wBuf[5] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][2];
        wBuf[6] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[7] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[8] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PWM_set_value                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PWM_set_value(int pwm_param)
{
    int mode = 0;
    int bkl_auto_start_level;
    unsigned char *pwm_value;
    unsigned short pwm_table[SHDISP_BKL_AUTO_PWM_TBL_NUM];

    SHDISP_TRACE("in\n");

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_OFF:
        break;

    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, pwm_table);

        pwm_value = (unsigned char *)pwm_table;
        shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_PARAM_WRITE,
                                            pwm_value + 1,
                                            sizeof(unsigned short) + 1);
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        if (pwm_param == SHDISP_BDIC_BL_PWM_FIX_PARAM) {
            bkl_auto_start_level = shdisp_bdic_PD_get_auto_bkl_level();
            shdisp_bdic_LD_LCD_BKL_get_auto_fix_param(mode, bkl_auto_start_level, pwm_table);
            pwm_value = (unsigned char *)pwm_table;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_PARAM_WRITE,
                                                pwm_value + 1,
                                                sizeof(unsigned short) + 1);
        } else {
            pwm_value = (unsigned char *)shdisp_main_bkl_lumi_tbl;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_PARAM_WRITE,
                                                pwm_value + 1,
                                                (SHDISP_BKL_AUTO_PWM_TBL_NUM * 2) - 1);

            shdisp_bdic_LD_LCD_BKL_get_auto_param(mode, pwm_table);

            pwm_value = (unsigned char *)pwm_table;
            shdisp_bdic_bkl_adj_clmr_write_cmd(SHDISP_BDIC_BL_PARAM_WRITE,
                                                pwm_value + 1,
                                                (SHDISP_BKL_AUTO_PWM_TBL_NUM * 2) - 1);

        }
        break;

    default:
        break;

    }

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;
    int sensor_state;
    int i;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = lux_data  & 0x000000ff;
    lux_mode = ((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_on);
    SHDISP_PSALS_REGSET(shdisp_bdic_psals_init);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    sensor_state = shdisp_bdic_API_get_sensor_state();
    if (sensor_state == 0x00) {
        SHDISP_ERR("psals poweron(first) failed!! sensor_state = %02x\n", sensor_state);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PSALS_ON_NG;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */

        for (i = 0; i < 3; i++) {
            shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
            SHDISP_PSALS_REGSET(shdisp_bdic_psals_init);
            ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

            sensor_state = shdisp_bdic_API_get_sensor_state();
            if (sensor_state != 0x00) {
                break;
            }
            SHDISP_WARN("try psals poweron failed(%d)!! sensor_state = %02x\n", i + 1, sensor_state);
        }

        if (i == 3) {
            SHDISP_ERR("psals poweron retry over!!\n");
            if (psals_recovery_flag == 1) {
                psals_recovery_flag = 2;
            }
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PSALS;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_POWER_ON_NG;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        }
    }


    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_off                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_off);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    psals_recovery_flag = 0;
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_off                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_off(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_off1);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_off2);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_off3);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_on                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_on1);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_on2);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_init_als_on3);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_off                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_off(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_deinit_als_off1);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_PD_wait4i2ctimer_stop();

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_deinit_als_off2);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_on                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_ps_deinit_als_on);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_off                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_off(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_init_ps_off);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_on                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_init_ps_on1);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_PD_wait4i2ctimer_stop();

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_init_ps_on2);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_off                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_off(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_deinit_ps_off1);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    shdisp_bdic_PD_wait4i2ctimer_stop();

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_deinit_ps_off2);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_on                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_on(void)
{
    int ret;
    unsigned char lux_mode, bkl_mode;
    int lux_data;

    SHDISP_TRACE("in\n");

    lux_data = shdisp_bdic_get_lux_data();
    bkl_mode = (unsigned char)(lux_data  & 0x000000ff);
    lux_mode = (unsigned char)((lux_data & 0x0000ff00) >> 8);

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    SHDISP_PSALS_REGSET(shdisp_bdic_als_deinit_ps_on);
    ret = shdisp_SYS_bdic_i2c_doKick_if_exist();

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_write_threshold                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params)
{
    if (!prox_params) {
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_bdic_ps_init_set_threshold[0].data = (unsigned char)(prox_params->threshold_low & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[1].data = (unsigned char)((prox_params->threshold_low >> 8) & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[2].data = (unsigned char)(prox_params->threshold_high & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[3].data = (unsigned char)((prox_params->threshold_high >> 8) & 0x00FF);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_write_cmd                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_write_cmd(unsigned char reg, unsigned char val)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[2];

    ary[0] = reg;
    ary[1] = val;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_WRITE, 2, ary);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_WRITE, 2, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_msk_write_cmd                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_msk_write_cmd(unsigned char reg, unsigned char val, unsigned char msk)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[3];

    ary[0] = reg;
    ary[1] = msk;
    ary[2] = val;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_MASK_WRITE, 3, ary);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_MASK_WRITE, 3, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_is_recovery_successful                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_is_recovery_successful(void)
{
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    if (psals_recovery_flag == 2) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x08);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x08);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_RECOVERY_NG;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    psals_recovery_flag = 0;
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_bkl_adj_clmr_write_cmd                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_bkl_adj_clmr_write_cmd(int mode, unsigned char* value, int len )
{
#ifndef SHDISP_FW_STACK_EXCUTE
    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
    }
#endif
    switch (mode) {
    case SHDISP_BDIC_BL_PARAM_WRITE:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_WRITE, len, value);
        break;
    case SHDISP_BDIC_BL_MODE_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_BKLMODE_SET, len, value);
        bkl_mode_recovery = *value;
        break;
    case SHDISP_BDIC_ALS_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_LUXMODE_SET, len, value);
        lux_mode_recovery = *value;
        break;
    case SHDISP_BDIC_ALS_PARAM_SET:
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_PARAM_REFLECT, len, value);
        break;
    case SHDISP_BDIC_DEVICE_SET:
        break;
    default:
        break;
    }
#ifndef SHDISP_FW_STACK_EXCUTE
    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_SYS_bdic_i2c_doKick_if_exist();
        shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_BKL);
    }
#endif
    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_read_cmd                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_read_cmd(unsigned char reg, unsigned char *val)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[1];

    ary[0] = reg;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, ary);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 1, val);
    }

    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_clmr_burst_write_cmd                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_clmr_burst_write_cmd(unsigned char addr, unsigned char *val, unsigned char size)
{
    int result = SHDISP_RESULT_SUCCESS;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add_burst(SHDISP_CLMR_FWCMD_ALS_BURST_WRITE, addr, size, val);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
        shdisp_FWCMD_buf_add_burst(SHDISP_CLMR_FWCMD_ALS_BURST_WRITE, addr, size, val);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_get_opt                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value)
{
    int InfoReg2;

    SHDISP_TRACE("in\n");

    InfoReg2 = shdisp_SYS_getInfoReg2();

    *value = (unsigned short)(InfoReg2 & 0x0000ffff);

    SHDISP_DEBUG("value=0x%04x\n", *value);

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_clmr_get_ado_cmd                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_clmr_get_ado_cmd(unsigned short *ado_value)
{
    unsigned char rtnbuf[8] = {0};

    SHDISP_TRACE("in\n");

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_safe_finishanddoKick();
    }

    shdisp_SYS_bdic_i2c_set_api(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_LIGHTCTL_READ_NOFILTER, 0, NULL);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 6, rtnbuf);
    *ado_value = (unsigned short)(((unsigned short)rtnbuf[1] << 8) | (unsigned short)rtnbuf[0]);

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_wait4i2ctimer_stop                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_wait4i2ctimer_stop(void)
{
    int ret;
    int waitcnt = 3;
    unsigned char val = 0x00;

    SHDISP_TRACE("in\n");

    do {
        shdisp_SYS_delay_us(3 * 1000);

        ret = shdisp_bdic_IO_read_reg(BDIC_REG_SYSTEM8, &val);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("retry(%d)!! SYSTEM8 read error.\n", waitcnt);
            continue;
        }

        if (val != 0xA0) {
            SHDISP_DEBUG("retry(%d)!! SYSTEM8 = 0x%02x\n", waitcnt, val);
            continue;
        }

        SHDISP_TRACE("out 1\n");
        return SHDISP_RESULT_SUCCESS;
    } while (0 < --waitcnt);

    SHDISP_ERR("i2ctimer wait failed.\n");
    SHDISP_TRACE("out 2\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_get_auto_bkl_level                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_PD_get_auto_bkl_level(void)
{
    int i;
    unsigned int ret_value = 0;
    unsigned short ado_value;

    SHDISP_TRACE("in\n");

    shdisp_bdic_PD_clmr_get_ado_cmd(&ado_value);

    if (ado_value < mp_lux_th) {
        ret_value = 0;
    } else {
        for (i = 1; i < SHDISP_BKL_AUTO_PWM_TBL_NUM; i++) {
            if (ado_value <= shdisp_main_bkl_lumi_tbl[i]) {
                break;
            }
        }
        ret_value = i;
    }

    SHDISP_TRACE("out ADO=%04X, bkl_value=%d\n", ado_value, ret_value);
    return ret_value;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_sensor_state                                          */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_get_sensor_state(void)
{
    unsigned char para[1] = { 0x00 };
    unsigned char rtnbuf[4] = {0};

    SHDISP_TRACE("in\n");

    shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PHOTOSENSOR);
    shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_ALS_1BYTE_READ, 1, para);
    shdisp_FWCMD_buf_finish();
    shdisp_FWCMD_doKick(1, 4, rtnbuf);

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_PSALS) {
        shdisp_dbg_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force sensor state error.\n");
        rtnbuf[0] = 0x00;
        rtnbuf[1] = 0x00;
        rtnbuf[2] = 0x00;
        rtnbuf[3] = 0x00;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_TRACE("out\n");
    return (rtnbuf[3] << 24) | (rtnbuf[2] << 16) | (rtnbuf[1] << 8) | rtnbuf[0];
}

/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_write_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_write(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_write_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_multi_write_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_multi_write(reg, wval, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_read_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((reg == BDIC_REG_TEST_B3)
    ||  (reg == BDIC_REG_SYSTEM8)) {
        ret = shdisp_bdic_IO_read_no_check_reg(reg, val);
    } else {
        ret = shdisp_bdic_IO_read_check_reg(reg, val);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_no_check_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_read(reg, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_check_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    int retry = 0;
    unsigned char try_1st, try_2nd;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    try_1st = 0;
    try_2nd = 0;

    for (retry = 0; retry < 3; retry++) {
        ret = shdisp_SYS_bdic_i2c_read(reg, &try_1st);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        ret = shdisp_SYS_bdic_i2c_read(reg, &try_2nd);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        if (try_1st == try_2nd) {
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else if (retry == 2) {
            SHDISP_ERR("<OTHER> i2c read retry over! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                                            reg, try_1st, try_2nd);
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_BDIC;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_I2C_READ;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else {
            SHDISP_WARN("<OTHER> i2c read retry (%d)! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                                    retry, reg, try_1st, try_2nd);
        }
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;
    int maxreg;

    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }

    maxreg = (int)reg + (size - 1);
    if (maxreg > BDIC_REG_BANKSEL) {
        SHDISP_ERR("<OTHER> register address overflow.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SYS_bdic_i2c_multi_read(reg, val, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_read.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, 0x00, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_bank_set                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_bank_set(unsigned char val)
{
    int result = SHDISP_RESULT_SUCCESS;
    unsigned char ary[1];

    ary[0] = val;

    if (shdisp_FWCMD_buf_get_nokick()) {
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_I2C_BANK_SET, 1, ary);
    } else {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
        shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_I2C_BANK_SET, 1, ary);
        shdisp_FWCMD_buf_finish();
        result = shdisp_FWCMD_doKick(1, 0, 0);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, msk);
    return ret;
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_photo_sensor_IO_read_reg                                      */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];
    unsigned char rbuf[1];

    wbuf[0] = reg;
    rbuf[0] = 0x00;

    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = 1;
    msg.wbuf = &wbuf[0];
    msg.rbuf = &rbuf[0];

    ret = shdisp_bdic_API_ALS_transfer(&msg);
    if (ret == SHDISP_RESULT_SUCCESS) {
        *val = rbuf[0];
    }

    return ret;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_probe                                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    int rc = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in pdev = 0x%p\n", pdev );

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!\n");
            rc = 0;
            goto probe_done;
        } else {
            shdisp_SYS_set_irq_port(res->start, pdev);
        }
    }

probe_done:
    SHDISP_TRACE("out rc = %d\n", rc );

    return rc;
#else
    return SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_OF */
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_remove                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_remove(struct platform_device *pdev)
{
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_bdic_dt_match[] = {
    { .compatible = "sharp,shdisp_bdic", } ,
    {}
};
#else
#define shdisp_bdic_dt_match NULL
#endif /* CONFIG_OF */

static struct platform_driver shdisp_bdic_driver = {
    .probe = shdisp_bdic_probe,
    .remove = shdisp_bdic_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_bdic",
        .of_match_table = shdisp_bdic_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_register_driver                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_register_driver(void)
{
    return platform_driver_register(&shdisp_bdic_driver);
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
