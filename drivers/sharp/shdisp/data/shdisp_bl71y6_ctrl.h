/* drivers/sharp/shdisp/data/shdisp_bl71y6_ctrl.h  (Display Driver)
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

#ifndef SHDISP_BL71Y6_CTRL_H
#define SHDISP_BL71Y6_CTRL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bl71y6.h"

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

#ifdef SHDISP_BL71Y6_CTRL_DECLARE

const shdisp_bdicRegSetting_t shdisp_bdic_set_bank0[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};
const size_t shdisp_bdic_set_bank0_size = sizeof(shdisp_bdic_set_bank0) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_set_bank1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0}
};
const size_t shdisp_bdic_set_bank1_size = sizeof(shdisp_bdic_set_bank1) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_init1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x80,                       0xFF,  10600},
     {BDIC_REG_SYSTEM3,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};
const size_t shdisp_bdic_init1_size = sizeof(shdisp_bdic_init1) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_init2[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_STR,    0x20,                       0xFF,      0},
     {BDIC_REG_SYSTEM3,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM4,             SHDISP_BDIC_STR,    0x31,                       0xFF,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_STR,    0x28,                       0xFF,      0},
     {BDIC_REG_I2C_TIMER,           SHDISP_BDIC_STR,    0xFF,                       0xFF,      0},
     {BDIC_REG_I2C_SYS,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_I2C_DATA0,           SHDISP_BDIC_STR,    0x72,                       0xFF,      0},
     {BDIC_REG_XEN_TSD,             SHDISP_BDIC_STR,    0x01,                       0xFF,      0},

     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_STR,    0x01,                       0xFF,      0},
     {BDIC_REG_LDO,                 SHDISP_BDIC_STR,    0x01,                       0xFF,      0},
     {BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_STR,    0x80,                       0xFF,      0},
     {BDIC_REG_ALS_DATA1_SET,       SHDISP_BDIC_STR,    0x21,                       0xFF,      0},

     {BDIC_REG_MODE_M1,             SHDISP_BDIC_STR,    0x03,                       0xFF,      0},
     {BDIC_REG_PSDATA_SET,          SHDISP_BDIC_STR,    0x02,                       0xFF,      0},
     {BDIC_REG_PS_HT_LSB,           SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {BDIC_REG_PS_HT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_PS_LT_LSB,           SHDISP_BDIC_STR,    0x0F,                       0xFF,      0},
     {BDIC_REG_PS_LT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_CPM1,                SHDISP_BDIC_STR,    BDIC_REG_CPM1_VAL,          0xFF,      0},
     {BDIC_REG_CPM2,                SHDISP_BDIC_STR,    BDIC_REG_CPM2_VAL,          0xFF,      0},
     {BDIC_REG_DCDC2_ERES2_1,       SHDISP_BDIC_STR,    0x33,                       0xFF,      0},
     {BDIC_REG_DCDC2_ERES2_2,       SHDISP_BDIC_STR,    0x05,                       0xFF,      0},
     {BDIC_REG_DCDC2_CLIMIT_1,      SHDISP_BDIC_STR,    0x06,                       0xFF,      0},
     {BDIC_REG_DCDC2_CLIMIT_2,      SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_DCDC2_GM,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_DCDC2_BF,            SHDISP_BDIC_STR,    0xC3,                       0xFF,      0},
     {BDIC_REG_DCDC2_SW,            SHDISP_BDIC_STR,    0xC3,                       0xFF,      0},
     {BDIC_REG_DCDC2_OSC_1,         SHDISP_BDIC_STR,    0xB1,                       0xFF,      0},
     {BDIC_REG_DCDC2_OSC_2,         SHDISP_BDIC_STR,    0x01,                       0xFF,      0},
     {BDIC_REG_VO2_PFML,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PFML_VAL,      0xFF,      0},
     {BDIC_REG_VO2_PWML,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PWML_VAL,      0xFF,      0},
     {BDIC_REG_VO2_PWMH,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PWMH_VAL,      0xFF,      0},
     {BDIC_REG_DCCD2_MODE_SEL,      SHDISP_BDIC_STR,    0x85,                       0xFF,      0},
     {BDIC_REG_DCDC1_CUR_SEL_DRV,   SHDISP_BDIC_STR,    0x80,                       0xFF,      0},
     {BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_DCDC1_CLIMT_1,       SHDISP_BDIC_STR,    0xF5,                       0xFF,      0},
     {BDIC_REG_DCDC1_CLIMT_2,       SHDISP_BDIC_STR,    BDIC_REG_DCDC1_CLIMT_2_VAL, 0xFF,      0},
     {BDIC_REG_DCDC1_GM,            SHDISP_BDIC_STR,    BDIC_REG_DCDC1_GM_VAL,      0xFF,      0},
     {BDIC_REG_DCDC1_BF,            SHDISP_BDIC_STR,    0xF3,                       0xFF,      0},
     {BDIC_REG_DCDC1_SW,            SHDISP_BDIC_STR,    0xF3,                       0xFF,      0},
     {BDIC_REG_DCDC1_OSC_1,         SHDISP_BDIC_STR,    0x51,                       0xFF,      0},
     {BDIC_REG_DCDC1_OSC_2,         SHDISP_BDIC_STR,    BDIC_REG_DCDC1_OSC_2_VAL,   0xFF,      0},
     {BDIC_REG_DCDC1_OVDETREF,      SHDISP_BDIC_STR,    BDIC_REG_DCDC1_OVDETREF_VAL,0xFF,      0},
     {BDIC_REG_DCDC1_TIMER,         SHDISP_BDIC_STR,    0x04,                       0xFF,      0},
     {BDIC_REG_DCDC1_TH_LCUR,       SHDISP_BDIC_STR,    BDIC_REG_DCDC1_TH_LCUR_VAL, 0xFF,      0},
     {BDIC_REG_DCDC1_TH_HCUR,       SHDISP_BDIC_STR,    BDIC_REG_DCDC1_TH_HCUR_VAL, 0xFF,      0},
     {BDIC_REG_DCDC1_CUR_SOFT,      SHDISP_BDIC_STR,    0x04,                       0xFF,      0},
     {BDIC_REG_DCDC1_REF1_SOFT,     SHDISP_BDIC_STR,    0x22,                       0xFF,      0},
     {BDIC_REG_DCDC1_EAMPREF2,      SHDISP_BDIC_STR,    0x32,                       0xFF,      0},
     {BDIC_REG_DCDC1_PFMREF1,       SHDISP_BDIC_STR,    0x32,                       0xFF,      0},
     {BDIC_REG_DCDC1_PFMREF2,       SHDISP_BDIC_STR,    0x32,                       0xFF,      0},

     {BDIC_REG_M1LED,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0},
     {BDIC_REG_M2LED,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0},
     {BDIC_REG_PWMDC1,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_PWMDC2,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_GPIO_0,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GPIO_1,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GPIO_2,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GPIO_3,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GPIO_4,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0},
     {BDIC_REG_GPIO_5,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0},
     {BDIC_REG_GPIO_6,              SHDISP_BDIC_STR,    0x04,                       0xFF,      0},
     {BDIC_REG_GPIO_ANSW,           SHDISP_BDIC_STR,    0x1C,                       0xFF,      0},
     {BDIC_REG_GPIMSK0,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    0x21,                       0xFF,      0},
     {BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_INT_CTRL,            SHDISP_BDIC_STR,    0x02,                       0xFF,      0},
     {BDIC_REG_GPIO_OPD_PUSEL,      SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_DETECTOR,            SHDISP_BDIC_STR,    0x02,                       0xFF,      0},
     {BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMR2,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMR3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMR4,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMF3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_GIMF4,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_OPT_INT1,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_OPT_INT2,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0},
     {BDIC_REG_PS_DATA_L,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_PS_DATA_H,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_CLR_DATA_L,          SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_CLR_DATA_H,          SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_IR_DATA_L,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_IR_DATA_H,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_OPT0_LT_L,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_OPT0_LT_H,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0},

     {BDIC_REG_OPT23_HT_L,          SHDISP_BDIC_STR,    0xFF,                       0xFF,      0},
     {BDIC_REG_OPT23_HT_H,          SHDISP_BDIC_STR,    0xFF,                       0xFF,      0},

     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};
const size_t shdisp_bdic_init2_size = sizeof(shdisp_bdic_init2) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_active[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x80,                       0x80,   1000},
     {BDIC_REG_SYSTEM3,             SHDISP_BDIC_SET,    0x02,                       0x02,      0}
};
const size_t shdisp_bdic_active_size = sizeof(shdisp_bdic_active) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_standby[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM3,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x80,      0}
};
const size_t shdisp_bdic_standby_size = sizeof(shdisp_bdic_standby) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x10,                       0x50,      0}
};
const size_t shdisp_bdic_vsn_on_size = sizeof(shdisp_bdic_vsn_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on_ts1[] = {
     {BDIC_REG_CPM1,                SHDISP_BDIC_STR,    BDIC_REG_CPM1_VAL_TS1,      0xFF,      0},
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x20,      0},
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x10,                       0x50,  16000},
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_SET,    0x20,                       0x20,      0}
};
const size_t shdisp_bdic_vsn_on_ts1_size = sizeof(shdisp_bdic_vsn_on_ts1) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_vsn_off[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x40,                       0x50,   1000}
};
const size_t shdisp_bdic_vsn_off_size = sizeof(shdisp_bdic_vsn_off) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_vsp_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_SET,    0x01,                       0x01,   3000}
};
const size_t shdisp_bdic_vsp_on_size = sizeof(shdisp_bdic_vsp_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_vsp_off[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x01,      0},
     {BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x80,                       0xFF,   2000}
};
const size_t shdisp_bdic_vsp_off_size = sizeof(shdisp_bdic_vsp_off) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_bkl_current[] = {
     {BDIC_REG_M1LED,               SHDISP_BDIC_STR,    0x01,                       0xFF,      0},
     {BDIC_REG_M2LED,               SHDISP_BDIC_STR,    0x01,                       0xFF,      0},
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x55,      0}
};
const size_t shdisp_bdic_bkl_current_size = sizeof(shdisp_bdic_bkl_current) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_bkl_on[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_RMW,    BDIC_REG_SYSTEM1_BKL,       0x03,   5000}
};
const size_t shdisp_bdic_bkl_on_size = sizeof(shdisp_bdic_bkl_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_bkl_post_start[] = {
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x04,                       0x05,      0}
};
const size_t shdisp_bdic_bkl_post_start_size = sizeof(shdisp_bdic_bkl_post_start) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_bkl_off[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x03,  15000},
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x05,      0}
};
const size_t shdisp_bdic_bkl_off_size = sizeof(shdisp_bdic_bkl_off) / sizeof(shdisp_bdicRegSetting_t);

shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_CH0_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH1_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH2_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
};
const size_t shdisp_bdic_led_fix_on_size = sizeof(shdisp_bdic_led_fix_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0},
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x01,                       0xF3,   6000}
};
const size_t shdisp_bdic_led_ani_on_size = sizeof(shdisp_bdic_led_ani_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x10,                       0x10,      0}
};
const size_t shdisp_bdic_led_lposc_enable_size = sizeof(shdisp_bdic_led_lposc_enable) /
                                                 sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_CLR,    0x00,                       0xF3,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x10,      0}
};
const size_t shdisp_bdic_led_off_size = sizeof(shdisp_bdic_led_off) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_led_off_fix[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
};
const size_t shdisp_bdic_led_off_fix_size = sizeof(shdisp_bdic_led_off_fix) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM4,             SHDISP_BDIC_SET,    0x02,                       0x02,   5000},
     {BDIC_REG_GIMR4,               SHDISP_BDIC_SET,    0x08,                       0x08,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
};
const size_t shdisp_bdic_sensor_power_on_size = sizeof(shdisp_bdic_sensor_power_on) / sizeof(shdisp_bdicRegSetting_t);

const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_off[] = {
     {BDIC_REG_SYSTEM4,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0}
};
const size_t shdisp_bdic_sensor_power_off_size = sizeof(shdisp_bdic_sensor_power_off) /
                                                 sizeof(shdisp_bdicRegSetting_t);

static shdisp_bdicRegSetting_t shdisp_bdic_ps_init_set_threshold[] = {
     {SENSOR_REG_PS_LT_LSB,         SHDISP_ALS_STRMS,   0xFF,   0xFF,      0},
     {SENSOR_REG_PS_LT_MSB,         SHDISP_ALS_STRM,    0xFF,   0xFF,      0},
     {SENSOR_REG_PS_HT_LSB,         SHDISP_ALS_STRM,    0xFF,   0xFF,      0},
     {SENSOR_REG_PS_HT_MSB,         SHDISP_ALS_STRM,    0xFF,   0xFF,      0},
     {BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,   0x00,   1000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_psals_init[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x28,                       0xFF,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x90,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_off1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,      0},
};

#define shdisp_bdic_ps_init_als_off2            (shdisp_bdic_ps_init_set_threshold)
#define shdisp_bdic_ps_init_als_off2_size       (shdisp_bdic_ps_init_set_threshold_size)

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_off3[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xE0,                       0xFF,   1000},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,      0},
     {BDIC_REG_GIMR4,               SHDISP_BDIC_CLR,    0x00,                       0x20,      0},
     {BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,  35000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_off1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x00,                       0x01,      0},
     {BDIC_REG_GIMR4,               SHDISP_BDIC_CLR,    0x00,                       0x28,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x08,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_off2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x80,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x20,                       0xF8,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STR,     0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STR,     0xEC,                       0xFF,      0}
};

#define shdisp_bdic_ps_init_als_on2             (shdisp_bdic_ps_init_set_threshold)
#define shdisp_bdic_ps_init_als_on2_size        (shdisp_bdic_ps_init_set_threshold_size)

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on3[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xC0,                       0xFF,   1000},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,      0},
     {BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,  35000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x18,                       0xF8,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STR,     0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STR,     0x00,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,   1000},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_off[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x1C,                       0xFF,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,   1000},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x08,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_on2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x0C,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x24,                       0xFF,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xCC,                       0xFF,   1000},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x01,                       0x01,      0}
};

#define shdisp_bdic_als_deinit_ps_off1       (shdisp_bdic_ps_deinit_als_off1)
#define shdisp_bdic_als_deinit_ps_off2       (shdisp_bdic_ps_deinit_als_off2)

static const shdisp_bdicRegSetting_t shdisp_bdic_als_deinit_ps_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {CLMR_SET_BKL_LUT_MODE,        SHDISP_LUX_MODE,    0x00,                       0x01,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x0C,                       0xFF,      0},
     {SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,      0},
     {SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0},
     {SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,   1000},
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xEC,                       0xFF,   1000},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,      0},
     {BDIC_REG_GIMR4,               SHDISP_BDIC_CLR,    0x00,                       0x20,      0},
     {BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,  20000}
};

const shdisp_bdicRegSetting_t shdisp_bdic_dcdc1_err[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x03,      0}
};
const size_t shdisp_bdic_dcdc1_err_size = sizeof(shdisp_bdic_dcdc1_err) / sizeof(shdisp_bdicRegSetting_t);

#else /* SHDISP_BL71Y6_CTRL_DECLARE */

extern const shdisp_bdicRegSetting_t shdisp_bdic_set_bank0[];
extern const size_t shdisp_bdic_set_bank0_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_set_bank1[];
extern const size_t shdisp_bdic_set_bank1_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_init1[];
extern const size_t shdisp_bdic_init1_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_init2[];
extern const size_t shdisp_bdic_init2_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_active[];
extern const size_t shdisp_bdic_active_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_standby[];
extern const size_t shdisp_bdic_standby_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on[];
extern const size_t shdisp_bdic_vsn_on_size;
extern const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on_ts1[];
extern const size_t shdisp_bdic_vsn_on_ts1_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_vsn_off[];
extern const size_t shdisp_bdic_vsn_off_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_vsp_on[];
extern const size_t shdisp_bdic_vsp_on_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_vsp_off[];
extern const size_t shdisp_bdic_vsp_off_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_bkl_current[];
extern const size_t shdisp_bdic_bkl_current_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_bkl_on[];
extern const size_t shdisp_bdic_bkl_on_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_bkl_post_start[];
extern const size_t shdisp_bdic_bkl_post_start_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_bkl_off[];
extern const size_t shdisp_bdic_bkl_off_size;

extern shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[];
extern const size_t shdisp_bdic_led_fix_on_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[];
extern const size_t shdisp_bdic_led_ani_on_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[];
extern const size_t shdisp_bdic_led_lposc_enable_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_led_off[];
extern const size_t shdisp_bdic_led_off_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_led_off_fix[];
extern const size_t shdisp_bdic_led_off_fix_size;


extern const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_on[];
extern const size_t shdisp_bdic_sensor_power_on_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_off[];
extern const size_t shdisp_bdic_sensor_power_off_size;

extern const shdisp_bdicRegSetting_t shdisp_bdic_dcdc1_err[];
extern const size_t shdisp_bdic_dcdc1_err_size;

#endif /* SHDISP_BL71Y6_CTRL_DECLARE */

#endif /* SHDISP_BL71Y6_CTRL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
