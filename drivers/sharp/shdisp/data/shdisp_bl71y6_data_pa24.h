/* drivers/sharp/shdisp/data/shdisp_bl71y6_data_pa24.h  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

#ifndef SHDISP_BL71Y6_DATA_PA24_H
#define SHDISP_BL71Y6_DATA_PA24_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bl71y6.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BKL_FIX_TBL_NUM                  (256)
#define SHDISP_BKL_AUTO_STEP_NUM                (253)
#define SHDISP_BKL_AUTO_PWM_TBL_NUM             (1 + 32)
#define SHDISP_BKL_EMERGENCY_LIMIT_AUTO         (0x006C)
#define SHDISP_BKL_EMERGENCY_LIMIT_FIX          (0x006C)
#define SHDISP_BKL_PWM_LOWER_LIMIT              (0x0010)
#define SHDISP_BKL_PWM_UPPER_LIMIT              (0x03E8)

#define SHDISP_TRI_LED_COLOR_TBL_NUM            (10)
#define NUM_SHDISP_BKL_TBL_MODE                 (SHDISP_BKL_TBL_MODE_CHARGE + 1)

#define SHDISP_INT_ENABLE_GFAC                  (0x002C0308)
#define SHDISP_LUX_CHANGE_LEVEL1                (0x0B)
#define SHDISP_LUX_CHANGE_LEVEL2                (0x01)

#define CABC_LUX_LEVEL_LUT0_1                   (0x01)
#define CABC_LUX_LEVEL_LUT1_2                   (0x02)
#define CABC_LUX_LEVEL_LUT2_3                   (0x03)
#define CABC_LUX_LEVEL_LUT3_4                   (0x04)
#define CABC_LUX_LEVEL_LUT4_5                   (0x05)

#define SHDISP_COL_VARI_KIND                    (3)
#define SHDISP_HANDSET_COLOR_WHITE              (0x01)
#define SHDISP_HANDSET_COLOR_PINK               (0x02)
#define SHDISP_HANDSET_COLOR_NAVY               (0x04)
#define SHDISP_HANDSET_COLOR_BLACK              (0x06)

/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

#define BDIC_REG_VO2_PFML_VAL                   (0x17)
#define BDIC_REG_VO2_PWML_VAL                   (0x17)
#define BDIC_REG_VO2_PWMH_VAL                   (0x17)
#define BDIC_REG_M1LED_VAL                      (0xBE)
#define BDIC_REG_M2LED_VAL                      (0xBE)
#define BDIC_REG_SYSTEM1_BKL                    (0x03)
#define BDIC_REG_DCDC1_CLIMT_2_VAL              (0xC0)
#define BDIC_REG_DCDC1_GM_VAL                   (0x08)
#define BDIC_REG_DCDC1_OSC_2_VAL                (0x01)
#define BDIC_REG_DCDC1_OVDETREF_VAL             (0x88)
#define BDIC_REG_DCDC1_TH_LCUR_VAL              (0x12)
#define BDIC_REG_DCDC1_TH_HCUR_VAL              (0x16)

#define BDIC_REG_ALS_ADJ0_L_DEFAULT_A           (0x5C)
#define BDIC_REG_ALS_ADJ0_H_DEFAULT_A           (0x3F)
#define BDIC_REG_ALS_ADJ1_L_DEFAULT_A           (0xAA)
#define BDIC_REG_ALS_ADJ1_H_DEFAULT_A           (0x4C)
#define BDIC_REG_ALS_SHIFT_DEFAULT_A            (0x03)
#define BDIC_REG_CLEAR_OFFSET_DEFAULT_A         (0x00)
#define BDIC_REG_IR_OFFSET_DEFAULT_A            (0x00)

#define BDIC_REG_ALS_ADJ0_L_DEFAULT_B           (0xAC)
#define BDIC_REG_ALS_ADJ0_H_DEFAULT_B           (0x0C)
#define BDIC_REG_ALS_ADJ1_L_DEFAULT_B           (0x00)
#define BDIC_REG_ALS_ADJ1_H_DEFAULT_B           (0x00)
#define BDIC_REG_ALS_SHIFT_DEFAULT_B            (0x03)
#define BDIC_REG_CLEAR_OFFSET_DEFAULT_B         (0x00)
#define BDIC_REG_IR_OFFSET_DEFAULT_B            (0x00)

#define BDIC_REG_CPM1_VAL                       (0x14)
#define BDIC_REG_CPM1_VAL_TS1                   (0x05)
#define BDIC_REG_CPM2_VAL                       (0x31)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const unsigned short shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM] = {
    0x0000,
    0x0010,
    0x0010,
    0x0011,
    0x0012,
    0x0013,
    0x0014,
    0x0015,
    0x0016,
    0x0017,
    0x0018,
    0x0019,
    0x001A,
    0x001B,
    0x001C,
    0x001D,
    0x001E,
    0x001F,
    0x0020,
    0x0022,
    0x0023,
    0x0024,
    0x0025,
    0x0026,
    0x0027,
    0x0028,
    0x0029,
    0x002A,
    0x002B,
    0x002C,
    0x002D,
    0x002E,
    0x002F,
    0x0030,
    0x0031,
    0x0033,
    0x0034,
    0x0035,
    0x0036,
    0x0037,
    0x0038,
    0x0039,
    0x003A,
    0x003B,
    0x003C,
    0x003D,
    0x003E,
    0x003F,
    0x0040,
    0x0041,
    0x0043,
    0x0044,
    0x0045,
    0x0046,
    0x0047,
    0x0048,
    0x0049,
    0x004A,
    0x004B,
    0x004C,
    0x004D,
    0x004E,
    0x004F,
    0x0050,
    0x0052,
    0x0053,
    0x0054,
    0x0055,
    0x0056,
    0x0057,
    0x0058,
    0x0059,
    0x005A,
    0x005C,
    0x005D,
    0x005E,
    0x005F,
    0x0060,
    0x0061,
    0x0063,
    0x0064,
    0x0065,
    0x0067,
    0x0068,
    0x0069,
    0x006B,
    0x006C,
    0x006D,
    0x006F,
    0x0070,
    0x0072,
    0x0073,
    0x0075,
    0x0076,
    0x0078,
    0x0079,
    0x007B,
    0x007D,
    0x007F,
    0x0080,
    0x0082,
    0x0084,
    0x0086,
    0x0088,
    0x008A,
    0x008C,
    0x008E,
    0x0090,
    0x0092,
    0x0094,
    0x0096,
    0x0099,
    0x009B,
    0x009D,
    0x00A0,
    0x00A2,
    0x00A5,
    0x00A7,
    0x00AA,
    0x00AD,
    0x00AF,
    0x00B2,
    0x00B5,
    0x00B8,
    0x00BB,
    0x00BE,
    0x00C1,
    0x00C4,
    0x00C7,
    0x00CB,
    0x00CE,
    0x00D1,
    0x00D5,
    0x00D8,
    0x00DC,
    0x00E0,
    0x00E3,
    0x00E7,
    0x00EB,
    0x00EF,
    0x00F3,
    0x00F7,
    0x00FB,
    0x00FF,
    0x0103,
    0x0108,
    0x010C,
    0x0110,
    0x0115,
    0x0119,
    0x011E,
    0x0122,
    0x0127,
    0x012C,
    0x0131,
    0x0135,
    0x013A,
    0x013F,
    0x0144,
    0x0149,
    0x014E,
    0x0154,
    0x0159,
    0x015E,
    0x0163,
    0x0169,
    0x016E,
    0x0174,
    0x0179,
    0x017F,
    0x0184,
    0x018A,
    0x0190,
    0x0195,
    0x019B,
    0x01A1,
    0x01A7,
    0x01AD,
    0x01B3,
    0x01B9,
    0x01BF,
    0x01C5,
    0x01CB,
    0x01D2,
    0x01D8,
    0x01DE,
    0x01E5,
    0x01EB,
    0x01F1,
    0x01F8,
    0x01FE,
    0x0205,
    0x020C,
    0x0212,
    0x0219,
    0x0220,
    0x0226,
    0x022D,
    0x0234,
    0x023B,
    0x0242,
    0x0249,
    0x0250,
    0x0257,
    0x025E,
    0x0265,
    0x026C,
    0x0273,
    0x027A,
    0x0281,
    0x0289,
    0x0290,
    0x0297,
    0x029F,
    0x02A6,
    0x02AD,
    0x02B5,
    0x02BC,
    0x02C4,
    0x02CB,
    0x02D3,
    0x02DA,
    0x02E2,
    0x02EA,
    0x02F1,
    0x02F9,
    0x0301,
    0x0308,
    0x0310,
    0x0318,
    0x0320,
    0x0328,
    0x032F,
    0x0337,
    0x033F,
    0x0347,
    0x034F,
    0x0357,
    0x035F,
    0x0367,
    0x036F,
    0x0377,
    0x037F,
    0x0387,
    0x038F,
    0x0397,
    0x039F,
    0x03A7,
    0x03AF,
    0x03B8,
    0x03C0,
    0x03C8,
    0x03D0,
    0x03D8,
    0x03E0,
    0x03E8
};

static const unsigned short shdisp_main_bkl_lumi_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
    0x1D00,
    0x0000, 0x0000, 0x0001, 0x0002, 0x0002, 0x0004, 0x0005, 0x0007,
    0x0008, 0x000C, 0x000F, 0x0018, 0x0036, 0x004C, 0x0062, 0x0079,
    0x00AA, 0x00DD, 0x0163, 0x01F1, 0x0476, 0x05E6, 0x0752, 0x0A25,
    0x0E56, 0x153D, 0x1C12, 0x2995, 0x36F5, 0x443A, 0x516A, 0x58B0
};

static const unsigned short shdisp_main_bkl_min_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
    0x3D00,
    0x0005, 0x0005, 0x0005, 0x000B, 0x000B, 0x0010, 0x0010, 0x0010,
    0x0010, 0x0010, 0x0010, 0x0015, 0x001A, 0x0020, 0x0020, 0x0020,
    0x0025, 0x0025, 0x002A, 0x002F, 0x0035, 0x0035, 0x003A, 0x003F,
    0x003F, 0x004F, 0x0064, 0x009E, 0x00DD, 0x0131, 0x0185, 0x01CF
};

static const unsigned short shdisp_main_bkl_max_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
    0x3D00,
    0x0054, 0x0054, 0x0054, 0x005F, 0x005F, 0x0074, 0x008E, 0x00A3,
    0x00BD, 0x00ED, 0x0112, 0x0161, 0x01F4, 0x0238, 0x0268, 0x0297,
    0x02E6, 0x032B, 0x039E, 0x03F8, 0x0471, 0x04C5, 0x0509, 0x057D,
    0x05F6, 0x067F, 0x06D9, 0x075C, 0x07C0, 0x0829, 0x0898, 0x0907
};

static const struct shdisp_bdic_led_color_index shdisp_triple_led_color_index_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM] = {
    {0, 0, 0,  0},
    {1, 0, 0,  1},
    {0, 1, 0,  2},
    {1, 1, 0,  3},
    {0, 0, 1,  4},
    {1, 0, 1,  5},
    {0, 1, 1,  6},
    {1, 1, 1,  7},
    {2, 0, 0,  8},
    {0, 2, 0,  9}
};

static const unsigned char shdisp_clrvari_index[SHDISP_COL_VARI_KIND] = {
    SHDISP_HANDSET_COLOR_WHITE,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_NAVY
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    { 0x00, 0x00, 0x00 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 },
    { 0x30, 0x50, 0x00 },
    { 0x00, 0x00, 0x7F },
    { 0x50, 0x00, 0x50 },
    { 0x00, 0x50, 0x7F },
    { 0x40, 0x60, 0x60 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 },
    { 0x30, 0x50, 0x00 },
    { 0x00, 0x00, 0x7F },
    { 0x50, 0x00, 0x50 },
    { 0x00, 0x50, 0x50 },
    { 0x30, 0x60, 0x50 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x08, 0x00, 0x00 },
    { 0x00, 0x08, 0x00 },
    { 0x03, 0x04, 0x00 },
    { 0x00, 0x00, 0x10 },
    { 0x08, 0x00, 0x08 },
    { 0x00, 0x04, 0x08 },
    { 0x04, 0x05, 0x04 },
    { 0x08, 0x00, 0x00 },
    { 0x00, 0x08, 0x00 }
  }
};

static const unsigned char shdisp_triple_led_anime_tbl[SHDISP_COL_VARI_KIND][2][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 },
        { 0x30, 0x50, 0x00 },
        { 0x00, 0x00, 0x7F },
        { 0x50, 0x00, 0x50 },
        { 0x00, 0x50, 0x7F },
        { 0x40, 0x60, 0x60 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 },
        { 0x30, 0x50, 0x00 },
        { 0x00, 0x00, 0x7F },
        { 0x50, 0x00, 0x50 },
        { 0x00, 0x50, 0x50 },
        { 0x30, 0x60, 0x50 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x08, 0x00, 0x00 },
        { 0x00, 0x08, 0x00 },
        { 0x06, 0x08, 0x00 },
        { 0x00, 0x00, 0x10 },
        { 0x08, 0x00, 0x08 },
        { 0x00, 0x08, 0x10 },
        { 0x08, 0x0A, 0x08 },
        { 0x08, 0x00, 0x00 },
        { 0x00, 0x08, 0x00 }
    }
  }
};

static const struct shdisp_bdic_bkl_ado_tbl shdisp_bdic_bkl_ado_tbl[5] = {
    {   0,      1,   811,      149},
    {   1,     15,   663,      481},
    {  15,    221,   448,     4724},
    { 221,   3670,   252,    73562},
    {3670,  65536,   294,  -109588},
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



#endif /* SHDISP_BL71Y6_DATA_PA24_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
