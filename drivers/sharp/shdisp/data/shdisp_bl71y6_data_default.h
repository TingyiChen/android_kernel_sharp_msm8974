/* drivers/sharp/shdisp/data/shdisp_bl71y6_data_default.h  (Display Driver)
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

#ifndef SHDISP_BL71Y6_DATA_DEFAULT_H
#define SHDISP_BL71Y6_DATA_DEFAULT_H

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
#define SHDISP_BKL_EMERGENCY_LIMIT_FIX          (0x006B)
#define SHDISP_BKL_PWM_LOWER_LIMIT              (0x000E)
#define SHDISP_BKL_PWM_UPPER_LIMIT              (0x03E8)

#define SHDISP_TRI_LED_COLOR_TBL_NUM            (8)
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
#define SHDISP_HANDSET_COLOR_BLACK              (0x06)

/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

#define BDIC_REG_VO2_PFML_VAL                   (0x1A)
#define BDIC_REG_VO2_PWML_VAL                   (0x1A)
#define BDIC_REG_VO2_PWMH_VAL                   (0x1A)
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
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0014,
     0x0015,
     0x0015,
     0x0015,
     0x0015,
     0x0016,
     0x0016,
     0x0016,
     0x0017,
     0x0017,
     0x0018,
     0x0018,
     0x0019,
     0x0019,
     0x001A,
     0x001A,
     0x001B,
     0x001C,
     0x001C,
     0x001D,
     0x001E,
     0x001F,
     0x0020,
     0x0020,
     0x0021,
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
     0x002D,
     0x002E,
     0x002F,
     0x0030,
     0x0032,
     0x0033,
     0x0034,
     0x0036,
     0x0037,
     0x0038,
     0x003A,
     0x003C,
     0x003D,
     0x003F,
     0x0040,
     0x0042,
     0x0043,
     0x0045,
     0x0047,
     0x0049,
     0x004A,
     0x004C,
     0x004E,
     0x0050,
     0x0052,
     0x0054,
     0x0056,
     0x0058,
     0x005A,
     0x005C,
     0x005E,
     0x0060,
     0x0062,
     0x0064,
     0x0066,
     0x0069,
     0x006B,
     0x006D,
     0x006F,
     0x0072,
     0x0074,
     0x0077,
     0x0079,
     0x007B,
     0x007E,
     0x0080,
     0x0083,
     0x0086,
     0x0088,
     0x008B,
     0x008E,
     0x0090,
     0x0093,
     0x0096,
     0x0099,
     0x009B,
     0x009E,
     0x00A1,
     0x00A4,
     0x00A7,
     0x00AA,
     0x00AD,
     0x00B0,
     0x00B3,
     0x00B6,
     0x00BA,
     0x00BD,
     0x00C0,
     0x00C3,
     0x00C7,
     0x00CA,
     0x00CD,
     0x00D1,
     0x00D4,
     0x00D7,
     0x00DB,
     0x00DF,
     0x00E2,
     0x00E6,
     0x00E9,
     0x00ED,
     0x00F1,
     0x00F4,
     0x00F8,
     0x00FC,
     0x0100,
     0x0104,
     0x0107,
     0x010B,
     0x010F,
     0x0113,
     0x0117,
     0x011B,
     0x011F,
     0x0124,
     0x0128,
     0x012C,
     0x0130,
     0x0134,
     0x0139,
     0x013D,
     0x0141,
     0x0146,
     0x014A,
     0x014F,
     0x0153,
     0x0158,
     0x015C,
     0x0161,
     0x0166,
     0x016A,
     0x016F,
     0x0174,
     0x0179,
     0x017E,
     0x0182,
     0x0187,
     0x018C,
     0x0191,
     0x0196,
     0x019B,
     0x01A0,
     0x01A5,
     0x01AB,
     0x01B0,
     0x01B5,
     0x01BA,
     0x01C0,
     0x01C5,
     0x01CA,
     0x01D0,
     0x01D5,
     0x01DB,
     0x01E0,
     0x01E6,
     0x01EC,
     0x01F1,
     0x01F7,
     0x01FD,
     0x0202,
     0x0208,
     0x020E,
     0x0214,
     0x021A,
     0x0220,
     0x0226,
     0x022C,
     0x0232,
     0x0238,
     0x023F,
     0x0245,
     0x024B,
     0x0251,
     0x0258,
     0x025E,
     0x0264,
     0x026B,
     0x0271,
     0x0278,
     0x027E,
     0x0285,
     0x028C,
     0x0292,
     0x0299,
     0x02A0,
     0x02A7,
     0x02AE,
     0x02B5,
     0x02BC,
     0x02C3,
     0x02CA,
     0x02D1,
     0x02D8,
     0x02DF,
     0x02E6,
     0x02EE,
     0x02F5,
     0x02FC,
     0x0304,
     0x030B,
     0x0313,
     0x031A,
     0x0322,
     0x0329,
     0x0331,
     0x0339,
     0x0341,
     0x0348,
     0x0350,
     0x0358,
     0x0360,
     0x0368,
     0x0370,
     0x0378,
     0x0380,
     0x0388,
     0x0391,
     0x0399,
     0x03A1,
     0x03AA,
     0x03B2,
     0x03BB,
     0x03C3,
     0x03CC,
     0x03D4,
     0x03DD,
     0x03E8
};

static const unsigned short shdisp_main_bkl_lumi_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
     0x1D00,
     0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0006, 0x0009, 0x000C,
     0x000F, 0x0015, 0x001C, 0x002F, 0x006E, 0x009E, 0x00D0, 0x0104,
     0x0174, 0x01EA, 0x032A, 0x0485, 0x074F, 0x09EE, 0x0C92, 0x11E0,
     0x19D9, 0x2707, 0x3402, 0x4D42, 0x6588, 0x7CDA, 0x9343, 0x9F4A
};

static const unsigned short shdisp_main_bkl_min_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
     0x3D00,
     0x0006, 0x0006, 0x0007, 0x0007, 0x0007, 0x0008, 0x0009, 0x000B,
     0x000C, 0x000E, 0x0010, 0x0013, 0x001A, 0x001D, 0x001F, 0x0022,
     0x0025, 0x0028, 0x002D, 0x0031, 0x0036, 0x0039, 0x003C, 0x0040,
     0x0045, 0x0058, 0x0071, 0x00AD, 0x0103, 0x015A, 0x01B3, 0x0225
};

static const unsigned short shdisp_main_bkl_max_tbl[SHDISP_BKL_AUTO_PWM_TBL_NUM] = {
     0x3D00,
     0x002E, 0x0033, 0x0039, 0x003E, 0x0048, 0x0058, 0x007D, 0x009D,
     0x00BD, 0x00F3, 0x011E, 0x017B, 0x022A, 0x0281, 0x02CE, 0x030F,
     0x036A, 0x03C7, 0x0466, 0x04D5, 0x0577, 0x05ED, 0x064A, 0x06DE,
     0x0787, 0x0851, 0x08EA, 0x09DD, 0x0AA8, 0x0B62, 0x0C13, 0x0C4F
};

static const struct shdisp_bdic_led_color_index shdisp_triple_led_color_index_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM] = {
    {0, 0, 0,  0},
    {1, 0, 0,  1},
    {0, 1, 0,  2},
    {1, 1, 0,  3},
    {0, 0, 1,  4},
    {1, 0, 1,  5},
    {0, 1, 1,  6},
    {1, 1, 1,  7}
};

static const unsigned char shdisp_clrvari_index[SHDISP_COL_VARI_KIND] = {
    SHDISP_HANDSET_COLOR_BLACK,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_WHITE
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F }
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
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F }
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
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F }
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
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F }
    }
  }
};

static const struct shdisp_bdic_bkl_ado_tbl shdisp_bdic_bkl_ado_tbl[5] = {
    {   0,      0,  1786,      200},
    {   0,      8,  1286,      689},
    {   8,    145,   683,     6951},
    { 145,   2116,   460,    33156},
    {2116,  65536,   311,   498553}
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



#endif /* SHDISP_BL71Y6_DATA_DEFAULT_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
