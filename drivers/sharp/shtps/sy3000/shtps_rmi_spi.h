/* drivers/sharp/shtps/sy3000/shtps_rmi_spi.h
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
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
#ifndef __SHTPS_RMI_SPI_H__
#define __SHTPS_RMI_SPI_H__

#if 0 /** For build test */
	#undef CONFIG_SHTPS_SY3000_TM2945_001
	#undef CONFIG_SHTPS_SY3000_TM2955_001
	#undef CONFIG_SHTPS_SY3000_TM2974_001
	#undef CONFIG_SHTPS_SY3000_TM2979_001

]	#define CONFIG_SHTPS_SY3000_TM2945_001
#endif

#if defined( CONFIG_SHTPS_SY3000_TM2945_001 )
	#include "tm2945-001/shtps_cfg_tm2945-001.h"
#elif defined( CONFIG_SHTPS_SY3000_TM2955_001 )
	#include "tm2955-001/shtps_cfg_tm2955-001.h"
#elif defined( CONFIG_SHTPS_SY3000_TM2974_001 )
	#include "tm2974-001/shtps_cfg_tm2974-001.h"
#elif defined( CONFIG_SHTPS_SY3000_TM2979_001 )
	#include "tm2979-001/shtps_cfg_tm2979-001.h"
#else
	#include "tm2945-001/shtps_cfg_tm2945-001.h"
#endif

/* ===================================================================================
 * Debug
 */
#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_ERR_PRINT(...)	printk(KERN_ERR "[shtps] " __VA_ARGS__)
#else
	#define SHTPS_LOG_ERR_PRINT(...)
#endif /* defined( SHTPS_LOG_ERROR_ENABLE ) */

#if defined( SHTPS_LOG_DEBUG_ENABLE )
    #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
        #define	SHTPS_LOG_DBG_PRINT(...)					\
            if((gLogOutputEnable & 0x02) != 0){				\
                printk(KERN_DEBUG "[shtps] " __VA_ARGS__);	\
            }
        #define SHTPS_LOG_DEBUG(p)				\
            if((gLogOutputEnable & 0x02) != 0){	\
                p								\
            }
        #define SHTPS_LOG_FUNC_CALL()							\
            if((gLogOutputEnable & 0x02) != 0){					\
                printk(KERN_DEBUG "[shtps] %s()\n", __func__);	\
            }
        #define SHTPS_LOG_FUNC_CALL_INPARAM(param)						\
            if((gLogOutputEnable & 0x02) != 0){							\
                printk(KERN_DEBUG "[shtps]%s(%d)\n", __func__, param);	\
            }
    #else
        #define	SHTPS_LOG_DBG_PRINT(...)	printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
        #define SHTPS_LOG_DEBUG(p)	p
        #define SHTPS_LOG_FUNC_CALL()	printk(KERN_DEBUG "[shtps] %s()\n", __func__)
        #define SHTPS_LOG_FUNC_CALL_INPARAM(param)	\
                                        printk(KERN_DEBUG "[shtps]%s(%d)\n", __func__, param)
    #endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */

	#define SHTPS_LOG_ANALYSIS(...)		printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
#else
	#define	SHTPS_LOG_DBG_PRINT(...)
	#define SHTPS_LOG_DEBUG(p)
	#define SHTPS_LOG_FUNC_CALL()
	#define SHTPS_LOG_FUNC_CALL_INPARAM(param)
	#define SHTPS_LOG_ANALYSIS(...)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_EVENT_ENABLE ) && defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
    #define SHTPS_LOG_EVENT(p)				\
        if((gLogOutputEnable & 0x01) != 0){	\
            p								\
        }
#elif defined( SHTPS_LOG_EVENT_ENABLE )
	#define SHTPS_LOG_EVENT(p)	p
#else
	#define SHTPS_LOG_EVENT(p)
#endif /* defined( SHTPS_LOG_EVENT_ENABLE ) */

#define _log_msg_sync(id, fmt, ...)
#define _log_msg_send(id, fmt, ...)
#define _log_msg_recv(id, fmt, ...)

/* ===================================================================================
 * Common
 */
#define SPI_ERR_CHECK(check, label) \
	if((check)) goto label

#define SHTPS_POSTYPE_X (0)
#define SHTPS_POSTYPE_Y (1)

#define SHTPS_TOUCH_CANCEL_COORDINATES_X (0)
#define SHTPS_TOUCH_CANCEL_COORDINATES_Y (9999)


int shtps_get_logflag(void);
int shtps_read_touchevent_from_outside(void);
#endif /* __SHTPS_RMI_SPI_H__ */
