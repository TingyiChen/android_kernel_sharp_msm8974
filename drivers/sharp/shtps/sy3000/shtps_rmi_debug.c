/* drivers/sharp/shtps/sy3000/shtps_rmi_debug.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/time.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi_spi.h"
#include "shtps_rmi_debug.h"

#if defined( SHTPS_DEVELOP_MODE_ENABLE )

/* -----------------------------------------------------------------------------------
 */
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	SHTPS_PARAM_DEF( SHTPS_TOUCH_EMURATOR_LOG_ENABLE, 				0  );
	SHTPS_PARAM_DEF( SHTPS_TOUCH_EMURATOR_START_WAIT_TIME_SEC, 		5  );

	#define	SHTPS_TOUCH_EMU_LOG_V(...)					\
		if(SHTPS_TOUCH_EMURATOR_LOG_ENABLE != 0){			\
			printk(KERN_DEBUG "[shtps][emu] " __VA_ARGS__);	\
		}

	#define	SHTPS_TOUCH_EMU_LOG_DBG_PRINT(...)												\
		if(((shtps_get_logflag() & 0x02) != 0) || (SHTPS_TOUCH_EMURATOR_LOG_ENABLE != 0)){	\
			printk(KERN_DEBUG "[shtps][emu] " __VA_ARGS__);									\
		}

	#define SHTPS_TOUCH_EMU_LOG_FUNC_CALL()													\
		if(((shtps_get_logflag() & 0x02) != 0) || (SHTPS_TOUCH_EMURATOR_LOG_ENABLE != 0)){	\
			printk(KERN_DEBUG "[shtps][emu] %s()\n", __func__);								\
		}
	#define SHTPS_TOUCH_EMU_LOG_FUNC_CALL_INPARAM(param)									\
		if(((shtps_get_logflag() & 0x02) != 0) || (SHTPS_TOUCH_EMURATOR_LOG_ENABLE != 0)){	\
			printk(KERN_DEBUG "[shtps][emu] %s(%d)\n", __func__, param);					\
		}

	#define F12_SET_DATA_FINGERSTATE(F12_2D_DATA01, para)		(F12_2D_DATA01[0] = para)
	#define F12_SET_DATA_XPOS(F12_2D_DATA01, para)				{F12_2D_DATA01[1] = para & 0xff; F12_2D_DATA01[2] = para >> 8;}
	#define F12_SET_DATA_YPOS(F12_2D_DATA01, para)				{F12_2D_DATA01[3] = para & 0xff; F12_2D_DATA01[4] = para >> 8;}
	#define F12_SET_DATA_Z(F12_2D_DATA01, para)					(F12_2D_DATA01[5] = para)
	#define F12_SET_DATA_WX(F12_2D_DATA01, para)				(F12_2D_DATA01[6] = para)
	#define F12_SET_DATA_WY(F12_2D_DATA01, para)				(F12_2D_DATA01[7] = para)
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	enum{
		SHTPS_TOUCH_EMURATOR_STATE_DISABLE = 0,
		SHTPS_TOUCH_EMURATOR_STATE_WAITING,
		SHTPS_TOUCH_EMURATOR_STATE_RUNNING,
	};

	struct shtps_touch_emurator_list_elem{
		unsigned long	generation_time_ms;
		unsigned char	finger_id;
		unsigned short	finger_x;
		unsigned short	finger_y;
		unsigned char	finger_state;
		unsigned char	finger_wx;
		unsigned char	finger_wy;
		unsigned char	finger_z;
	};

	struct shtps_touch_emurator_info{
		int										state;
		struct timeval							emu_start_timeval;
		int										touch_list_buffering_size;
		int										touch_list_num;
		int										current_touch_index;
		struct shtps_touch_emurator_list_elem	*touch_list;
		struct delayed_work						touch_emu_read_touchevent_delayed_work;
		struct shtps_touch_info					fw_report_info;
	};
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */


/* -----------------------------------------------------------------------------------
 */
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	struct shtps_touch_emurator_info gShtps_touch_emu_info;
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
static int shtps_touch_emu_read_touchevent_timer_start(unsigned long delay_sec)
{
	SHTPS_TOUCH_EMU_LOG_FUNC_CALL_INPARAM((int)delay_sec);

	cancel_delayed_work(&gShtps_touch_emu_info.touch_emu_read_touchevent_delayed_work);
	schedule_delayed_work(&gShtps_touch_emu_info.touch_emu_read_touchevent_delayed_work, msecs_to_jiffies(delay_sec * 1000));

	return 0;
}

static int shtps_touch_emu_read_touchevent_timer_stop(void)
{
	SHTPS_TOUCH_EMU_LOG_FUNC_CALL();

	cancel_delayed_work(&gShtps_touch_emu_info.touch_emu_read_touchevent_delayed_work);

	return 0;
}

static void shtps_touch_emu_read_touchevent_delayed_work_function(struct work_struct *work)
{
	struct timeval current_timeval;
	unsigned long generation_time_us;
	unsigned long specified_time_us;
	unsigned long next_event_time_us;
	unsigned long next_event_time_total_us;
	int ret;
	
	int sleepcount;

	SHTPS_TOUCH_EMU_LOG_DBG_PRINT("shtps_touch_emu_read_touchevent_delayed_work_function() start\n");

	if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_WAITING){
		gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_RUNNING;

		do_gettimeofday(&gShtps_touch_emu_info.emu_start_timeval);
	}

	while(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_RUNNING){
		/* call read touchevent function */
		ret = shtps_read_touchevent_from_outside();
		if(ret < 0){
			gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_DISABLE;
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT("touch emurate end by state not active\n");
		}
		else if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_DISABLE){
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT("touch emurate end\n");
		}

		if(gShtps_touch_emu_info.state != SHTPS_TOUCH_EMURATOR_STATE_RUNNING){
			break;
		}

		generation_time_us = gShtps_touch_emu_info.touch_list[gShtps_touch_emu_info.current_touch_index].generation_time_ms * 1000;

		next_event_time_total_us = 0;

		sleepcount = 0;

		while(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_RUNNING){
		
			do_gettimeofday(&current_timeval);
			
			specified_time_us = ((current_timeval.tv_sec - gShtps_touch_emu_info.emu_start_timeval.tv_sec) * 1000000)
								 + ((current_timeval.tv_usec - gShtps_touch_emu_info.emu_start_timeval.tv_usec));

			if(generation_time_us > specified_time_us){
				next_event_time_us = generation_time_us - specified_time_us;
				if(next_event_time_us > (1 * 1000 * 1000)){
					next_event_time_us = (1 * 1000 * 1000);
				}
				next_event_time_total_us += next_event_time_us;

				usleep(next_event_time_us);

				sleepcount++;
			}
			else{
				SHTPS_TOUCH_EMU_LOG_V("generation_time_us=%lu, specified_time_us=%lu, next_event_time_total_us=%lu sleepcount=%d\n",
					generation_time_us,
					specified_time_us,
					next_event_time_total_us,
					sleepcount
				);
				break;
			}
		}
	}

	if(gShtps_touch_emu_info.touch_list != NULL){
		kfree(gShtps_touch_emu_info.touch_list);
		gShtps_touch_emu_info.touch_list = NULL;
	}
	gShtps_touch_emu_info.touch_list_num = 0;
	gShtps_touch_emu_info.touch_list_buffering_size = 0;
	gShtps_touch_emu_info.current_touch_index = 0;
	SHTPS_TOUCH_EMU_LOG_DBG_PRINT("shtps_touch_emu_read_touchevent_delayed_work_function() end\n");
}

int shtps_touch_emu_is_running(void)
{
	if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_RUNNING){
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(shtps_touch_emu_is_running);

int shtps_touch_emu_set_finger_info(u8 *buf, int bufsize)
{
	int				i;
	struct shtps_touch_emurator_list_elem	*touch_list;
	int				set_data_flg = 0;
	unsigned long	set_data_time = 0;
	struct timeval current_timeval;
	unsigned long specified_time_ms;
	struct fingers *fw_repot_finger_tmp;
	
	SHTPS_TOUCH_EMU_LOG_V("shtps_touch_emu_set_finger_info() start\n");
	for(i = 0; i < SHTPS_FINGER_MAX; i++){
		u8 *F12_2D_DATA01 = buf + 2 + (8 * i);
		fw_repot_finger_tmp = &gShtps_touch_emu_info.fw_report_info.fingers[i];
		F12_SET_DATA_FINGERSTATE(	F12_2D_DATA01,	fw_repot_finger_tmp->state);
		F12_SET_DATA_XPOS(			F12_2D_DATA01,	fw_repot_finger_tmp->x);
		F12_SET_DATA_YPOS(			F12_2D_DATA01,	fw_repot_finger_tmp->y);
		F12_SET_DATA_Z(				F12_2D_DATA01,	fw_repot_finger_tmp->z);
		F12_SET_DATA_WX(			F12_2D_DATA01,	fw_repot_finger_tmp->wx);
		F12_SET_DATA_WY(			F12_2D_DATA01,	fw_repot_finger_tmp->wy);
	}

	do_gettimeofday(&current_timeval);
	
	specified_time_ms = ((current_timeval.tv_sec - gShtps_touch_emu_info.emu_start_timeval.tv_sec) * 1000)
						 + ((current_timeval.tv_usec - gShtps_touch_emu_info.emu_start_timeval.tv_usec) / 1000);

	for(i = gShtps_touch_emu_info.current_touch_index; i < gShtps_touch_emu_info.touch_list_num; i++){
		touch_list = &gShtps_touch_emu_info.touch_list[i];
		if(touch_list->generation_time_ms <= specified_time_ms){
			if((set_data_flg != 0) && (set_data_time != touch_list->generation_time_ms)){
				break;
			}
			else{
				u8 *F12_2D_DATA01 = buf + 2 + (8 * touch_list->finger_id);
				F12_SET_DATA_FINGERSTATE(	F12_2D_DATA01,	touch_list->finger_state);
				F12_SET_DATA_XPOS(			F12_2D_DATA01,	touch_list->finger_x);
				F12_SET_DATA_YPOS(			F12_2D_DATA01,	touch_list->finger_y);
				F12_SET_DATA_Z(				F12_2D_DATA01,	touch_list->finger_z);
				F12_SET_DATA_WX(			F12_2D_DATA01,	touch_list->finger_wx);
				F12_SET_DATA_WY(			F12_2D_DATA01,	touch_list->finger_wy);

				fw_repot_finger_tmp = &gShtps_touch_emu_info.fw_report_info.fingers[touch_list->finger_id];
				fw_repot_finger_tmp->state	= touch_list->finger_state;
				fw_repot_finger_tmp->x		= touch_list->finger_x;
				fw_repot_finger_tmp->y		= touch_list->finger_y;
				fw_repot_finger_tmp->z		= touch_list->finger_z;
				fw_repot_finger_tmp->wx		= touch_list->finger_wx;
				fw_repot_finger_tmp->wy		= touch_list->finger_wy;

				set_data_flg = 1;

				set_data_time = touch_list->generation_time_ms;

				gShtps_touch_emu_info.current_touch_index++;
				SHTPS_TOUCH_EMU_LOG_V("touch emu set time=%lu [%d] state=%d x=%d y=%d wx=%d wy=%d z=%d\n",
										touch_list->generation_time_ms,
										touch_list->finger_id,
										touch_list->finger_state,
										touch_list->finger_x,
										touch_list->finger_y,
										touch_list->finger_wx,
										touch_list->finger_wy,
										touch_list->finger_z
										);
			}
		}
		else{
			break;
		}
	}

	if(gShtps_touch_emu_info.current_touch_index >= gShtps_touch_emu_info.touch_list_num){
		gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_DISABLE;
	}
	SHTPS_TOUCH_EMU_LOG_V("shtps_touch_emu_set_finger_info() end current index=(%d/%d)\n", gShtps_touch_emu_info.current_touch_index, gShtps_touch_emu_info.touch_list_num);
	return 0;
}
EXPORT_SYMBOL(shtps_touch_emu_set_finger_info);

static int shtps_get_arguments(
	char	*argStr,		/* [I/O] arguments strings (processed in function) */
	char	**argList,		/* [I/O] arguments pointer output buffer */
	int		argListMaxSize	/* [I/ ] arguments list size */
)
{
	int i;
	int argListNum = 0;
	int isParam;

	if((argStr == NULL) || (argList == NULL) || (argListMaxSize < 1)){
		return 0;
	}

	isParam = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if( (argStr[i] == '\0') ){
			break;
		}
		else if( (argStr[i] == '\n') || (argStr[i] == ',') || (argStr[i] == ' ') ){
			argStr[i] = '\0';
			if(isParam == 1){
				argListNum++;
			}
			isParam = 0;
			if(argListNum >= argListMaxSize){
				break;
			}
			continue;
		}
		else{
			if(isParam == 0){
				isParam = 1;
				argList[argListNum] = &argStr[i];
			}
		}
	}

	return argListNum;
}

static
int shtps_sysfs_start_touch_emurator(const char *buf, size_t count)
{
	u8 *data;

	if(NULL == buf || 0 == count){
		if(gShtps_touch_emu_info.touch_list == NULL){
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT("touch emu no list data! please set data.\n");
			return -EINVAL;
		}
	}
	else{
		data = (u8*)kmalloc(count, GFP_KERNEL);
		if(data == NULL){
			return -ENOMEM;
		}
		memcpy(data, buf, count);

		gShtps_touch_emu_info.touch_list_num = count / sizeof(struct shtps_touch_emurator_list_elem);
		if(gShtps_touch_emu_info.touch_list != NULL){
			kfree(gShtps_touch_emu_info.touch_list);
		}
		gShtps_touch_emu_info.touch_list = (struct shtps_touch_emurator_list_elem *)data;
	}

	gShtps_touch_emu_info.current_touch_index = 0;
	gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_WAITING;

	{
		int i;
		struct shtps_touch_emurator_list_elem *list_elem;
		for(i = 0; i < gShtps_touch_emu_info.touch_list_num; i++){
			list_elem = &gShtps_touch_emu_info.touch_list[i];
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT("time=%lu state[%d] Touch Info[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
					list_elem->generation_time_ms,
					list_elem->finger_state,
					list_elem->finger_id,
					list_elem->finger_x,
					list_elem->finger_y,
					list_elem->finger_wx,
					list_elem->finger_wy,
					list_elem->finger_z);
		}
	}
	SHTPS_TOUCH_EMU_LOG_DBG_PRINT("touch emu start list_num=%d\n",
							gShtps_touch_emu_info.touch_list_num);

	memset(&gShtps_touch_emu_info.fw_report_info, 0, sizeof(gShtps_touch_emu_info.fw_report_info));

	shtps_touch_emu_read_touchevent_timer_stop();
	shtps_touch_emu_read_touchevent_timer_start(SHTPS_TOUCH_EMURATOR_START_WAIT_TIME_SEC);

	return 0;
}

static
int shtps_sysfs_stop_touch_emurator(void)
{
	SHTPS_TOUCH_EMU_LOG_DBG_PRINT("stop touch emurator. state(%d => %d)\n", gShtps_touch_emu_info.state, SHTPS_TOUCH_EMURATOR_STATE_DISABLE);
	shtps_touch_emu_read_touchevent_timer_stop();
	gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_DISABLE;

	return 0;
}


static
ssize_t show_sysfs_emurator_ctrl(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "emurator state=%d\n", gShtps_touch_emu_info.state);
	
	return( strlen(buf) );
}

static
ssize_t store_sysfs_emurator_ctrl(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	char *argbuf;
	int argc;
	char *argv[10];

	
	argbuf = (char*)kmalloc(count, GFP_KERNEL);
	if(argbuf == NULL){
		SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "argbuf alloc error.\n" );
		return( count );
	}
	memcpy(argbuf, buf, count);

	argc = shtps_get_arguments( argbuf, argv, sizeof(argv)/sizeof(char *) );

	SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "store_sysfs_emurator_ctrl() call start command = %s\n", argv[0] );

	if(strcmp(argv[0], "start") == 0){
		if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_DISABLE){
			// call emurator start function
			ret = shtps_sysfs_start_touch_emurator(NULL, 0);
		}
		else{
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "emurator is already running.\n" );
		}
	}
	else if(strcmp(argv[0], "stop") == 0){
		shtps_sysfs_stop_touch_emurator();
	}
	else if(strcmp(argv[0], "clear") == 0){
		if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_RUNNING){
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "emurator is running. stop emurator and clear buffer.\n" );
			shtps_sysfs_stop_touch_emurator();
		}
		else{
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "clear emurator data buffer. %d byte(list num = %d) => 0\n",
								gShtps_touch_emu_info.touch_list_buffering_size,
								gShtps_touch_emu_info.touch_list_num );
			if(gShtps_touch_emu_info.touch_list != NULL){
				kfree(gShtps_touch_emu_info.touch_list);
				gShtps_touch_emu_info.touch_list = NULL;
			}
			gShtps_touch_emu_info.touch_list_num = 0;
			gShtps_touch_emu_info.touch_list_buffering_size = 0;
		}
	}
	else if(strcmp(argv[0], "start_wait_time") == 0){
		if(argc >= 2){
			SHTPS_TOUCH_EMURATOR_START_WAIT_TIME_SEC = simple_strtol(argv[1], NULL, 0);
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "set emurator start wait time = %d(sec)\n", SHTPS_TOUCH_EMURATOR_START_WAIT_TIME_SEC );
		}
		else{
			SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "few parameters!\n" );
		}
	}
	else{
		SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "command [%s] is not support!\n", argv[0] );
	}

	SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "store_sysfs_emurator_ctrl() call end\n" );
	kfree(argbuf);

	if(ret < 0){
		return ret;
	}
	return( count );
}


static
ssize_t show_sysfs_emurator_data(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "emurator_data_size = %d byte(list num = %d)\n",
							gShtps_touch_emu_info.touch_list_buffering_size,
							gShtps_touch_emu_info.touch_list_num );
	
	return( strlen(buf) );
}

static
ssize_t store_sysfs_emurator_data(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	char *data;
	int copy_size;

	SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "store_sysfs_emurator_data() call start count = %d\n", count );

	copy_size = count;

	if(gShtps_touch_emu_info.state == SHTPS_TOUCH_EMURATOR_STATE_DISABLE){
		// buffering data
		if(gShtps_touch_emu_info.touch_list == NULL){
			gShtps_touch_emu_info.touch_list_num = 0;
			gShtps_touch_emu_info.touch_list_buffering_size = 0;

			data = (char *)kmalloc(copy_size, GFP_KERNEL);
			if(data == NULL){
				return -ENOMEM;
			}
			memcpy(data, buf, copy_size);
		}
		else{
			data = (char *)kmalloc(gShtps_touch_emu_info.touch_list_buffering_size + copy_size, GFP_KERNEL);
			if(data == NULL){
				return -ENOMEM;
			}
			memcpy(data, gShtps_touch_emu_info.touch_list, gShtps_touch_emu_info.touch_list_buffering_size);
			memcpy(data + gShtps_touch_emu_info.touch_list_buffering_size, buf, copy_size);
			kfree(gShtps_touch_emu_info.touch_list);
		}
		gShtps_touch_emu_info.touch_list = (struct shtps_touch_emurator_list_elem *)data;
		gShtps_touch_emu_info.touch_list_buffering_size += copy_size;
		gShtps_touch_emu_info.touch_list_num = gShtps_touch_emu_info.touch_list_buffering_size / sizeof(struct shtps_touch_emurator_list_elem);

		SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "store_sysfs_emurator_data() call end current size = %d byte(list num = %d)\n",
								gShtps_touch_emu_info.touch_list_buffering_size,
								gShtps_touch_emu_info.touch_list_num );
	}
	else{
		SHTPS_TOUCH_EMU_LOG_DBG_PRINT( "emurator is running. not set data.\n" );
	}

	return( copy_size );
}

struct shtps_debug_param_regist_info emurator_sysfs_regist_info[] = {
	{"emurator", SHTPS_DEBUG_PARAM_NAME_LOG, &SHTPS_TOUCH_EMURATOR_LOG_ENABLE, NULL, NULL},
	{"emurator", "ctrl", NULL, show_sysfs_emurator_ctrl, store_sysfs_emurator_ctrl},
	{"emurator", "data", NULL, show_sysfs_emurator_data, store_sysfs_emurator_data},
	{"emurator", "start_wait_time", &SHTPS_TOUCH_EMURATOR_START_WAIT_TIME_SEC, NULL, NULL},

	{NULL, NULL, NULL, NULL, NULL}
};
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */


/* -----------------------------------------------------------------------------------
 */
#define SHTPS_DEBUG_PARAM_DIR_NAME_ROOT		"debug"
#define SHTPS_DEBUG_PARAM_DIR_NAME_PARAM	"parameter"

struct shtps_debug_info{
	struct list_head	kobj_list;
	struct list_head	attr_list;

};

static struct shtps_debug_info *gDegbuInfo;


struct shtps_debug_kobj_list_info{
	struct list_head	kobj_list;
	struct kobject		*parent_kobj_p;
	struct kobject		*kobj_p;
};

struct shtps_debug_attr_list_info{
	struct list_head		attr_list;
	struct kobject			*parent_kobj_p;
	struct kobj_attribute	*kobj_attr_p;
	struct kobj_attribute	kobj_attr;
	int						*param_p;
};

static struct kobject* shtps_debug_search_kobj(struct kobject *parent_kobj_p, const char *search_name)
{
	struct list_head *list_p;
	struct shtps_debug_kobj_list_info *item_info_p;
	struct kobject *find_kobj_p = NULL;

	list_for_each(list_p, &(gDegbuInfo->kobj_list)){
		item_info_p = list_entry(list_p, struct shtps_debug_kobj_list_info, kobj_list);

		if( strcmp(kobject_name(item_info_p->kobj_p), search_name) == 0 ){
			if(parent_kobj_p == NULL){
				find_kobj_p = item_info_p->kobj_p;
				break;
			}else{
				if(item_info_p->parent_kobj_p == parent_kobj_p){
					find_kobj_p = item_info_p->kobj_p;
					break;
				}
			}
		}
	}

	return find_kobj_p;
}

static struct shtps_debug_attr_list_info* shtps_debug_search_attr(struct kobject *parent_kobj_p, const char *search_name)
{
	struct list_head *list_p;
	struct shtps_debug_attr_list_info *item_info_p;
	struct shtps_debug_attr_list_info *find_attr_p = NULL;

	list_for_each(list_p, &(gDegbuInfo->attr_list)){
		item_info_p = list_entry(list_p, struct shtps_debug_attr_list_info, attr_list);

		if( strcmp(item_info_p->kobj_attr.attr.name, search_name) == 0 ){
			if(parent_kobj_p == NULL){
				find_attr_p = item_info_p;
				break;
			}else{
				if(item_info_p->parent_kobj_p == parent_kobj_p){
					find_attr_p = item_info_p;
					break;
				}
			}
		}
	}

	return find_attr_p;
}

static ssize_t shtps_debug_common_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t count;
	struct shtps_debug_attr_list_info *find_attr_p = NULL;

	find_attr_p = shtps_debug_search_attr(kobj, attr->attr.name);

	if(find_attr_p == NULL){
		count = snprintf(buf, PAGE_SIZE, "-\n");
	}else if(find_attr_p->param_p == NULL){
		count = snprintf(buf, PAGE_SIZE, "-\n");
	}else{
		count = snprintf(buf, PAGE_SIZE, "%d\n", *(find_attr_p->param_p));
	}

	return count;
}

static ssize_t shtps_debug_common_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_debug_attr_list_info *find_attr_p = NULL;
	int val;

	sscanf(buf,"%d", &val);

	find_attr_p = shtps_debug_search_attr(kobj, attr->attr.name);

	if( (find_attr_p != NULL) && (find_attr_p->param_p != NULL) ){
		*(find_attr_p->param_p) = val;
	}

	return count;
}

static struct kobject* shtps_debug_create_kobj(struct kobject *parent_kobj_p, const char *name)
{
	struct shtps_debug_kobj_list_info *list_info_p = NULL;
	struct kobject *create_kobj_p = NULL;

	list_info_p = kzalloc(sizeof(struct shtps_debug_kobj_list_info), GFP_KERNEL);
	if(list_info_p != NULL){
		create_kobj_p = kobject_create_and_add(name, parent_kobj_p);
		if(create_kobj_p != NULL){
			list_info_p->kobj_p = create_kobj_p;
			list_info_p->parent_kobj_p = parent_kobj_p;
			list_add_tail(&(list_info_p->kobj_list), &(gDegbuInfo->kobj_list));
		}
		else{
			kfree(list_info_p);
		}
	}

	return create_kobj_p;
}

static void shtps_debug_delete_kobj_all(void)
{
	struct shtps_debug_kobj_list_info *item_info_p;

	while( list_empty( &(gDegbuInfo->kobj_list) ) == 0 ){
		item_info_p = list_entry(gDegbuInfo->kobj_list.next, struct shtps_debug_kobj_list_info, kobj_list);

		if(item_info_p != NULL){
			kobject_put(item_info_p->kobj_p);
			list_del_init(&(item_info_p->kobj_list));
			kfree(item_info_p);
		}
	}

	list_del_init(&(gDegbuInfo->kobj_list));
}

static int shtps_debug_create_attr(struct kobject *parent_kobj_p, struct kobj_attribute attr, int *param_p)
{
	struct shtps_debug_attr_list_info *list_info_p = NULL;
	int rc;

	list_info_p = kzalloc(sizeof(struct shtps_debug_attr_list_info), GFP_KERNEL);
	if(list_info_p != NULL){
		list_info_p->parent_kobj_p = parent_kobj_p;
		list_info_p->kobj_attr_p = &(list_info_p->kobj_attr);
		list_info_p->kobj_attr = attr;
		if(param_p == NULL){
			list_info_p->param_p = NULL;
		}else{
			list_info_p->param_p = param_p;
		}
		rc = sysfs_create_file(parent_kobj_p, &(list_info_p->kobj_attr.attr));
		if(rc == 0){
			list_add_tail(&(list_info_p->attr_list), &(gDegbuInfo->attr_list));
		}else{
			kfree(list_info_p);
		}
	}

	return 0;
}

static void shtps_debug_delete_attr_all(void)
{
	struct shtps_debug_attr_list_info *item_info_p;

	while( list_empty( &(gDegbuInfo->attr_list) ) == 0 ){
		item_info_p = list_entry(gDegbuInfo->attr_list.next, struct shtps_debug_attr_list_info, attr_list);

		if(item_info_p != NULL){
			sysfs_remove_file(item_info_p->parent_kobj_p, &(item_info_p->kobj_attr_p->attr));
			list_del_init(&(item_info_p->attr_list));
			kfree(item_info_p);
		}
	}

	list_del_init(&(gDegbuInfo->attr_list));
}

static int shtps_debug_param_obj_init(struct kobject *shtps_root_kobj)
{
	struct kobject *kobj_p;

	gDegbuInfo = (struct shtps_debug_info*)kmalloc(sizeof(struct shtps_debug_info), GFP_KERNEL);
	if(gDegbuInfo == NULL){
		return -1;
	}

	INIT_LIST_HEAD(&(gDegbuInfo->kobj_list));
	INIT_LIST_HEAD(&(gDegbuInfo->attr_list));

	if(shtps_root_kobj == NULL){
		return -1;
	}

	kobj_p = shtps_debug_create_kobj(shtps_root_kobj, SHTPS_DEBUG_PARAM_DIR_NAME_ROOT);
	if(kobj_p == NULL){
		return -1;
	}

	return 0;
}

static void shtps_debug_param_obj_deinit(void)
{
	shtps_debug_delete_attr_all();
	shtps_debug_delete_kobj_all();
	if(gDegbuInfo != NULL){
		kfree(gDegbuInfo);
		gDegbuInfo = NULL;
	}
}

int shtps_debug_param_add(struct shtps_debug_param_regist_info *info_p)
{
	int i;
	struct kobject *shtps_root_kobj_p = NULL;

	shtps_root_kobj_p = shtps_debug_search_kobj(NULL, SHTPS_DEBUG_PARAM_DIR_NAME_ROOT);

	if(shtps_root_kobj_p == NULL){
		return -1;
	}

	for(i = 0; info_p[i].param_name != NULL; i++)
	{
		struct kobject *parent_kobj_p = NULL;
		struct kobject *param_kobj_p = NULL;
		struct kobj_attribute kobj_attr;

		parent_kobj_p = shtps_debug_search_kobj(shtps_root_kobj_p, info_p[i].parent_name);
		if(parent_kobj_p == NULL){
			parent_kobj_p = shtps_debug_create_kobj(shtps_root_kobj_p, info_p[i].parent_name);
			if(parent_kobj_p == NULL){
				return -1;
			}
		}

		param_kobj_p = shtps_debug_search_kobj(parent_kobj_p, SHTPS_DEBUG_PARAM_DIR_NAME_PARAM);
		if(param_kobj_p == NULL){
			param_kobj_p = shtps_debug_create_kobj(parent_kobj_p, SHTPS_DEBUG_PARAM_DIR_NAME_PARAM);
			if(param_kobj_p == NULL){
				return -1;
			}
		}

		kobj_attr.attr.name = info_p[i].param_name;
//		kobj_attr.attr.mode = (S_IRUGO | S_IWUGO);
		kobj_attr.attr.mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
		if(info_p[i].show == NULL){
			kobj_attr.show = shtps_debug_common_show;
		}else{
			kobj_attr.show = info_p[i].show;
		}

		if(info_p[i].store == NULL){
			kobj_attr.store = shtps_debug_common_store;
		}else{
			kobj_attr.store = info_p[i].store;
		}

		if( (strcmp(info_p[i].param_name, SHTPS_DEBUG_PARAM_NAME_ONOFF) == 0) ||
			(strcmp(info_p[i].param_name, SHTPS_DEBUG_PARAM_NAME_LOG) == 0) )
		{
			shtps_debug_create_attr(parent_kobj_p, kobj_attr, info_p[i].param_p);
		}else{
			shtps_debug_create_attr(param_kobj_p, kobj_attr, info_p[i].param_p);
		}
	}

	return 0;
}
/* -----------------------------------------------------------------------------------
 */
int shtps_debug_init(struct shtps_debug_init_param *param)
{
	if(!param){
		return -1;
	}

	/*  */
	shtps_debug_param_obj_init(param->shtps_root_kobj);

	#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
		gShtps_touch_emu_info.state = SHTPS_TOUCH_EMURATOR_STATE_DISABLE;
		INIT_DELAYED_WORK(&gShtps_touch_emu_info.touch_emu_read_touchevent_delayed_work, shtps_touch_emu_read_touchevent_delayed_work_function);
		shtps_debug_param_add(emurator_sysfs_regist_info);
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */

	return 0;
}
EXPORT_SYMBOL(shtps_debug_init);

void shtps_debug_deinit(void)
{
	#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
		if(gShtps_touch_emu_info.touch_list != NULL){
			kfree(gShtps_touch_emu_info.touch_list);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */

	/*  */
	shtps_debug_param_obj_deinit();
}
EXPORT_SYMBOL(shtps_debug_deinit);

void shtps_debug_sleep_enter(void)
{
	#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
		if(shtps_touch_emu_is_running() != 0){
			shtps_sysfs_stop_touch_emurator();
		}
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
}
EXPORT_SYMBOL(shtps_debug_sleep_enter);

#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

MODULE_DESCRIPTION("SHARP TOUCHPANEL DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
