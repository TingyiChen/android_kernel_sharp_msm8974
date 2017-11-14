/* drivers/sharp/shcamled/shcamled.c
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/leds-pm8xxx.h>
#include <linux/kthread.h>
#include "linux/leds.h"
#include "linux/qpnp/qpnp-api.h"

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shcamled_pmic_set_torch_led_1_current(unsigned mA);
int shcamled_pmic_flash_prepare(void);
static int shcamled_pmic_set_torch_led_2_current(unsigned mA);
static ssize_t shcamled_torch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static int shcamled_torch_probe(struct platform_device *pdev);
static int shcamled_torch_remove(struct platform_device *pdev);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define FALSE 0
#define TRUE 1

#define SHCAMLED_TORCH_DRV_NAME	"shcamled_torch"
#define SHCAMLED_TORCH_PERMISSION	(S_IRUSR | S_IWUSR) | (S_IRGRP | S_IWGRP)

//#define SHCAMLED_ENABLE_DEBUG

#ifdef SHCAMLED_ENABLE_DEBUG
#define SHCAMLED_INFO(fmt, args...) pr_err(fmt, ##args)
#define SHCAMLED_TRACE(fmt, args...) pr_err(fmt, ##args)
#define SHCAMLED_ERROR(fmt, args...) pr_err(fmt, ##args)
#else
#define SHCAMLED_INFO(x...)	do {} while(0)
#define SHCAMLED_TRACE(x...)	do {} while(0)
#define SHCAMLED_ERROR(fmt, args...) pr_err(fmt, ##args)
#endif

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct mutex shcamled_mut = __MUTEX_INITIALIZER(shcamled_mut);
struct task_struct *p_flash_off_thread = NULL;
struct task_struct *p_flash_on_thread = NULL;
wait_queue_head_t shcamled_msg_off_wait;
wait_queue_head_t shcamled_msg_on_wait;
static int flash_off_thread_active = 0;
static int flash_on_thread_active = 0;
static atomic_t flash_on_prepare;
struct completion flash_on_complete;

static struct platform_device shcamled_torch_dev = {
	.name   = "shcamled_torch",
};

static struct platform_driver shcamled_torch_driver = {
	.probe = shcamled_torch_probe,
	.remove = shcamled_torch_remove,
	.shutdown = NULL,
	.driver = {
		.name = SHCAMLED_TORCH_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static DEVICE_ATTR(shcamled_torch, SHCAMLED_TORCH_PERMISSION, NULL, shcamled_torch_store);

/* LED trigger (declare cam_led_x_trigger) */
DEFINE_LED_TRIGGER(cam_torch_flash_0_trigger);
DEFINE_LED_TRIGGER(cam_torch_flash_1_trigger);
DEFINE_LED_TRIGGER(cam_torch_led_1_trigger);
static int flash_mode=0;
static int torch_mode=0;

/* mutex */
static DEFINE_MUTEX(shcamled_torch_mut);
static DEFINE_MUTEX(shcamled_red_mut);

/* user info */
static int shcamled_use_torch_led_cam = FALSE;
static int shcamled_use_torch_led_comm = FALSE;

static int camstatus;
module_param_named(
	camstatus, camstatus, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/* ------------------------------------------------------------------------- */
/* CODE                                                                      */
/* ------------------------------------------------------------------------- */
static int flash_off_thread(void * arg)
{
	SHCAMLED_TRACE("%s start\n", __FUNCTION__);
	while(!kthread_should_stop()){
		SHCAMLED_TRACE("%s %d wait_event_interruptible\n", __FUNCTION__, __LINE__);
		wait_event_interruptible(shcamled_msg_off_wait, kthread_should_stop() || flash_off_thread_active);

		if(flash_off_thread_active){
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(false) start\n", __FUNCTION__, __LINE__);
//			qpnp_flash_control_enable(false);
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(false) end\n", __FUNCTION__, __LINE__);
			flash_off_thread_active = 0;
			atomic_set(&flash_on_prepare, 0);
			mutex_unlock(&shcamled_mut);
		}
	}

	SHCAMLED_TRACE("%s end\n", __FUNCTION__);
	return 0;
}

static int flash_on_thread(void * arg)
{
	SHCAMLED_TRACE("%s start\n", __FUNCTION__);
	while(!kthread_should_stop()){
		SHCAMLED_TRACE("%s %d wait_event_interruptible\n", __FUNCTION__, __LINE__);
		wait_event_interruptible(shcamled_msg_on_wait, kthread_should_stop() || flash_on_thread_active);

		if(flash_on_thread_active){
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) start\n", __FUNCTION__, __LINE__);
//			qpnp_flash_control_enable(true);
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) end\n", __FUNCTION__, __LINE__);
			complete(&flash_on_complete);
			flash_on_thread_active = 0;
			mutex_unlock(&shcamled_mut);
		}
	}

	SHCAMLED_TRACE("%s end\n", __FUNCTION__);
	return 0;
}

int shcamled_pmic_flash_prepare(void)
{
	SHCAMLED_TRACE("%s start\n", __FUNCTION__);
	mutex_lock(&shcamled_mut);
	atomic_set(&flash_on_prepare, 1);
	init_completion(&flash_on_complete);
	flash_on_thread_active = 1;
	wake_up_interruptible(&shcamled_msg_on_wait);

	SHCAMLED_TRACE("%s end\n", __FUNCTION__);
	return 0;
}
/* ------------------------------------------------------------------------- */
/* shcamled_pmic_set_torch_led_1_current                                     */
/* ------------------------------------------------------------------------- */
int shcamled_pmic_set_torch_led_1_current(unsigned mA)
{
	unsigned int led_current = 0;
	
	mutex_lock(&shcamled_torch_mut);

	SHCAMLED_TRACE("%s in mA:%d ucam:%d ucom:%d\n", __FUNCTION__, mA,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	if(mA == 0){
		if((shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) ||
		   (shcamled_use_torch_led_cam == TRUE && shcamled_use_torch_led_comm == FALSE)) {
			/* request turning off */
			if(flash_mode != 0){
				led_trigger_event(cam_torch_flash_0_trigger, 0);
				led_trigger_event(cam_torch_flash_1_trigger, 0);
				SHCAMLED_TRACE("%s %d wake_up_interruptible start\n", __FUNCTION__, __LINE__);
				mutex_lock(&shcamled_mut);
//				qpnp_flash_control_enable(false);
				if(atomic_read(&flash_on_prepare) == 1){
					wait_for_completion(&flash_on_complete);
				}
				flash_off_thread_active = 1;
				wake_up_interruptible(&shcamled_msg_off_wait);
				SHCAMLED_TRACE("%s %d wake_up_interruptible end\n", __FUNCTION__, __LINE__);

				flash_mode = 0;
			}
			if(torch_mode != 0){
				led_trigger_event(cam_torch_led_1_trigger, 0);
				torch_mode = 0;
			}
			shcamled_use_torch_led_cam = FALSE;
		}
	} else {
		/* calucurate mA for leds_qpnq */
		led_current = mA / 25 /* 12.5 * 2 */;
		if(led_current > 1){
			led_current = led_current - 1;
		} else {
			led_current = 0;
		}
/* for pre-flash */
//		if(mA > 100){
		if(mA > 250){
			led_current = (led_current * 1000 + 0x4E) / 0x4F;
		} else {
			led_current = (led_current * 200 + 0x0E) / 0x0F;
		}
		if(led_current < 1){
			led_current = 1;
		}
		SHCAMLED_TRACE("%s led_current=%d\n", __FUNCTION__, led_current);
		
		/* request turning on */
/* for pre-flash */
//		if(mA > 100){
		if(mA > 250){
			if(torch_mode != 0){
				led_trigger_event(cam_torch_led_1_trigger, 0);
				torch_mode = 0;
			}
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) start\n", __FUNCTION__, __LINE__);
//			qpnp_flash_control_enable(true);
			if(atomic_read(&flash_on_prepare) == 0){
				shcamled_pmic_flash_prepare();
			}
			wait_for_completion(&flash_on_complete);
			atomic_set(&flash_on_prepare, 0);
			SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) end\n", __FUNCTION__, __LINE__);
			led_trigger_event(cam_torch_flash_0_trigger, led_current);
			led_trigger_event(cam_torch_flash_1_trigger, led_current);
			flash_mode = 1;
		} else {
			if(flash_mode != 0){
				led_trigger_event(cam_torch_flash_0_trigger, 0);
				led_trigger_event(cam_torch_flash_1_trigger, 0);
				SHCAMLED_TRACE("%s %d wake_up_interruptible start\n", __FUNCTION__, __LINE__);
				mutex_lock(&shcamled_mut);
//				qpnp_flash_control_enable(false);
				if(atomic_read(&flash_on_prepare) == 1){
					wait_for_completion(&flash_on_complete);
				}
				flash_off_thread_active = 1;
				wake_up_interruptible(&shcamled_msg_off_wait);
				SHCAMLED_TRACE("%s %d wake_up_interruptible end\n", __FUNCTION__, __LINE__);
				flash_mode = 0;
			}
			led_trigger_event(cam_torch_led_1_trigger, led_current);
			torch_mode = 1;
		}
		
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == FALSE) {
			shcamled_use_torch_led_cam = TRUE;
		} else if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) {
			shcamled_use_torch_led_comm = FALSE;
			shcamled_use_torch_led_cam = TRUE;
		}
	}

	SHCAMLED_TRACE("%s done ucam:%d ucom:%d\n", __FUNCTION__,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	mutex_unlock(&shcamled_torch_mut);

	return 0;
}
EXPORT_SYMBOL(shcamled_pmic_set_torch_led_1_current);

/* ------------------------------------------------------------------------- */
/* shcamled_pmic_set_torch_led_2_current                                     */
/* ------------------------------------------------------------------------- */
static int shcamled_pmic_set_torch_led_2_current(unsigned mA)
{
	unsigned int led_current = 0;
	
	mutex_lock(&shcamled_torch_mut);

	SHCAMLED_TRACE("%s in mA:%d ucam:%d ucom:%d\n", __FUNCTION__, mA,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	if(mA == 0) {
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) {
			/* request turning off */
			if(flash_mode != 0){
				led_trigger_event(cam_torch_flash_0_trigger, 0);
				led_trigger_event(cam_torch_flash_1_trigger, 0);
				SHCAMLED_TRACE("%s %d wake_up_interruptible start\n", __FUNCTION__, __LINE__);
				mutex_lock(&shcamled_mut);
//				qpnp_flash_control_enable(false);
				if(atomic_read(&flash_on_prepare) == 1){
					wait_for_completion(&flash_on_complete);
				}
				flash_off_thread_active = 1;
				wake_up_interruptible(&shcamled_msg_off_wait);
				SHCAMLED_TRACE("%s %d wake_up_interruptible end\n", __FUNCTION__, __LINE__);
				flash_mode = 0;
			}
			if(torch_mode != 0){
				led_trigger_event(cam_torch_led_1_trigger, 0);
				torch_mode = 0;
			}
			shcamled_use_torch_led_comm = FALSE;
		}
	} else {
		/* calucurate mA for leds_qpnq */
		led_current = mA / 25 /* 12.5 * 2 */;
		if(led_current > 1){
			led_current = led_current - 1;
		} else {
			led_current = 0;
		}
		if(mA > 100){
			led_current = (led_current * 1000 + 0x4E) / 0x4F;
		} else {
			led_current = (led_current * 200 + 0x0E) / 0x0F;
		}
		if(led_current < 1){
			led_current = 1;
		}
		SHCAMLED_TRACE("%s led_current=%d\n", __FUNCTION__, led_current);
		
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == FALSE){
			/* request turning on */
			if(mA > 100){
				if(torch_mode != 0){
					led_trigger_event(cam_torch_led_1_trigger, 0);
					torch_mode = 0;
				}
				SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) start\n", __FUNCTION__, __LINE__);
//				qpnp_flash_control_enable(true);
				if(atomic_read(&flash_on_prepare) == 0){
					shcamled_pmic_flash_prepare();
				}
				wait_for_completion(&flash_on_complete);
				atomic_set(&flash_on_prepare, 0);
				SHCAMLED_TRACE("%s %d qpnp_flash_control_enable(true) end\n", __FUNCTION__, __LINE__);
				led_trigger_event(cam_torch_flash_0_trigger, led_current);
				led_trigger_event(cam_torch_flash_1_trigger, led_current);
				flash_mode = 1;
			} else {
				if(flash_mode != 0){
					led_trigger_event(cam_torch_flash_0_trigger, 0);
					led_trigger_event(cam_torch_flash_1_trigger, 0);
					SHCAMLED_TRACE("%s %d wake_up_interruptible start\n", __FUNCTION__, __LINE__);
					mutex_lock(&shcamled_mut);
//					qpnp_flash_control_enable(false);
					if(atomic_read(&flash_on_prepare) == 1){
						wait_for_completion(&flash_on_complete);
					}
					flash_off_thread_active = 1;
					wake_up_interruptible(&shcamled_msg_off_wait);
					SHCAMLED_TRACE("%s %d wake_up_interruptible end\n", __FUNCTION__, __LINE__);
					flash_mode = 0;
				}
				led_trigger_event(cam_torch_led_1_trigger, led_current);
				torch_mode = 1;
			}
			shcamled_use_torch_led_comm = TRUE;
		}
	}

	SHCAMLED_TRACE("%s done ucam:%d ucom:%d\n", __FUNCTION__,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	mutex_unlock(&shcamled_torch_mut);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_store                                                      */
/* ------------------------------------------------------------------------- */
static ssize_t shcamled_torch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	ssize_t ret = -EINVAL;
	int proc_ret;
	char *after;
	unsigned long val = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	/* count written bytes size and check size */
	if (isspace(*after))
		count++;

	if(count == size) {
		ret = count;
		proc_ret = shcamled_pmic_set_torch_led_2_current(val);
		if(proc_ret) {
			ret = -EFAULT;
			SHCAMLED_ERROR("%s failed ret:%d\n", __FUNCTION__, proc_ret);
		}
		SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, proc_ret);
	}

	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_prove                                                      */
/* ------------------------------------------------------------------------- */
static int shcamled_torch_probe(struct platform_device *pdev)
{

	int ret;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	led_trigger_register_simple("flash0_trigger", &cam_torch_flash_0_trigger);
	led_trigger_register_simple("flash1_trigger", &cam_torch_flash_1_trigger);
	led_trigger_register_simple("torch_trigger", &cam_torch_led_1_trigger);

	ret = device_create_file(&pdev->dev, &dev_attr_shcamled_torch);
	if (ret) {
		SHCAMLED_ERROR("%s failed create file \n", __FUNCTION__);
		return ret;
	}
	
	init_waitqueue_head(&shcamled_msg_off_wait);
	init_waitqueue_head(&shcamled_msg_on_wait);
	
	p_flash_off_thread = kthread_create(flash_off_thread, &shcamled_mut, "flash_off_thread");
	if(p_flash_off_thread != NULL){
		wake_up_process(p_flash_off_thread);
		SHCAMLED_TRACE("%s %d wake_up_process\n", __FUNCTION__, __LINE__);
	} else {
		SHCAMLED_TRACE("%s %d p_flash_off_thread =%p\n", __FUNCTION__, __LINE__, p_flash_off_thread);
	}
	p_flash_on_thread = kthread_create(flash_on_thread, &shcamled_mut, "flash_on_thread");
	if(p_flash_on_thread != NULL){
		wake_up_process(p_flash_on_thread);
		SHCAMLED_TRACE("%s %d wake_up_process\n", __FUNCTION__, __LINE__);
	} else {
		SHCAMLED_TRACE("%s %d p_flash_on_thread =%p\n", __FUNCTION__, __LINE__, p_flash_on_thread);
	}
	atomic_set(&flash_on_prepare, 0);
	
	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_remove                                                     */
/* ------------------------------------------------------------------------- */
static int shcamled_torch_remove(struct platform_device *pdev)
{

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	led_trigger_unregister_simple(cam_torch_flash_0_trigger);
	led_trigger_unregister_simple(cam_torch_flash_1_trigger);
	led_trigger_unregister_simple(cam_torch_led_1_trigger);

	device_remove_file(&pdev->dev, &dev_attr_shcamled_torch);

	if(p_flash_off_thread != NULL){
		kthread_stop(p_flash_off_thread);
	}
	if(p_flash_on_thread != NULL){
		kthread_stop(p_flash_on_thread);
	}
	
	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_driver_init                                                      */
/* ------------------------------------------------------------------------- */
static int __init shcamled_torch_driver_init(void)
{
	int ret;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	/* torch 1. register platform-driver for device */
	ret = platform_driver_register(&shcamled_torch_driver);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* torch 2. register platform-device */
	ret = platform_device_register(&shcamled_torch_dev);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		platform_driver_unregister(&shcamled_torch_driver);
		return ret;
	}

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return 0;
}
module_init(shcamled_torch_driver_init);

/* ------------------------------------------------------------------------- */
/* shcamled_driver_exit                                                      */
/* ------------------------------------------------------------------------- */
static void __exit shcamled_torch_driver_exit(void)
{
	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	platform_driver_unregister(&shcamled_torch_driver);
	platform_device_unregister(&shcamled_torch_dev);

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
}
module_exit(shcamled_torch_driver_exit);

MODULE_DESCRIPTION("SHARP CAMERA TORCH LED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
