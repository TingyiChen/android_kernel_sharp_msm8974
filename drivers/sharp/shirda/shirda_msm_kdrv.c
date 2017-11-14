/* drivers/sharp/shirda/shirda_msm_kdrv.c (sharp IrDA driver)
 *
 * Copyright (C) 2011 - 2013 SHARP CORPORATION All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <asm/io.h>

#include <linux/gpio.h>

#include "sharp/irda_kdrv_api.h"

#define SHIRDA_DRIVER_NAME	SHIRDA_DEVFILE_NAME
#include "shirda_kdrv.h"



#define SHIRDA_KERNEL_DRIVER_VERSION	"1.20.00"
#define SHIRDA_KERNEL_DRIVER_NAME	SHIRDA_DEVFILE_NAME
#define SHIRDA_KERNEL_DRIVER_NAME2	SHIRDA_DEVFILE_NAME2
#define SHIRDA_KERNEL_DRIVER_NAME3	SHIRDA_DEVFILE_NAME3

#define SHIRDA_CORE_CLK_FREQ	(7372800)

#define SHIRDA_GPIO_TXD		(45)
#define SHIRDA_GPIO_RXD		(46)
#define SHIRDA_GPIO_SD		(47)
#define SHIRDA_GPIO_TX_FUNC	(2)
#define SHIRDA_GPIO_RX_FUNC	(2)
#define SHIRDA_GPIO_SD_FUNC	(0)

#define	SHIRDA_TXD_PULL		GPIO_CFG_NO_PULL
#define	SHIRDA_TXD_STRENGTH	GPIO_CFG_2MA

#define	SHIRDA_RXD_PULL		GPIO_CFG_PULL_UP
#define	SHIRDA_RXD_STRENGTH	GPIO_CFG_2MA

#define	SHIRDA_SD_PULL		GPIO_CFG_NO_PULL
#define	SHIRDA_SD_STRENGTH	GPIO_CFG_2MA

#define SHIRDA_SD_SHUTDOWN	(1)
#define SHIRDA_SD_ACTIVE	(0)

#define SHIRDA_WAIT_MODESEQ	(200*1000)
#define SHIRDA_MODESEQ_SDACTIME	(1)
#define SHIRDA_MODESEQ_WAITTXD	(400*1000)

#define SHIRDA_LEDRCMODE_WAIT	(1*1000)
#define	SHIRDA_LEDRCMODE_ST1	(1)
#define SHIRDA_LEDRCMODE_ST2	(1)
#define SHIRDA_LEDRCMODE_ST3	(10)
#define SHIRDA_LEDRCMODE_ST4	(10*1000)


#define GSBIREG_BASE		(0xF995E000)
#define UART_DM_REGISTER	(0x0)
#define UART_DM_IRDA		(0x00B8)

#define SHIRDA_UART_DM_IRDA    (GSBIREG_BASE + UART_DM_REGISTER + UART_DM_IRDA)


#define MEDIUM_RATE_EN		(0x0010)
#define IRDA_LOOPBACK		(0x0008)
#define INVERT_IRDA_TX		(0x0004)
#define INVERT_IRDA_RX		(0x0002)
#define IRDA_EN			(0x0001)

#define SHIRDA_UART_CLK_NAME	"f995e000.serial"

static int shirda_open(struct inode *inode, struct file *fp);
static int shirda_release(struct inode *inode, struct file *fp);
static int shirda_driver_init(struct platform_device *pdev);
static int shirda_driver_remove(struct platform_device *pdev);

static int shirda2_open(struct inode *inode, struct file *fp);
static int shirda2_release(struct inode *inode, struct file *fp);
static int shirda3_open(struct inode *inode, struct file *fp);
static int shirda3_release(struct inode *inode, struct file *fp);


struct platform_device *shirda_device = NULL;
static struct platform_driver shirda_driver = {
	.remove = __devexit_p(shirda_driver_remove),
	.driver = {
		.name  = SHIRDA_KERNEL_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static struct file_operations shirda_ops = {
	.owner		= THIS_MODULE,
	.open		= shirda_open,
	.release	= shirda_release,
};

static struct miscdevice shirda_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= SHIRDA_KERNEL_DRIVER_NAME,
	.fops	= &shirda_ops,
};

static struct file_operations shirda2_ops = {
	.owner		= THIS_MODULE,
	.open		= shirda2_open,
	.release	= shirda2_release,
};

static struct miscdevice shirda2_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= SHIRDA_KERNEL_DRIVER_NAME2,
	.fops	= &shirda2_ops,
};

static struct file_operations shirda3_ops = {
	.owner		= THIS_MODULE,
	.open		= shirda3_open,
	.release	= shirda3_release,
};

static struct miscdevice shirda3_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= SHIRDA_KERNEL_DRIVER_NAME3,
	.fops	= &shirda3_ops,
};





static struct msm_gpio shirda_gpio_idle[] = {
	{ GPIO_CFG(SHIRDA_GPIO_TXD, 0,                   GPIO_CFG_OUTPUT,
		SHIRDA_TXD_PULL, SHIRDA_TXD_STRENGTH), "IRDA_TX"},
	{ GPIO_CFG(SHIRDA_GPIO_RXD, SHIRDA_GPIO_RX_FUNC, GPIO_CFG_INPUT,
		SHIRDA_RXD_PULL, SHIRDA_RXD_STRENGTH), "IRDA_RX"},
	{ GPIO_CFG(SHIRDA_GPIO_SD,  SHIRDA_GPIO_SD_FUNC, GPIO_CFG_OUTPUT,
		SHIRDA_SD_PULL,  SHIRDA_SD_STRENGTH),  "IRDA_SD"},
};

static struct msm_gpio shirda_gpio_active2[] = {
	{ GPIO_CFG(SHIRDA_GPIO_TXD, SHIRDA_GPIO_TX_FUNC, GPIO_CFG_OUTPUT,
		SHIRDA_TXD_PULL, SHIRDA_TXD_STRENGTH), "IRDA_TX"},
	{ GPIO_CFG(SHIRDA_GPIO_RXD, 0,                   GPIO_CFG_INPUT,
		SHIRDA_RXD_PULL, SHIRDA_RXD_STRENGTH), "IRDA_RX"},
	{ GPIO_CFG(SHIRDA_GPIO_SD,  SHIRDA_GPIO_SD_FUNC, GPIO_CFG_OUTPUT,
		SHIRDA_SD_PULL,  SHIRDA_SD_STRENGTH),  "IRDA_SD"},
};

static struct msm_gpio shirda_gpio_active[] = {
	{ GPIO_CFG(SHIRDA_GPIO_TXD, SHIRDA_GPIO_TX_FUNC, GPIO_CFG_OUTPUT,
		SHIRDA_TXD_PULL, SHIRDA_TXD_STRENGTH), "IRDA_TX"},
	{ GPIO_CFG(SHIRDA_GPIO_RXD, SHIRDA_GPIO_RX_FUNC, GPIO_CFG_INPUT,
		SHIRDA_RXD_PULL, SHIRDA_RXD_STRENGTH), "IRDA_RX"},
	{ GPIO_CFG(SHIRDA_GPIO_SD,  SHIRDA_GPIO_SD_FUNC, GPIO_CFG_OUTPUT,
		SHIRDA_SD_PULL,  SHIRDA_SD_STRENGTH),  "IRDA_SD"},
};

static int shirda_gpio_idle_init[] = {
	0,
	0,
	SHIRDA_SD_SHUTDOWN,
};

static spinlock_t shirda_lock;
#define SHIRDA_WLOCK_SUSPEND_NAME "shirda_suspend_lock"
static struct wake_lock shirda_wlock_suspend;


typedef enum {
	SHIRDA_GSBI_NORMAL,
	SHIRDA_GSBI_INVERT,
	SHIRDA_GSBI_IRDA,
	SHIRDA_GSBI_IRDA_MIR,
} shirda_gsbi_mode;

typedef enum {
	SHIRDA_LED_SIR_FUNC,
	SHIRDA_LED_FIR_FUNC,
	SHIRDA_LED_RC_START_FUNC,
	SHIRDA_LED_RC_END_FUNC,
} shirda_led_function;

typedef enum {
	SHIRDA_STATE_INIT,
	SHIRDA_STATE_IDLE,
	SHIRDA_STATE_READY,
	SHIRDA_STATE_OPEN,
	SHIRDA_STATE_OPEN2,
	SHIRDA_STATE_OPEN3,
	SHIRDA_STATE_ERROR
} shirda_main_state;

static shirda_main_state shirda_state = SHIRDA_STATE_INIT;


static int shirda_msm_gpios_enable(const struct msm_gpio *table, int size);
static int shirda_msm_gpios_request(const struct msm_gpio *table, int size);
static void shirda_msm_gpios_disable_free(const struct msm_gpio *table,
								int size);
static int shirda_msm_gpios_disable(const struct msm_gpio *table, int size);
static void shirda_msm_gpios_free(const struct msm_gpio *table, int size);

/*linux/arch/arm/mach-msm/gpio.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
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

static int shirda_msm_gpios_enable(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = 0; i < size; i++) {
		g = table + i;
		rc = gpio_tlmm_config(g->gpio_cfg, GPIO_CFG_ENABLE);
		if (rc) {
			IRDALOG_ERROR("gpio_tlmm_config(%d) <%s> failed: %d\n",
			       GPIO_PIN(g->gpio_cfg), g->label ?: "?", rc);
			shirda_msm_gpios_disable(table, i);
			return rc;
		}
	}
	return 0;
}

static int shirda_msm_gpios_request(const struct msm_gpio *table, int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;
	for (i = 0; i < size; i++) {
		g = table + i;
		if ((GPIO_FUNC(g->gpio_cfg)) == 0) {
			rc = gpio_request(GPIO_PIN(g->gpio_cfg), g->label);
			if (rc) {
				IRDALOG_ERROR(
					"gpio_request(%d) <%s> failed: %d\n",
					GPIO_PIN(g->gpio_cfg),
					g->label ?: "?", rc);
				shirda_msm_gpios_free(table, i);
				return rc;
			}
		}
	}
	return 0;
}

static void shirda_msm_gpios_disable_free(const struct msm_gpio *table,
								int size)
{
	shirda_msm_gpios_disable(table, size);
	shirda_msm_gpios_free(table, size);
}

static int shirda_msm_gpios_disable(const struct msm_gpio *table, int size)
{
	int rc = 0;
	int i;
	const struct msm_gpio *g;
	for (i = size-1; i >= 0; i--) {
		int tmp;
		g = table + i;
		tmp = gpio_tlmm_config(g->gpio_cfg, GPIO_CFG_DISABLE);
		if (tmp) {
			IRDALOG_ERROR(
				"gpio_tlmm_config(0x%08x, GPIO_CFG_DISABLE)"
				" <%s> failed: %d\n",
				g->gpio_cfg, g->label ?: "?", rc);
			if (!rc)
				rc = tmp;
		}
	}
	return rc;
}

static void shirda_msm_gpios_free(const struct msm_gpio *table, int size)
{
	int i;
	const struct msm_gpio *g;
	for (i = size-1; i >= 0; i--) {
		g = table + i;
		gpio_free(GPIO_PIN(g->gpio_cfg));
	}
}

static int shirda_gpios_direction(const struct msm_gpio *table,
						const int *pol, int size);

static int shirda_gpios_direction(const struct msm_gpio *table,
						const int *pol, int size)
{
	int rc = 0;
	int i;
	const struct msm_gpio *g;
	const int *p;

	for (i = 0; i < size; i++) {
		g = table + i;
		p = pol + i;
		if ((GPIO_FUNC(g->gpio_cfg)) == 0) {

			if (GPIO_DIR(g->gpio_cfg) == GPIO_CFG_INPUT) {
				rc = gpio_direction_input(
						GPIO_PIN(g->gpio_cfg));
			} else {
				rc = gpio_direction_output(
						GPIO_PIN(g->gpio_cfg), *p);
			}
			if (rc != 0) {
				IRDALOG_ERROR(
					"Set gpio dir error(0x%08x): %s\n",
						rc, g->label ?: "?");
			}
		}
	}
	return rc;
}

static int shirda_gpio_init(void)
{
	int ret = 0;


	ret = shirda_msm_gpios_request(shirda_gpio_idle,
					ARRAY_SIZE(shirda_gpio_idle));
	if (ret != 0) {
		IRDALOG_FATAL("gpios_request() fail errno = %d\n", ret);
		return ret;
	}

	ret = shirda_gpios_direction(shirda_gpio_idle, shirda_gpio_idle_init,
					ARRAY_SIZE(shirda_gpio_idle));
	if (ret != 0) {
		IRDALOG_FATAL("gpios direction set fail errno = %d\n", ret );
		shirda_msm_gpios_free(shirda_gpio_active,
					ARRAY_SIZE(shirda_gpio_active));
		return ret;
	}

	ret = shirda_msm_gpios_enable(shirda_gpio_idle,
					ARRAY_SIZE(shirda_gpio_idle));
	if (ret != 0) {
		IRDALOG_FATAL("gpios_enable() fail errno = %d\n", ret);
		shirda_msm_gpios_free(shirda_gpio_active,
					ARRAY_SIZE(shirda_gpio_active));
		return ret;
	}


	return 0;
}

static void shirda_gpio_free(void)
{
	shirda_msm_gpios_disable_free(shirda_gpio_idle,
						ARRAY_SIZE(shirda_gpio_idle));
}

static void shirda_led_set_sir(void)
{
	struct timespec wait_time;
	unsigned long lock_flag;

	wait_time.tv_sec  = 0;
	wait_time.tv_nsec = SHIRDA_WAIT_MODESEQ;
	hrtimer_nanosleep(&wait_time, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	spin_lock_irqsave(&shirda_lock, lock_flag);

	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_SHUTDOWN);
	udelay(SHIRDA_MODESEQ_SDACTIME);
	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_ACTIVE);

	spin_unlock_irqrestore(&shirda_lock, lock_flag);

	wait_time.tv_sec  = 0;
	wait_time.tv_nsec = SHIRDA_MODESEQ_WAITTXD;
	hrtimer_nanosleep(&wait_time, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	return;
}

static void shirda_led_set_rc(void)
{
	struct timespec wait_time;
	unsigned long lock_flag;

	wait_time.tv_sec  = 0;
	wait_time.tv_nsec = SHIRDA_LEDRCMODE_WAIT;
	hrtimer_nanosleep(&wait_time, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	spin_lock_irqsave(&shirda_lock, lock_flag);

	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_SHUTDOWN);

	udelay(SHIRDA_LEDRCMODE_ST1);
	gpio_set_value(SHIRDA_GPIO_TXD, 1);

	udelay(SHIRDA_LEDRCMODE_ST2);
	gpio_set_value(SHIRDA_GPIO_TXD, 0);

	udelay(SHIRDA_LEDRCMODE_ST3);
	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_ACTIVE);

	spin_unlock_irqrestore(&shirda_lock, lock_flag);

	wait_time.tv_sec  = 0;
	wait_time.tv_nsec = SHIRDA_LEDRCMODE_ST4;
	hrtimer_nanosleep(&wait_time, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	return;
}

static int shirda_gpio_set_irda_enable(shirda_led_function func)
{
	int ret = 0;


	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_ACTIVE);

	switch (func) {
	case SHIRDA_LED_SIR_FUNC:
		shirda_led_set_sir();

		ret = shirda_msm_gpios_enable(shirda_gpio_active,
					ARRAY_SIZE(shirda_gpio_active));
		if (ret != 0) {
			IRDALOG_FATAL("gpios_enable() fail errno = %d\n", ret);
		}
		break;
	case SHIRDA_LED_RC_START_FUNC:
		shirda_led_set_rc();

		ret = shirda_msm_gpios_enable(shirda_gpio_active2,
					ARRAY_SIZE(shirda_gpio_active2));
		if (ret != 0) {
			IRDALOG_FATAL("gpios_enable() fail errno = %d\n", ret);
		}
		break;
	case SHIRDA_LED_RC_END_FUNC:
	default:
		IRDALOG_FATAL("Internal error! shirda_led_function = %d\n",
									func);
		ret = -EIO;
		break;
	}


	return ret;
}

static int shirda_gpio_set_irda_disable(shirda_led_function func)
{
	int ret = 0;


	switch (func) {
	case SHIRDA_LED_SIR_FUNC:
		break;
	case SHIRDA_LED_RC_END_FUNC:
		shirda_led_set_sir();
		break;
	default:
		IRDALOG_FATAL("Internal error! shirda_led_function = %d\n",
									func);
		ret = -EIO;
		break;
	}

	ret = shirda_msm_gpios_enable(shirda_gpio_idle,
					ARRAY_SIZE(shirda_gpio_idle));
	if (ret != 0) {
		IRDALOG_FATAL("gpios_enable() fail errno = %d\n", ret);
	}

	gpio_set_value(SHIRDA_GPIO_SD, SHIRDA_SD_SHUTDOWN);

	return ret;
}


static int shirda_clk_enable(void)
{
	struct clk *core_clk;
	struct clk *iface_clk;


	core_clk  = clk_get_sys(SHIRDA_UART_CLK_NAME, "core_clk");
	if (unlikely(IS_ERR(core_clk))) {
		IRDALOG_FATAL("The core_clk(%s) address get error.\n",
							SHIRDA_UART_CLK_NAME);
		return -EIO;
	}

	clk_set_rate(core_clk, SHIRDA_CORE_CLK_FREQ);
	clk_prepare_enable(core_clk);

	iface_clk  = clk_get_sys(SHIRDA_UART_CLK_NAME, "iface_clk");
	if (unlikely(IS_ERR(iface_clk))) {
		IRDALOG_FATAL("The iface_clk(%s) address get error.\n",
							SHIRDA_UART_CLK_NAME);
		clk_disable_unprepare(core_clk);
		clk_put(core_clk);
		return -EIO;
	}

	clk_prepare_enable(iface_clk);

	return 0;
}









static void shirda_clk_disable(void)
{
	struct clk *core_clk;
	struct clk *iface_clk;


	core_clk  = clk_get_sys(SHIRDA_UART_CLK_NAME, "core_clk");
	if (unlikely(IS_ERR(core_clk))) {
		IRDALOG_FATAL("The core_clk(%s) address get error.\n",
							SHIRDA_UART_CLK_NAME);
	} else {
		clk_disable_unprepare(core_clk);
		clk_put(core_clk);
	}

	iface_clk  = clk_get_sys(SHIRDA_UART_CLK_NAME, "iface_clk");
	if (unlikely(IS_ERR(iface_clk))) {
		IRDALOG_FATAL("The iface_clk(%s) address get error.\n",
							SHIRDA_UART_CLK_NAME);
	} else {
		clk_disable_unprepare(iface_clk);
		clk_put(iface_clk);
	}

	return;
}
































static int shirda_set_uartdm_irda_enable(shirda_gsbi_mode mode)
{
	unsigned long	lock_flag;
	void __iomem	*gsbi_irda = ioremap_nocache(SHIRDA_UART_DM_IRDA, 4);
	int		ret = 0;

	ret = shirda_clk_enable();
	if (ret != 0) {
		IRDALOG_FATAL("Fatal error! can't enable clk.\n");
		goto uartdm_irda_ena_err;
	}

	spin_lock_irqsave(&shirda_lock, lock_flag);

	switch (mode) {
	case SHIRDA_GSBI_NORMAL:
		writel_relaxed((INVERT_IRDA_RX), gsbi_irda);
		mb();
		break;
	case SHIRDA_GSBI_INVERT:
		writel_relaxed((INVERT_IRDA_RX | INVERT_IRDA_TX), gsbi_irda);
		mb();
		break;
	case SHIRDA_GSBI_IRDA:
		writel_relaxed((INVERT_IRDA_RX | IRDA_EN), gsbi_irda);
		mb();
		break;
	case SHIRDA_GSBI_IRDA_MIR:
		writel_relaxed((INVERT_IRDA_RX | IRDA_EN | MEDIUM_RATE_EN),
								 gsbi_irda);
		mb();
		break;
	default:
		IRDALOG_FATAL("Internal error! shirda_gsbi_mode = %d\n", mode);
		ret = -EIO;
		break;
	}

	spin_unlock_irqrestore(&shirda_lock, lock_flag);

	shirda_clk_disable();

uartdm_irda_ena_err:

	if (gsbi_irda != NULL)	iounmap(gsbi_irda);

	return ret;
}

static void shirda_set_uartdm_irda_disable(void)
{
	unsigned long lock_flag;
	void __iomem *gsbi_irda = ioremap_nocache(SHIRDA_UART_DM_IRDA, 4);


	if (shirda_clk_enable() != 0) {
		IRDALOG_FATAL("Fatal error! can't enable clk.\n");
		goto uartdm_irda_dis_err;
	}

	spin_lock_irqsave(&shirda_lock, lock_flag);

	writel_relaxed((INVERT_IRDA_RX), gsbi_irda);
	mb();

	spin_unlock_irqrestore(&shirda_lock, lock_flag);

	shirda_clk_disable();

uartdm_irda_dis_err:
	if (gsbi_irda != NULL)	iounmap(gsbi_irda);
}

static int shirda_open(struct inode *inode, struct file *fp)
{
	int ret = 0;


	switch (shirda_state) {
	case SHIRDA_STATE_IDLE:
	case SHIRDA_STATE_READY:
		ret = shirda_set_uartdm_irda_enable(SHIRDA_GSBI_IRDA);
		if (ret != 0) {
			IRDALOG_ERROR("IrDA setting was not completed.\n");
		} else {
			ret = shirda_gpio_set_irda_enable(SHIRDA_LED_SIR_FUNC);
		}
		break;
	case SHIRDA_STATE_OPEN2:
	case SHIRDA_STATE_OPEN3:
		IRDALOG_INFO("Already using by IrDA. state = %d\n",
								shirda_state);
		return -EBUSY;
	default:
		IRDALOG_ERROR("open error. state = %d\n", shirda_state);
		return -EPERM;
	}

	if (ret == 0) {
		wake_lock(&shirda_wlock_suspend);
	}


	if (ret != 0) {
		shirda_set_uartdm_irda_disable();
	} else {
		shirda_state = SHIRDA_STATE_OPEN;
	}

	return ret;
}

static int shirda_release(struct inode *inode, struct file *fp)
{
	int ret = 0;


	if (shirda_state != SHIRDA_STATE_OPEN) {
		IRDALOG_ERROR("close error. state = %d\n", shirda_state);
		ret = -EPERM;
	} else {
		ret = shirda_gpio_set_irda_disable(SHIRDA_LED_SIR_FUNC);
		if (ret == 0) {
			shirda_state = SHIRDA_STATE_READY;
			wake_unlock(&shirda_wlock_suspend);
		} else {
			IRDALOG_FATAL("irda disable fatal error.\n");
			shirda_state = SHIRDA_STATE_ERROR;
			shirda_gpio_free();
		}
		shirda_set_uartdm_irda_disable();
	}
	return ret;
}

static int shirda2_open(struct inode *inode, struct file *fp)
{
	int ret = 0;


	switch (shirda_state) {
	case SHIRDA_STATE_IDLE:
	case SHIRDA_STATE_READY:
		ret = shirda_set_uartdm_irda_enable(SHIRDA_GSBI_INVERT);
		if (ret != 0) {
			IRDALOG_ERROR("IrDA setting was not completed.\n");
		} else {
			ret = shirda_gpio_set_irda_enable(
						SHIRDA_LED_RC_START_FUNC);
		}
		break;
	case SHIRDA_STATE_OPEN:
	case SHIRDA_STATE_OPEN3:
		IRDALOG_INFO("Already using by IrDA. state = %d\n",
								shirda_state);
		return -EBUSY;
	default:
		IRDALOG_ERROR("open error. state = %d\n", shirda_state);
		return -EPERM;
	}

	if (ret == 0) {
		wake_lock(&shirda_wlock_suspend);
		IRDALOG_INFO("wake_lock_suspend\n");
	}


	if (ret != 0) {
		shirda_gpio_set_irda_disable(SHIRDA_LED_RC_END_FUNC);
		shirda_set_uartdm_irda_disable();
	} else {
		shirda_state = SHIRDA_STATE_OPEN2;
	}

	return ret;
}

static int shirda2_release(struct inode *inode, struct file *fp)
{
	int ret = 0;


	if (shirda_state != SHIRDA_STATE_OPEN2) {
		IRDALOG_ERROR("close error. state = %d\n", shirda_state);
		ret = -EPERM;
	} else {
		ret = shirda_gpio_set_irda_disable(SHIRDA_LED_RC_END_FUNC);
		if (ret == 0) {
			shirda_state = SHIRDA_STATE_READY;
			wake_unlock(&shirda_wlock_suspend);
			IRDALOG_INFO("wake_unlock_suspend\n");
		} else {
			IRDALOG_FATAL("irda disable fatal error.\n");
			shirda_state = SHIRDA_STATE_ERROR;
			shirda_gpio_free();
		}
		shirda_set_uartdm_irda_disable();
	}
	return ret;
}

static int shirda3_open(struct inode *inode, struct file *fp)
{
	int ret = 0;


	switch (shirda_state) {
	case SHIRDA_STATE_IDLE:
	case SHIRDA_STATE_READY:
		ret = shirda_set_uartdm_irda_enable(SHIRDA_GSBI_IRDA_MIR);
		if (ret != 0) {
			IRDALOG_ERROR("IrDA setting was not completed.\n");
		} else {
			ret = shirda_gpio_set_irda_enable(
						SHIRDA_LED_RC_START_FUNC);
		}
		break;
	case SHIRDA_STATE_OPEN:
	case SHIRDA_STATE_OPEN2:
		IRDALOG_INFO("Already using by IrDA. state = %d\n",
								shirda_state);
		return -EBUSY;
	default:
		IRDALOG_ERROR("open error. state = %d\n", shirda_state);
		return -EPERM;
	}

	if (ret == 0) {
		wake_lock(&shirda_wlock_suspend);
		IRDALOG_INFO("wake_lock_suspend\n");
	}


	if (ret != 0) {
		shirda_gpio_set_irda_disable(SHIRDA_LED_RC_END_FUNC);
		shirda_set_uartdm_irda_disable();
	} else {
		shirda_state = SHIRDA_STATE_OPEN3;
	}

	return ret;
}

static int shirda3_release(struct inode *inode, struct file *fp)
{
	int ret = 0;


	if (shirda_state != SHIRDA_STATE_OPEN3) {
		IRDALOG_ERROR("close error. state = %d\n", shirda_state);
		ret = -EPERM;
	} else {
		ret = shirda_gpio_set_irda_disable(SHIRDA_LED_RC_END_FUNC);
		if (ret == 0) {
			shirda_state = SHIRDA_STATE_READY;
			wake_unlock(&shirda_wlock_suspend);
			IRDALOG_INFO("wake_unlock_suspend\n");
		} else {
			IRDALOG_FATAL("irda disable fatal error.\n");
			shirda_state = SHIRDA_STATE_ERROR;
			shirda_gpio_free();
		}
		shirda_set_uartdm_irda_disable();
	}
	return ret;
}





static int __devinit shirda_driver_init(struct platform_device *pdev)
{
	int ret = 0;


	ret = shirda_gpio_init();
	if (ret != 0) {
		IRDALOG_FATAL("gpio initialization failed. errno = %d\n", ret);
		shirda_state = SHIRDA_STATE_ERROR;
		return ret;
	}

	spin_lock_init(&shirda_lock);

	if (shirda_set_uartdm_irda_enable(SHIRDA_GSBI_NORMAL) == 0) {
		shirda_state = SHIRDA_STATE_READY;
	} else {
		IRDALOG_ERROR("IrDA setting was not completed.\n");
		shirda_state = SHIRDA_STATE_IDLE;
	}

	misc_register(&shirda_misc);

	misc_register(&shirda2_misc);
	misc_register(&shirda3_misc);


	wake_lock_init(&shirda_wlock_suspend, WAKE_LOCK_SUSPEND,
						SHIRDA_WLOCK_SUSPEND_NAME);

	return ret;
}

static int shirda_driver_remove(struct platform_device *pdev)
{


	shirda_gpio_free();

	wake_lock_destroy(&shirda_wlock_suspend);

	misc_deregister(&shirda_misc);

	misc_deregister(&shirda2_misc);
	misc_deregister(&shirda3_misc);


	return 0;
}

static int __init shirda_module_init(void)
{
	int ret = 0;


	shirda_device = platform_device_alloc(
				(const char *)SHIRDA_KERNEL_DRIVER_NAME,
							(unsigned int) -1);
	if (shirda_device == NULL) {
		IRDALOG_FATAL("platform_device_alloc() failed\n");
		return -ENODEV;
	}

	ret = platform_device_add(shirda_device);
	if (ret != 0) {
		platform_device_put(shirda_device);
		IRDALOG_FATAL("platform_device_add() failed errno = %d\n",
								 ret);
	} else {
		ret = platform_driver_probe(&shirda_driver,
							shirda_driver_init);
		if (ret != 0) {
			platform_device_unregister(shirda_device);
			IRDALOG_FATAL(
				"platform_driver_probe() failed errno = %d\n",
								ret);
		}
	}

	return ret;
}

static void __exit shirda_module_exit(void)
{
	platform_driver_unregister(&shirda_driver);
	platform_device_unregister(shirda_device);
}

module_init(shirda_module_init);
module_exit(shirda_module_exit);

MODULE_DESCRIPTION("msm IrDA driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION(SHIRDA_KERNEL_DRIVER_VERSION);
