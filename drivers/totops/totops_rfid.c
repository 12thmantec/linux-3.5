/*
 * linux/drivers/urbetter/totops_rfid.c
 *
 * RFID measurement code for TOTOPS smdk platform.
 *
 * Copyright (C) 2013 Totops Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <stddef.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/map.h>
#include <mach/gpio.h>

#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>

#include "totops_rfid.h"

static struct device *gdev;
static rfid_device_info totops_rfid_info;
static bool totops_rfid_initial = false;
static bool wifi_led_statues= false;

static ssize_t totops_rfid_show_property(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t totops_rfid_store_property(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int totops_rfid_suspend(struct device *dev);
static void totops_rfid_resume(struct device *dev);
static int __devinit totops_rfid_probe(struct platform_device *pdev);
static int __devinit totops_rfid_remove(struct platform_device *pdev);

static struct device_attribute totops_rfid_attrs[] = {
	TOTOPS_RFID_ATTR(rfid_ce),
	TOTOPS_RFID_ATTR(rfid_icc),
	TOTOPS_RFID_ATTR(rfid_sps),
	TOTOPS_RFID_ATTR(gps_power),
	TOTOPS_RFID_ATTR(email_led),
	TOTOPS_RFID_ATTR(wifi_led),
};

// totops_rfid_pm_ops - RFID power manager related operations.s
static const struct dev_pm_ops totops_rfid_pm_ops = {
	.prepare	= totops_rfid_suspend,
	.complete	= totops_rfid_resume,
};

static struct platform_driver totops_rfid_driver = {
	// driver here is device_driver, platform independent drivers. but what's the differences between
	// .pm ops and .suspend/resume ops?
	.driver = {
		.name	= "totops-rfid",
		.owner	= THIS_MODULE,
		.pm	= &totops_rfid_pm_ops,
	},
	.probe		= totops_rfid_probe,
	.remove		= __devexit_p(totops_rfid_remove),
};


/* example - 1. define a cable check procedure. */
void totops_cable_check_status(int flag)
{
	/*
    charger_type_t status = 0;

    if (flag == 0)
	status = CHARGER_BATTERY;
    else
	status = CHARGER_USB;

    samsung_cable_status_update(status);
    */
}
/* 2.export symbol */
EXPORT_SYMBOL(totops_cable_check_status);

/************************************************************************
 * totops_rfid_generate_event - generate uevent update msgs.
 *
 * @dev: rfid device
 * @reason: msg types.
 *
 * Auth: Hu Chuan 13.10.17
 ***********************************************************************/
static void totops_rfid_generate_event(struct device* dev, rfid_update_reason reason)
{
	char *envp[2] = {NULL, NULL};

	switch (reason)
	{
	case RFID_UPDATE_SYSFS:
		envp[0] = "SOURCE=sysfs";
		break;
	case RFID_UPDATE_KEY:
		envp[0] = "SOURCE=key";
		break;
	case RFID_CARD_INCOMING:
		envp[0] = "SOURCE=card";
		break;
	default:
		envp[0] = "SOURCE=unknown";
		break;
	}
	envp[1] = NULL;

	kobject_uevent_env(&gdev->kobj, KOBJ_CHANGE, envp);
	sysfs_notify(&gdev->kobj, NULL, "rfid_msg"); 
}

static void totops_rfid_update_status(struct device *dev)
{}

/************************************************************************
 *  totops_rfid_enable- enable/disable rfid module,GPX0_5
 *                      rfid power enable,GPX1_4
 *
 * @dev: rfid device.
 * @attr:device attributes
 * @buf: data buffer.
 *
 * ret : operation status.
 * Auth: Hu Chuan 13.9.22
 ***********************************************************************/
static bool totops_rfid_enable(bool flag)
{
    int err;
    printk(KERN_DEBUG "%s\n",__func__);

    //enable/disable rfid
    err = gpio_request(EXYNOS4_GPX0(5), "GPX0");
    if (err){
        printk(KERN_ERR "#### failed to request GPX0_5 ####\n");
        return false;
    }

    s3c_gpio_setpull(EXYNOS4_GPX0(5), S3C_GPIO_PULL_NONE);

    if(true == flag){
        gpio_direction_output(EXYNOS4_GPX0(5), 1); 
    }else{
        gpio_direction_output(EXYNOS4_GPX0(5), 0); 
    }

    gpio_free(EXYNOS4_GPX0(5));

    return true;
}

static bool totops_gps_enable(bool on) 
{ 
    int err; 
         
     //gps power enable
    err = gpio_request(EXYNOS4_GPX0(6), "GPX0-6"); 
    if (err){
       printk(KERN_ERR "#### failed to request GPX0_6 ####\n"); 
       return false;
    }
        
    s3c_gpio_cfgpin(EXYNOS4_GPX0(6), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(EXYNOS4_GPX0(6), S3C_GPIO_PULL_NONE); 
    if(on){
        gpio_direction_output(EXYNOS4_GPX0(6), 1); 
    }else{
        gpio_direction_output(EXYNOS4_GPX0(6), 0); 
    }
    
    gpio_free(EXYNOS4_GPX0(6)); 
         
    return true; 
} 

#define EMAIL_LED_GPIO EXYNOS4_GPX2(1)
static bool totops_email_led(bool on) 
{ 
    int err; 
         
     //email led  enable
    err = gpio_request(EMAIL_LED_GPIO, "EMAIL_LED_GPIO"); 
    if (err){
       printk(KERN_ERR "#### failed to request EMAIL_LED_GPIO ####\n"); 
       return false;
    }
        
    s3c_gpio_cfgpin(EMAIL_LED_GPIO, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(EMAIL_LED_GPIO, S3C_GPIO_PULL_NONE); 
    if(on){
        gpio_direction_output(EMAIL_LED_GPIO, 1); 
    }else{
        gpio_direction_output(EMAIL_LED_GPIO, 0); 
    }
    
    gpio_free(EMAIL_LED_GPIO); 
         
    return true; 
} 

#define WIFI_LED_GPIO EXYNOS4_GPX0(6)
static bool totops_wifi_led(bool on) 
{ 
    int err; 

    gpio_request(EXYNOS4_GPX3(2), "GPX3");
    if(gpio_get_value(EXYNOS4_GPX3(2)))
    {
	gpio_free(EXYNOS4_GPX3(2));
    }
    else
    {
    	gpio_free(EXYNOS4_GPX3(2));
	return;
    }
        
     //wifi led  enable
    err = gpio_request(WIFI_LED_GPIO, "WIFI_LED_GPIO"); 
    if (err){
       printk(KERN_ERR "#### failed to request WIFI_LED_GPIO ####\n"); 
       return false;
    }
        
    s3c_gpio_cfgpin(WIFI_LED_GPIO, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(WIFI_LED_GPIO, S3C_GPIO_PULL_NONE); 
    if(on){
        gpio_direction_output(WIFI_LED_GPIO, 1); 
	wifi_led_statues=true;
    }else{
        gpio_direction_output(WIFI_LED_GPIO, 0); 
	wifi_led_statues=false;
    }
    
    gpio_free(WIFI_LED_GPIO); 
         
    return true; 
} 


static int totops_rfid_create_attrs(struct device * dev)
{
	int i, rv;

	for (i = 0; i < ARRAY_SIZE(totops_rfid_attrs); i++)
	{
		if ((rv = device_create_file(dev, &totops_rfid_attrs[i])) != 0)
		{
			// revert
			while (i--)
			{
				device_remove_file(dev, &totops_rfid_attrs[i]);		
			}
			return rv;
		}
	}

	return rv;
}

/************************************************************************
 * totops_rfid_show_property - display power status
 *
 * @dev: rfid device.
 * @attr:device attributes
 * @buf: data buffer.
 *
 * ret : operation status.
 * Auth: Hu Chuan 13.9.22
 * Note: TODO. not Tested
 ***********************************************************************/
static ssize_t totops_rfid_show_property(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int i = 0;
	const ptrdiff_t offset = attr - totops_rfid_attrs;

	switch (offset)
	{
	case RFID_CE:
		i += sprintf(buf, "%d\n", totops_rfid_info.ce);
		break;
	
	case RFID_ICC:
		i += sprintf(buf, "%d\n", totops_rfid_info.icc);
		break;

	case RFID_SPS:
		/* not in use now */
		i += sprintf(buf, "uart\n");
		break;

	case GPS_POWER:
		i += sprintf(buf, "%d\n", totops_rfid_info.gps_power);
		break;
	
	case EMAIL_LED:
		i += sprintf(buf, "%d\n", totops_rfid_info.email_led);
		break;
	
	case WIFI_LED:
		i += sprintf(buf, "%d\n", totops_rfid_info.wifi_led);
		break;
	
	default:
		i = -EINVAL;
	}

	return i;
}

/************************************************************************
 * totops_rfid_store_property - store power status && send update msgs.
 *
 * @dev: rfid device.
 * @attr:device attributes
 * @buf: data write into /sys/.../ce
 * @count: data cnt
 *
 * ret : write character cnt.
 * Auth: Hu Chuan 13.9.22
 ***********************************************************************/
static ssize_t totops_rfid_store_property(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	int props = 0;
	int i = count;
	const ptrdiff_t offset = attr - totops_rfid_attrs;

	ret = kstrtoint(buf, 0, &props);
	if (ret != 0) return ret;

	switch (offset)
	{
	case RFID_CE:
		printk("store ce = %d\n", props);
        if(0 == props){
            if(totops_rfid_enable(false))
                totops_rfid_info.ce = props;
        }else{
            if(totops_rfid_enable(true))
                totops_rfid_info.ce = props;
        }
		break;
	
	case RFID_ICC:
		printk("store icc = %d\n", props);
		totops_rfid_info.icc = props;
        totops_rfid_generate_event(NULL, RFID_CARD_INCOMING);
		break;

	case RFID_SPS:
		/* not support now */
		break;

    case GPS_POWER:
		printk("store gps power = %d\n", props);
        if(0 == props){
            if(totops_gps_enable(false))
                totops_rfid_info.gps_power = props;
        }else{
            if(totops_gps_enable(true))
                totops_rfid_info.gps_power = props;
        }
		break;
    case EMAIL_LED:
		printk("store email led = %d\n", props);
        if(0 == props){
            if(totops_email_led(false))
                totops_rfid_info.email_led = props;
        }else{
            if(totops_email_led(true))
                totops_rfid_info.email_led = props;
        }
		break;
    case WIFI_LED:
		//printk("store wifi led = %d\n", props);
        if(0 == props){
            if(totops_wifi_led(false))
                totops_rfid_info.wifi_led = props;
        }else{
            if(totops_wifi_led(true))
                totops_rfid_info.wifi_led = props;
        }
		break;
	default:
		i = -EINVAL;
	}

	return i;
}

/************************************************************************
 * totops_rfid_get_property - get properties. called by upper app
 *
 * @property: which property to get
 * @val: integer return
 *
 * ret : OK or ERROR
 * Auth: Hu Chuan 13.10.16
 * Note:
 ***********************************************************************/
int totops_rfid_get_property(RFID_PROPERTY property, int* val)
{
	printk("%s : property = %d\n", __func__, property);

	switch (property)
	{
	case RFID_CE:
		*val = totops_rfid_info.ce;
		break;

	case RFID_ICC:
		*val = totops_rfid_info.icc;
		break;

	case RFID_SPS:
		*val = totops_rfid_info.sps;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/************************************************************************
 * irqreturn_t totops_rfid_icc_isr(int irq, void *data)
 *  -- RFID card incoming handler.
 *
 * Auth: Hu Chuan 13.10.17
 * Note: 
 ***********************************************************************/
static irqreturn_t totops_rfid_icc_isr(int irq, void *data)
{
	printk("There's a RFID card incoming\n");

	// call upper API like JNI
	totops_rfid_generate_event(NULL, RFID_CARD_INCOMING);
	
	return IRQ_HANDLED;
}

/************************************************************************
 * totops_rfid_icc_callback_register - register ICC register(GPX1_5) to answer
 *     RFID card come in msg.
 *	//RFID_CE  GPX0_5
 *	//RFID_ICC GPX1_5
 * Auth: Hu Chuan 13.10.16
 ***********************************************************************/
static int totops_rfid_icc_callback_register(void)
{
	int ret;
	int irq_num;

	irq_num = gpio_to_irq(EXYNOS4_GPX1(5));
	ret = request_irq(irq_num, totops_rfid_icc_isr, 0, "rfid_icc", NULL);
	if (ret != 0) {
		printk("register icc gpio request failed! ret=%d\n", ret);
		return ret;
	}
	irq_set_irq_type(irq_num, IRQ_TYPE_EDGE_RISING);
	return 0;
}

/************************************************************************
 * totops_rfid_icc_callback_unregister - unregister ICC handler
 *	//RFID_CE  GPX0_5
 *	//RFID_ICC GPX1_5
 * Auth: Hu Chuan 13.10.16
 ***********************************************************************/
static int totops_rfid_icc_callback_unregister(void)
{
	int irq_num;

	irq_num = gpio_to_irq(EXYNOS4_GPX1(5));
	free_irq(irq_num, NULL);
	gpio_free(irq_num);
	return 0;
}

/************************************************************************
 * totops_rfid_suspend - operations to enter suspend state
 *
 * @dev: rfid device.
 * @state:
 *
 * Auth: Hu Chuan 13.9.20
 * Note: TODO. not finished
 ***********************************************************************/
static int totops_rfid_suspend(struct device *dev)
{
	// power supply related ops
	printk("=====%s\n",__func__);
    	totops_gps_enable(false);
        gpio_direction_output(WIFI_LED_GPIO, 0); 
	return 0;
}

/************************************************************************
 * totops_rfid_resume - operations to enter resume state
 *
 * @dev: rfid device.
 *
 * Auth: Hu Chuan 13.9.20
 * Note: TODO. not finished
 ***********************************************************************/
static void totops_rfid_resume(struct device *dev)
{
    //totops_gps_power_on(true);
	// power supply related ops
	printk("=====%s\n",__func__);
	if(wifi_led_statues)
		gpio_direction_output(WIFI_LED_GPIO, 1); 	
}

/************************************************************************
 * totops_rfid_probe - probe used to detect rfid hardware, and proper 
 *   initialization operations, set default value.
 *
 * @pdev : platform dependent params
 *
 * Auth: Hu Chuan 13.9.20
 * Note: 
 ***********************************************************************/
static int __devinit totops_rfid_probe(struct platform_device *pdev)
{
	gdev = &pdev->dev;

	memset(&totops_rfid_info, 0, sizeof(totops_rfid_info));
	totops_rfid_info.icc = 0; // no card
	totops_rfid_info.sps = 0; // UART
	totops_rfid_info.ce  = 0; // not enable
	totops_rfid_info.gps_power  = 0; // not enable
	totops_rfid_info.email_led  = 0; // not enable
	totops_rfid_info.wifi_led  = 0; // not enable

	totops_rfid_create_attrs(&pdev->dev);

	// ICC (GPIO1_5 request callback)
	totops_rfid_icc_callback_register();

	totops_rfid_initial = true;
	totops_email_led(false);
	return 0;
}

/************************************************************************
 * totops_rfid_remove - procedure called when hardware is removed.
 *
 * @pdev : platform dependent params
 *
 * Auth: Hu Chuan 13.9.20
 * Note: 
 ***********************************************************************/
static int __devinit totops_rfid_remove(struct platform_device *pdev)
{
	totops_rfid_icc_callback_unregister();
	
	return 0;
}

/************************************************************************
 * totops_rfid_init - initialize RFID module.
 *
 * Auth: Hu Chuan 13.9.20
 * Note: TODO. not finished
 ***********************************************************************/
static int __init totops_rfid_init(void)
{
	// why should I vbus_wake_lock? the lock used in battery.
	//wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
    printk(KERN_INFO "%s\n",__func__);

	return platform_driver_register(&totops_rfid_driver);
}

static void __exit totops_rfid_exit(void)
{
	platform_driver_unregister(&totops_rfid_driver);
}

// register module into system kernel.
module_init(totops_rfid_init);
module_exit(totops_rfid_exit);

MODULE_AUTHOR("HuChuan <huchuan@totops.com>");
MODULE_DESCRIPTION("Totops RFID driver for Totops SMDK Board");
MODULE_LICENSE("GPL");

