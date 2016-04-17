#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pci.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>

#define DEVICE_NAME "leds0" //设备名(/dev/leds)

#define LED_MAJOR 240


#define EMAIL_LED EXYNOS4_GPX2(1)

void totops_email_led(bool on) 
{ 
    int err; 
         
     //email led  enable
    err = gpio_request(EMAIL_LED, "EMAIL_LED"); 
    if (err){
       printk(KERN_ERR "#### failed to request EMAIL_LED ####\n"); 
       return false;
    }
 
    if(on){
        gpio_direction_output(EMAIL_LED, 1); 
    }else{
        gpio_direction_output(EMAIL_LED, 0); 
    }
    
    gpio_free(EMAIL_LED); 
         
    return true; 
} 


static int led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
      printk("\ncmd = %d  arg = %d \n", cmd, arg);

	if(arg)
		totops_email_led(0);
	else
		totops_email_led(1);

    return 0;
}

struct file_operations led_fops={
    .owner = THIS_MODULE,
    .unlocked_ioctl = led_ioctl,
};

static struct miscdevice misc = {

    .minor = MISC_DYNAMIC_MINOR, //动态设备号

    .name = DEVICE_NAME,

    .fops = &led_fops,

};


static int __init led_init(void)
{
    int rc;

    gpio_request(EMAIL_LED, "EMAIL_LED"); 
    s3c_gpio_cfgpin(EMAIL_LED, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(EMAIL_LED, S3C_GPIO_PULL_NONE); 
    gpio_direction_output(EMAIL_LED, 1); 
    gpio_free(EMAIL_LED); 

    rc = misc_register(&misc);
    if(rc<0)
    {
        printk(KERN_ALERT"register %s char dev error\n","leds");
        return -1;
    }
    printk(KERN_ALERT"OK!\n");
    return 0;
}

static void __exit led_exit(void)
{
    unregister_chrdev(LED_MAJOR, "leds");
    printk(KERN_ALERT"module exit\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("cw");

