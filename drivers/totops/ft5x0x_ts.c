/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/ioc4.h>
#include <linux/io.h>

#include <linux/proc_fs.h>

#include <mach/gpio.h>

#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>

#include "ft5x0x_ts.h"

#include <linux/input/mt.h>
#include <linux/delay.h>


static u8 buf[40] = {0};

#define FT_INT_PORT		EXYNOS4_GPX0(4)
//#define FT_WAKE_PORT 	GPIO_TS_WAKE  // orig

#define FT_WAKE_PORT 	      EXYNOS4X12_GPM3(4)
//#define FT_POWER_PORT	GPIO_TS_POWER

#define EINT_NUM	 IRQ_EINT(4)



#define set_irq_type   irq_set_irq_type		//raymanfeng for kernel 3.0 8
//static int touch_key_down;
//static int touch_key_count;
static struct ft5x0x_ts_data *s_ft5x0x_ts = NULL;

static struct i2c_client *this_client;
//static struct ft5x0x_ts_platform_data *pdata;

#define CONFIG_FT5X0X_MULTITOUCH 1
#define CONFIG_ANDROID_4_ICS 1                //raymanfeng

//extern int MmFlag;
static  int MmFlag = 1;
//extern spinlock_t lock ;

//extern int g_touchscreen_init;
//static   int g_touchscreen_init;
//extern int g_touchscreen_init ;
//EXPORT_SYMBOL(g_touchscreen_init);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
        u16     x3;
        u16     y3;
        u16     x4;
        u16     y4;
        u16     x5;
        u16     y5;
	u16	pressure;
	s16  touch_ID1;
	s16  touch_ID2;
    	s16  touch_ID3;
    	s16  touch_ID4;
	s16  touch_ID5;
   	 u8  touch_point;
	u8   status;
};

struct tp_event {
	u16	x;
	u16	y;
   	 s16 id;
	u16	pressure;
	u8  touch_point;
	u8  flag;
};

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};


//static int s_model = 0;

//-----
#define TOUCH_TIMER     msecs_to_jiffies(50)
static struct timer_list timer;
static int flag_tp=0;
//static int g_x,g_y;
//static int first_time = 1;

#undef SCREEN_MAX_X
#undef SCREEN_MAX_Y

//#define SCREEN_MAX_X    1024//105
//#define SCREEN_MAX_Y    768
static int SCREEN_MAX_X = 1024;
static int SCREEN_MAX_Y = 600;

#if 0
#define FTprintk(x...) printk(x)
#else
#define FTprintk(x...) do{} while(0)
#endif

#define MAX_POINT                 5

//------
extern char g_selected_utmodel[];
static void check_model(void)
{
	//extern char g_selected_utmodel[];

#if 0
	if(strncmp(g_selected_utmodel, "s101", strlen("s101")) == 0)
	{
		SCREEN_MAX_X = 1280;
		SCREEN_MAX_Y = 800;
	} 
	if(strncmp(g_selected_utmodel, "s702", strlen("s702")) == 0)
	{
		SCREEN_MAX_X = 800;
		SCREEN_MAX_Y = 1280;
	}
	if(strncmp(g_selected_utmodel, "s703", strlen("s703")) == 0)
	{
		SCREEN_MAX_X = 800;
		SCREEN_MAX_Y = 1280;
	}	
#endif	
		FTprintk("ft x=%d, y=%d\n", SCREEN_MAX_X, SCREEN_MAX_Y);

}

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return ret;
}

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	
	struct ft5x0x_ts_data *data = container_of(work, struct ft5x0x_ts_data, pen_event_work);
	struct tp_event event;
	u8 start_reg=0x0;
	u8 buf[32] = {0};
	int ret,i,offset,points;
	static u8 points_last_flag[MAX_POINT]={0};
	struct tp_event  current_events[MAX_POINT];

	//FTprintk("--%s\n",__func__);
		
#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret =  ft5x0x_i2c_rxdata(buf, 31);
#else
	ret =  ft5x0x_i2c_rxdata(buf, 7);
#endif
	if (ret < 0) {
		FTprintk("ft5406_read_regs fail!--return --\n");
		enable_irq(this_client->irq);
		return;
	}
#if 0
	for (i=0; i<32; i++) {
		FTprintk("buf[%d] = 0x%x \n", i, buf[i]);
	}
#endif
	
	points = buf[2] & 0x07;
	FTprintk("point:%d\n",points);
	
	if (points == 0) {
#if   CONFIG_FT5X0X_MULTITOUCH		
		for(i=0;i<MAX_POINT;i++)
		{
			if(points_last_flag[i]!=0)
			{
				FTprintk("Point UP event.id=%d\n",i);
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);	
				
			}
		}

		memset(points_last_flag, 0, sizeof(points_last_flag));
#else
		input_report_abs(data->input_dev, ABS_PRESSURE, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
		input_sync(data->input_dev);
		enable_irq(this_client->irq);
		return; 
	}
	memset(&event, 0, sizeof(struct tp_event));

	FTprintk("-----\n");
	
#if CONFIG_FT5X0X_MULTITOUCH
  memset(current_events, 0, sizeof(current_events));
  
	for(i=0;i<points;i++){
		offset = i*6+3;
		event.x = (((s16)(buf[offset+0] & 0x0F))<<8) | ((s16)buf[offset+1]);
		event.y = (((s16)(buf[offset+2] & 0x0F))<<8) | ((s16)buf[offset+3]);
		event.id = (s16)(buf[offset+2] & 0xF0)>>4;
		event.flag = ((buf[offset+0] & 0xc0) >> 6);
		event.pressure = 200;
	
		
		FTprintk("x=%d, y=%d event.id=%d event.flag=%d\n",event.x,event.y,event.id,event.flag);
		if(event.x<=SCREEN_MAX_X && event.y<=SCREEN_MAX_Y+60){
		if(event.flag)
			memcpy(&current_events[event.id], &event, sizeof(event));
			//points_current[event.id] = event.flag;			
		}
	}

	FTprintk("+++++++++\n");
	
	for(i=0;i<MAX_POINT;i++)
	{	
		FTprintk("current_flag:%d,last_flag:%d\n",current_events[i].flag,points_last_flag[i]);

		  if((current_events[i].flag == 0) && (points_last_flag[i] != 0))
			{
      					FTprintk("Point UP event.id=%d\n",i);
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);		
					//input_report_key(data->input_dev, BTN_TOUCH, 0);
			}
		else  if(current_events[i].flag)	
			{	
		  			FTprintk("Point DN event.id=%d\n",i);
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
					input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(data->input_dev, ABS_MT_PRESSURE, event.pressure);
					input_report_abs(data->input_dev, ABS_MT_POSITION_X,  current_events[i].x);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y,  current_events[i].y);	
					//input_report_key(data->input_dev, BTN_TOUCH, 1);
			}		
			points_last_flag[i] = 	current_events[i].flag;
	}
#else
	event.x = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
	event.y = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	event.pressure =200;
	input_report_abs(data->input_dev, ABS_X, event.x);
	input_report_abs(data->input_dev, ABS_Y, event.y);
	//input_report_abs(data->input_dev, ABS_PRESSURE, event.pressure);
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	
#endif
	FTprintk("ft5406 sync  x = %d ,y = %d\n",event.x,event.y);

	input_sync(data->input_dev);
	
	enable_irq(this_client->irq);
	return;
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	
	//FTprintk("==ft5x0x_ts_interrupt=\n");
        flag_tp=1;
      //  mod_timer(&timer, jiffies + TOUCH_TIMER);

	disable_irq_nosync(this_client->irq);


	if (!work_pending(&ft5x0x_ts->pen_event_work)) 
	{
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	int ret;
	
	struct ft5x0x_ts_data *ts;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk(KERN_INFO"==ft5x0x_ts_suspend==\n");
//        gpio_direction_output(FT_POWER_PORT, 0);  


	gpio_direction_output(FT_WAKE_PORT, 0);	

	disable_irq_nosync(this_client->irq);

//add by jerry
	//msleep(200);
	mdelay(200);
//

	ret = cancel_work_sync(&ts->pen_event_work);	
	flush_workqueue(ts->ts_workqueue);

	mdelay(50);
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	
	printk(KERN_INFO"==ft5x0x_ts_resume==~~~\n");

#if 1 
    /*   
    	err = gpio_request(FT_WAKE_PORT, "ft5x0x-ts-wake-en");
	if(err<0) {
		//dev_err(&client->dev, "Error: FT_WAKE_PORT request failed !\n");
		//goto exit_gpio_request_failed;
	}
   */
	gpio_direction_output(FT_WAKE_PORT, 1);
	mdelay(100); //mdelay(50);//500 100ms delay for s901
	gpio_direction_output(FT_WAKE_PORT, 0);
	mdelay(50); //mdelay(25);//500
	gpio_direction_output(FT_WAKE_PORT, 1);
	mdelay(50);//500 
	

	//gpio_free(FT_WAKE_PORT);   // attention :: open for 5406  
#endif
	
	enable_irq(this_client->irq);

}
#endif  //CONFIG_HAS_EARLYSUSPEND


static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0, retry=0;

        FTprintk("ft 5x0x_ts_5406 ++++++++++++++++++++++++++++ \n");	
	FTprintk(KERN_INFO"==ft5x0x_ts_probe=\n");

	check_model();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	FTprintk(KERN_DEBUG"==kzalloc=\n");
	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	FTprintk(KERN_DEBUG"==i2c_set_clientdata=\n");
	this_client = client;
	this_client->irq = EINT_NUM;

	err = gpio_request(FT_WAKE_PORT, "ft5x0x-ts-wake-en");
	if(err<0) {
		dev_err(&client->dev, "Error: FT_WAKE_PORT request failed !\n");
		//goto exit_gpio_request_failed;
	}
	
	s_ft5x0x_ts=ft5x0x_ts;
	
	gpio_direction_output(FT_WAKE_PORT, 1);
	msleep(100); //msleep(50); 100ms delay for s901
	
	gpio_direction_output(FT_WAKE_PORT, 0);
	msleep(50); //msleep(30);
	
	gpio_direction_output(FT_WAKE_PORT, 1);
	

	msleep(100); //msleep(50);
	
//	gpio_free(FT_WAKE_PORT); //attention:: open for 5406  !!  // for support suspend
//	msleep(16);
	
	
	#if 1 //FIXME: disable it for testing
	for(retry=0; retry < 4; retry++) {
		err =ft5x0x_i2c_txdata( NULL, 0);
		if (err > 0)
			break;
	}

	if(err <= 0) {
	//	FTprintk("222222222222222222222222222222222222222222222222 \n");
		dev_err(&client->dev, "Warnning: I2C connection might be something wrong!\n");
		goto err_i2c_failed;
	}
	#endif




	i2c_set_clientdata(client, ft5x0x_ts);

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}


	s3c_gpio_setpull(FT_INT_PORT, S3C_GPIO_PULL_UP);
	//set_irq_type(this_client->irq, IRQ_TYPE_LEVEL_LOW);
  	set_irq_type(this_client->irq, IRQ_TYPE_EDGE_FALLING);
	//set_irq_type(this_client->irq, IRQ_TYPE_EDGE_BOTH);


	err = request_irq(this_client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq_nosync(this_client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	//__set_bit(EV_ABS, input_dev->evbit);	
	input_mt_init_slots(input_dev, MAX_POINT);

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	#ifdef CONFIG_ANDROID_4_ICS
		set_bit(ABS_PRESSURE, input_dev->absbit);
		//set_bit(BTN_TOUCH, input_dev->keybit);	// 2012-11-20 removed by ericli 
		input_set_abs_params(input_dev,
				     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
	
	#endif
     input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);

#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN,input_dev->evbit);
	
	
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit); //wigo 

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "ft5x0x_ts_probe: failed to register input device: %s\n", 
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	FTprintk(KERN_DEBUG"==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	if(ft5x0x_set_reg(0x88, 0x05) <= 0) //5, 6,7,8
        {
          FTprintk("error error \n");
          goto exit_set_reg_failed;
        }

	if(ft5x0x_set_reg(0x80, 30) <= 0)
        {
           FTprintk("error error \n");
          goto exit_set_reg_failed;
        }
	
    	enable_irq(this_client->irq);
        
     //   init_timer(&timer);
     //   timer.function = (void *)time_function;
     //   timer.expires = jiffies + TOUCH_TIMER;
     //   add_timer(&timer);

	dev_warn(&client->dev, "=====probe over =====\n");
	
	   
    return 0;

exit_set_reg_failed:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(this_client->irq, ft5x0x_ts);
exit_irq_request_failed:
//exit_platform_data_null:

	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	FTprintk(KERN_DEBUG"==singlethread error =\n");
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	i2c_set_clientdata(client, NULL);
//exit_gpio_request_failed:	
err_i2c_failed:	
	gpio_free(FT_WAKE_PORT); 
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
    //free_irq(EINT_NUM, ft5x0x_ts);
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	FTprintk(KERN_INFO"==ft5x0x_ts_remove=\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	//input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
	free_irq(this_client->irq, ft5x0x_ts);
	gpio_free(FT_WAKE_PORT); 
	//kfree(ft5x0x_ts);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	//destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};


extern struct regulator *regulator_get(struct device *dev, const char *id);
extern int regulator_enable(struct regulator *regulator);
extern int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV);

static int __init ft5x0x_ts_init(void)
{
	u8 ret = 0;
	extern char g_selected_tp[32];
#if 0
	if(strncmp( g_selected_tp, "ft54", strlen("ft54"))!=0) {
		FTprintk(KERN_INFO"## ft54 ft5x0x_ts_init= [%s]\n", g_selected_tp);	
		return -1;
	}
#endif

/*
    struct regulator *regulator = NULL;
    regulator = regulator_get(NULL, "axp22_dldo2");
    if(NULL != regulator){
        regulator_enable(regulator);
        regulator_set_voltage(regulator, 2900000, 3000000);
        printk("touch screen power ok\n");
    }else{
        printk("touch screen power failed\n");
    }
*/

	check_model();

	FTprintk(KERN_INFO"==ft5x0x_ts_init=\n");	
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	FTprintk("ret = %d.\n",ret);
	return ret;
	
}

static void __exit ft5x0x_ts_exit(void)
{
	FTprintk(KERN_INFO"==ft5x0x_ts_exit=\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

late_initcall(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
