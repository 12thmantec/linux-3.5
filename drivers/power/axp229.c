/*
 *  axp229_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/axp229.h>
#include <linux/slab.h>
//#include <mach/gpio-wifi.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <asm/mach/irq.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-adc.h>
#include <mach/map.h>

#define axp229_DEBUG 0
static int axp_probe_status=0;
static int  charging_source=0;
static struct wake_lock vbus_wake_lock;

struct axp229_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	struct early_suspend		early_suspend;

	int ac_online;
	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
};
struct axp229_chip *chip;

static int axp229_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct axp229_chip *chip = container_of(psy,
				struct axp229_chip, battery);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charging_source;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 4200;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int axp_read(struct i2c_client *client,
				int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}

static int axp_reads(struct i2c_client *client, int reg,
				 int len, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}
	return 0;
}

static int axp_write(struct i2c_client *client,
				 int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}
	return 0;
}


static int axp_writes(struct i2c_client *client, int reg,
				  int len, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}
	return 0;
}


static irqreturn_t charger_irq_handler(int irq, void *dev_id)
{
#if axp229_DEBUG	
    printk("=============charger_irq_handler\n");
 #endif   
	if(gpio_get_value(EXYNOS4_GPX0(3)))
	{
		charging_source=1;
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	power_supply_changed(&chip->ac);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void axp229_early_suspend(struct early_suspend *h)
{
	axp229_set_voltage(ALDO3,3300,0);
	axp229_set_charge_current(2250);
	disable_irq(IRQ_EINT(3));
}
void axp229_late_resume(struct early_suspend *h)
{
	axp229_set_voltage(ALDO3,3300,1);
	axp229_set_charge_current(1050);
	enable_irq(IRQ_EINT(3));
}
#endif

static void axp229_get_soc(void)
{
    uint8_t axp_reg;
    axp_read(chip->client,BAT_SOC, &axp_reg);
	chip->soc = axp_reg & 0x7F;

	if(chip->soc==100)
		chip->status = POWER_SUPPLY_STATUS_FULL;
#if axp229_DEBUG
	printk("===axp229_get_soc chip->soc %d%%\n",chip->soc);
#endif
}


static void axp229_get_status(void)
{
	uint8_t axp_reg;
	axp_read(chip->client,AXP22_MODE_CHGSTATUS, &axp_reg);
	if(axp_reg & 0x40)
	{
		charging_source=1;
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
		axp_read(chip->client,AXP22_STATUS, &axp_reg);
		if(axp_reg & 0x80){
			power_supply_changed(&chip->ac);
#if axp229_DEBUG
			printk("SCV4412 BOARD AXP221 AC CHARGING,Please Check!\n");
#endif
			return;
		}
		if(axp_reg & 0x20){
			power_supply_changed(&chip->usb);
#if axp229_DEBUG
			printk("SCV4412 BOARD AXP221 USB CHARGING,Please Check!\n");
#endif
		}
	}
	else
	{
		charging_source=0;
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
#if axp229_DEBUG
		printk("SCV4412 BOARD AXP221 DISCHARGING,Please Check!\n");
#endif
	}
}

void axp229_power_off(void)
{
    	uint8_t axp_reg;
	axp229_set_charge_current(2250);
	printk(KERN_EMERG "%s : SETTING AXP229 POWER OFF SYSTEM.\n", __func__);
    	axp_read(chip->client,POWER_KEY, &axp_reg);
	while(1)axp_write(chip->client,POWER_OFF_REG, axp_reg | 0x88); 	
}
EXPORT_SYMBOL_GPL(axp229_power_off);

void axp229_set_charge_current(unsigned int ma)
{
	switch(ma)
	{
		case 300:
			axp_write(chip->client,CHARGE_REG1, 0xC0); 
			break;
		case 450:
			axp_write(chip->client,CHARGE_REG1, 0xC1); 
			break;
		case 600:
			axp_write(chip->client,CHARGE_REG1, 0xC2); 
			break;
		case 750:
			axp_write(chip->client,CHARGE_REG1, 0xC3); 
			break;
		case 900:
			axp_write(chip->client,CHARGE_REG1, 0xC4); 
			break;
		case 1050:
			axp_write(chip->client,CHARGE_REG1, 0xC5); 
			break;
		case 1200:
			axp_write(chip->client,CHARGE_REG1, 0xC6); 
			break;
		case 1350:
			axp_write(chip->client,CHARGE_REG1, 0xC7); 
			break;
		case 1500:
			axp_write(chip->client,CHARGE_REG1, 0xC7); 
			break;
		case 1650:
			axp_write(chip->client,CHARGE_REG1, 0xC9); 
			break;
		case 1800:
			axp_write(chip->client,CHARGE_REG1, 0xCA); 
			break;
		case 1950:
			axp_write(chip->client,CHARGE_REG1, 0xCB); 
			break;
		case 2100:
			axp_write(chip->client,CHARGE_REG1, 0xCC); 
			break;
		case 2250:
			axp_write(chip->client,CHARGE_REG1, 0xCD); 
			break;
		case 2400:
			axp_write(chip->client,CHARGE_REG1, 0xCE); 
			break;
		case 2550:
			axp_write(chip->client,CHARGE_REG1, 0xCF); 
			break;
		default:
			printk("No Such Charge Current Setting,Please Check!\n");
			break;			
	}
}
EXPORT_SYMBOL_GPL(axp229_set_charge_current);

void axp229_set_arm_voltage(unsigned int target_freq)
{
    	if(axp_probe_status==0)
		return;
	if(target_freq<1000000)
		axp_write(chip->client,DCDC2, 0x18);
	else if(target_freq<1500000)
		axp_write(chip->client,DCDC2, 0x20);
	else
		axp_write(chip->client,DCDC2, 0x25);

}
EXPORT_SYMBOL_GPL(axp229_set_arm_voltage);

void axp229_set_core_voltage()
{
    uint8_t axp_reg;

    axp_read(chip->client,DCDC2, &axp_reg);
    printk("%s:reg value 0x%08x\n",__func__,axp_reg);

    axp_read(chip->client,DCDC3, &axp_reg);
    printk("%s:reg value 0x%08x\n",__func__,axp_reg);

    axp_write(chip->client, DCDC2, 0x20);

}

void axp229_set_voltage(uint8_t ldo,unsigned int mv,uint8_t onoff)
{
    uint8_t axp_reg;
    if(axp_probe_status==0)
	return;

	switch(ldo)
	{
	case DLDO2:
	            if(mv>0)
                    axp_write(chip->client, DLDO2, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg | 0x10); 
                }
                else
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg & 0xef); 
                }                               
		break;
	case DLDO3:
	            if(mv>0)
                    axp_write(chip->client, DLDO3, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg | 0x20); 
                }
                else
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg & 0xdf); 
                }                       
		break;
	case DLDO4:
	            if(mv>0)
                    axp_write(chip->client, DLDO4, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg | 0x40); 
                }
                else
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg & 0xbf); 
                }                       
		break;
	case ELDO2:
	            if(mv>0)
                    axp_write(chip->client, ELDO2, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg | 0x02); 
                }
                else
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg & 0xfd); 
                }                       
		break;
	case ELDO3:
	            if(mv>0)
                    axp_write(chip->client, ELDO3, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg | 0x04); 
                }
                else
                {
                    axp_read(chip->client,LDO2_EN, &axp_reg);
                    axp_write(chip->client,LDO2_EN, axp_reg & 0xfb); 
                }                       
		break;
	case ALDO1:
	            if(mv>0)
                    axp_write(chip->client, ALDO1, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,LDO1_EN, &axp_reg);
                    axp_write(chip->client,LDO1_EN, axp_reg | 0x40); 
                }
                else
                {
                    axp_read(chip->client,LDO1_EN, &axp_reg);
                    axp_write(chip->client,LDO1_EN, axp_reg & 0xbf); 
                }                       
		break;		
	case ALDO3:
	            if(mv>0)
                    axp_write(chip->client, ALDO3, (mv/100-7));
                if(onoff)
                {
                    axp_read(chip->client,ALDO3_EN, &axp_reg);
                    axp_write(chip->client,ALDO3_EN, axp_reg | 0x80); 
                }
                else
                {
                    axp_read(chip->client,ALDO3_EN, &axp_reg);
                    axp_write(chip->client,ALDO3_EN, axp_reg & 0x00); 
                }                       
		break;	
	case LDOIO0:
                if(mv>0)
                    axp_write(chip->client, LDOIO0, (mv/100-7));
                if(onoff)
                {
                    axp_write(chip->client,GPIO0, 0x03); 
                }
                else
                {
                    axp_write(chip->client,GPIO0, 0x04); 
                }                               
		break;												
	default:
		printk("No Such LDO Setting,Please Check!\n");
		break;
	}
}
EXPORT_SYMBOL_GPL(axp229_set_voltage);

static void axp229_work(struct work_struct *work)
{
	chip = container_of(work, struct axp229_chip, work.work);
	axp229_get_status();
	axp229_get_soc();
	power_supply_changed(&chip->battery);
	schedule_delayed_work(&chip->work, msecs_to_jiffies(2000));
}

static enum power_supply_property axp229_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property axp229_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE, 
};

static void axp_battery(void)
{
    int Cur_CoulombCounter;
    uint8_t ocv_cap[63];

    ocv_cap[0]  = OCVREG0;
    ocv_cap[1]  = 0xC1;
    ocv_cap[2]  = OCVREG1;
    ocv_cap[3]  = 0xC2;
    ocv_cap[4]  = OCVREG2;
    ocv_cap[5]  = 0xC3;
    ocv_cap[6]  = OCVREG3;
    ocv_cap[7]  = 0xC4;
    ocv_cap[8]  = OCVREG4;
    ocv_cap[9]  = 0xC5;
    ocv_cap[10] = OCVREG5;
    ocv_cap[11] = 0xC6;
    ocv_cap[12] = OCVREG6;
    ocv_cap[13] = 0xC7;
    ocv_cap[14] = OCVREG7;
    ocv_cap[15] = 0xC8;
    ocv_cap[16] = OCVREG8;
    ocv_cap[17] = 0xC9;
    ocv_cap[18] = OCVREG9;
    ocv_cap[19] = 0xCA;
    ocv_cap[20] = OCVREGA;
    ocv_cap[21] = 0xCB;
    ocv_cap[22] = OCVREGB;
    ocv_cap[23] = 0xCC;
    ocv_cap[24] = OCVREGC;
    ocv_cap[25] = 0xCD;
    ocv_cap[26] = OCVREGD;
    ocv_cap[27] = 0xCE;
    ocv_cap[28] = OCVREGE;
    ocv_cap[29] = 0xCF;
    ocv_cap[30] = OCVREGF;
    ocv_cap[31] = 0xD0;
    ocv_cap[32] = OCVREG10;
    ocv_cap[33] = 0xD1;
    ocv_cap[34] = OCVREG11;
    ocv_cap[35] = 0xD2;
    ocv_cap[36] = OCVREG12;
    ocv_cap[37] = 0xD3;
    ocv_cap[38] = OCVREG13;
    ocv_cap[39] = 0xD4;
    ocv_cap[40] = OCVREG14;
    ocv_cap[41] = 0xD5;
    ocv_cap[42] = OCVREG15;
    ocv_cap[43] = 0xD6;
    ocv_cap[44] = OCVREG16;
    ocv_cap[45] = 0xD7;
    ocv_cap[46] = OCVREG17;
    ocv_cap[47] = 0xD8;
    ocv_cap[48] = OCVREG18;
    ocv_cap[49] = 0xD9;
    ocv_cap[50] = OCVREG19;
    ocv_cap[51] = 0xDA;
    ocv_cap[52] = OCVREG1A;
    ocv_cap[53] = 0xDB;
    ocv_cap[54] = OCVREG1B;
    ocv_cap[55] = 0xDC;
    ocv_cap[56] = OCVREG1C;
    ocv_cap[57] = 0xDD;
    ocv_cap[58] = OCVREG1D;
    ocv_cap[59] = 0xDE;
    ocv_cap[60] = OCVREG1E;
    ocv_cap[61] = 0xDF;
    ocv_cap[62] = OCVREG1F;
    axp_writes(chip->client, 0xC0,63,ocv_cap);


    Cur_CoulombCounter = BATCAP * 1000 / 1456;
    axp_write(chip->client, AXP22_BATCAP0, ((Cur_CoulombCounter >> 8) | 0x80));
    axp_write(chip->client,AXP22_BATCAP1,Cur_CoulombCounter & 0x00FF);		
}

static int __devinit axp229_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	uint8_t axp_reg;
	axp_probe_status=0;
	int ret,err;

    	gpio_request(EXYNOS4_GPX3(2), "GPX3");
    	gpio_direction_output(EXYNOS4_GPX3(2), 1); 
    	gpio_free(EXYNOS4_GPX3(2));

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
    	
	chip->client = client;
	charging_source=0;
	i2c_set_clientdata(client, chip);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= axp229_get_property;
	chip->battery.properties	= axp229_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(axp229_battery_props);

	chip->ac.name		= "ac";
	chip->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	chip->ac.get_property	= axp229_get_property;
	chip->ac.properties	= axp229_power_props;
	chip->ac.num_properties	= ARRAY_SIZE(axp229_power_props);

	chip->usb.name		= "usb";
	chip->usb.type		= POWER_SUPPLY_TYPE_USB;
	chip->usb.get_property	= axp229_get_property;
	chip->usb.properties	= axp229_power_props;
	chip->usb.num_properties	= ARRAY_SIZE(axp229_power_props);

	chip->soc = 100; //100%

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power bat supply register\n");
		kfree(chip);
		return ret;
	}

	ret = power_supply_register(&client->dev, &chip->ac);
	if (ret) {
		dev_err(&client->dev, "failed: power ac supply register\n");
		kfree(chip);
		return ret;
	}

	ret = power_supply_register(&client->dev, &chip->usb);
	if (ret) {
		dev_err(&client->dev, "failed: power usb supply register\n");
		kfree(chip);
		return ret;
	}

	axp_probe_status=1;

    axp229_set_core_voltage();

	//gpx0_3为vbus_det，otg接口充电检测
    /*
	gpio_request(EXYNOS4_GPX0(3), "GPX0");
	s3c_gpio_cfgpin(EXYNOS4_GPX0(3), S3C_GPIO_SFN(0x0f));
	s3c_gpio_setpull(EXYNOS4_GPX0(3), S3C_GPIO_PULL_NONE);
	request_irq(IRQ_EINT(3), charger_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "charger-irq", NULL);
    */

    //开启各个外设的电源
	axp_write(chip->client, DLDO3, (2800/100-7));//touch screen 2.8V
    axp_write(chip->client, ALDO1, (3000/100-7));//wifi and bt  3.0v
    axp_write(chip->client, ELDO2, (1500/100-7));//camera 1.5V    
    axp_write(chip->client, ELDO3, (2800/100-7));//CAM 2.8V   
    axp_write(chip->client, DLDO4, (1800/100-7));//AUDIO 1.8V
    /*
    axp_write(chip->client, ALDO3, (1800/100-7));//sensor 1.8V
    */
    //axp_write(chip->client, DLDO3, (3300/100-7));//AUDIO 3.3V
    //axp_write(chip->client, 0x36, 0x68);  //6S POWER OFF 99
	
	axp229_set_voltage(DLDO3,2800,1);
	axp229_set_voltage(ALDO1,3000,1);
	axp229_set_voltage(ELDO2,1500,1);
	axp229_set_voltage(ELDO3,2800,1);
 	axp229_set_voltage(DLDO4,1800,1);  
	axp229_set_voltage(ALDO3,3300,1);          
    	/*
	axp229_set_voltage(ALDO3,1800,1);
    	*/
	//axp229_set_voltage(DLDO3,3300,1);
	
    	//gpio0配置为ldo，1.8v供电camera
    	axp_write(chip->client, GPIO0, 0x03);   
    	axp_write(chip->client, LDOIO0, (1800/100-7));//CAM 1.8V   
	axp229_set_voltage(LDOIO0,1800,1);
	
    //开启audio的电源
    //gpio_request(EXYNOS4212_GPM2(1), "GPM2_1");
    //gpio_direction_output(EXYNOS4212_GPM2(1), 1);
    //gpio_free(EXYNOS4212_GPM2(1));

    //开启USB PHY的电源
    err = gpio_request_one(EXYNOS4212_GPM2(0),GPIOF_OUT_INIT_LOW, "GPM2");
	if (err)
		printk(KERN_ERR "failed to request GPM2_0 ####\n");
    gpio_set_value(EXYNOS4212_GPM2(0), 1);
    gpio_free(EXYNOS4212_GPM2(0));


#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->early_suspend.suspend = axp229_early_suspend;
	chip->early_suspend.resume = axp229_late_resume;
	register_early_suspend(&chip->early_suspend);
#endif

    axp_write(chip->client, 0x40, 0x00); 
    axp_write(chip->client, 0x41, 0x00); 
    axp_write(chip->client, 0x42, 0x03); 
    axp_write(chip->client, 0x43, 0x00); 
    axp_write(chip->client, 0x44, 0x00); 
    axp_write(chip->client,DCDC3, 0x1A);

    //配置某些组电源受控于pwren引脚
    //pwren1 :   bit7 -- bit0   <->    dcdc1/dcdc2/dcdc3/dcdc4/dcdc5/aldo1/aldo2/aldo3
    //pwren2 :   bit7 -- bit0   <->    dldo1/dldo2/dldo3/dldo4/eldo1/eldo2/eldo3/dc5ldo
    //axp_write(chip->client, PWREN1, 0x72);  
    //axp_write(chip->client, PWREN2, 0x84); 
    axp_write(chip->client, GPIO1, 0x82);  
    axp_write(chip->client, PWREN1, 0x77);  
    axp_write(chip->client, PWREN2, 0xF6); 

    //打开电量计
    axp_write(chip->client, 0xB8, 0xF0); 

    axp_read(chip->client,  AXP22_HOTOVER_CTL, &axp_reg);//over temp power off,N_VBUSEN used
    axp_write(chip->client, AXP22_HOTOVER_CTL, axp_reg | 0x14);  

    axp_read(chip->client,  AXP22_VOFF_SET, &axp_reg);//less than 3.3V power off
    axp_write(chip->client, AXP22_VOFF_SET, axp_reg | 0x07);  

    axp_read(chip->client,  AXP22_IPS_SET, &axp_reg);//ips out select
    axp_write(chip->client, AXP22_IPS_SET, axp_reg & 0x7F);  

	axp229_set_charge_current(600);
    axp_read (chip->client,POWER_OFF_REG, &axp_reg);
    axp_write(chip->client,POWER_OFF_REG, axp_reg | 0x08); 

    //USB HUB_CONNECT引脚
	gpio_request(EXYNOS4_GPX0(2), "GPX0");
	s3c_gpio_setpull(EXYNOS4_GPX0(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX0(2), 1);
	gpio_free(EXYNOS4_GPX0(2)); 

    //建立更新电池状态的工作队列
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, axp229_work);
	schedule_delayed_work(&chip->work, 200);
	
	printk("SCV4412 AXP211 Probe OK\n");
	return 0;
}

static int __devexit axp229_remove(struct i2c_client *client)
{
	struct axp229_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int axp229_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct axp229_chip *chip = i2c_get_clientdata(client);
	axp229_set_voltage(ALDO3,3300,0);
	gpio_request(EXYNOS4_GPX0(2), "GPX0");
	gpio_direction_output(EXYNOS4_GPX0(2), 0);
	gpio_free(EXYNOS4_GPX0(2));
	axp229_set_charge_current(1800);
	cancel_delayed_work(&chip->work);
	return 0;
}

static int axp229_resume(struct i2c_client *client)
{
	struct axp229_chip *chip = i2c_get_clientdata(client);
	axp229_set_charge_current(600);
	schedule_delayed_work(&chip->work, 1000);
	gpio_request(EXYNOS4_GPX0(2), "GPX0");
	gpio_direction_output(EXYNOS4_GPX0(2), 1);
	gpio_free(EXYNOS4_GPX0(2));
	axp229_set_voltage(ALDO3,3300,1);
	return 0;
}

#else

#define axp229_suspend NULL
#define axp229_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id axp229_id[] = {
	{ "axp229", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axp229_id);

static struct i2c_driver axp229_i2c_driver = {
	.driver	= {
		.name	= "axp229",
	},
	.probe		= axp229_probe,
	.remove		= __devexit_p(axp229_remove),
	.suspend	= axp229_suspend,
	.resume		= axp229_resume,
	.id_table	= axp229_id,
};

static int __init axp229_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
    wake_lock(&vbus_wake_lock);

	return i2c_add_driver(&axp229_i2c_driver);
}
module_init(axp229_init);

static void __exit axp229_exit(void)
{
	axp229_power_off();
	i2c_del_driver(&axp229_i2c_driver);
}
module_exit(axp229_exit);

MODULE_AUTHOR("oscar<jasonel@qq.com>");
MODULE_DESCRIPTION("axp pmic driver");
MODULE_LICENSE("GPL");
