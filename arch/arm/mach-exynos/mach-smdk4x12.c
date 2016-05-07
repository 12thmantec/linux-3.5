/* linux/arch/arm/mach-exynos/mach-smdk4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/clk.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max77686.h>
#include <linux/v4l2-mediabus.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/smsc911x.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/axp229.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <linux/switch.h>
#include <plat/regs-serial.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/keypad.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/backlight.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <plat/iic.h>
#include <plat/sdhci.h>
#include <plat/ehci.h>
#include <media/s5p_fimc.h>
#include <plat/mipi_csis.h>
#include <plat/regs-srom.h>
#include <plat/sysmmu.h>
#include <plat/tv-core.h>
#include <plat/mfc.h>
#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>
#include <video/platform_lcd.h>
#include <mach/map.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>
#include <mach/dwmci.h>
#include <mach/dev.h>
#include <mach/ppmu.h>
#include <plat/fimg2d.h>
#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>
#include <linux/cma.h>
#include "common.h"
#include <mach/ohci.h>

#define GPIO_HUB_RESET EXYNOS4X12_GPM3(2)
#define GPIO_HUB_CONNECT EXYNOS4_GPX0(2)
#define GPIO_HUB_INT EXYNOS4_GPX2(3)
#include <media/ut2055_platform.h>
#define REG_INFORM4            (S5P_INFORM4)

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK4X12_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK4X12_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK4X12_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk4x12_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
};
static int camera_powerctrl(int onoff)
{

	if (onoff) 
	{
		axp229_set_voltage(LDOIO0,1800,1);
		axp229_set_voltage(ELDO2,1500,1);
		axp229_set_voltage(ELDO3,2800,1);
    		gpio_request(EXYNOS4X12_GPM1(3), "GPM1");
    		gpio_direction_output(EXYNOS4X12_GPM1(3), 0);
    		gpio_free(EXYNOS4X12_GPM1(3));
		mdelay(10);
    		gpio_request(EXYNOS4X12_GPM1(3), "GPM1");
    		gpio_direction_output(EXYNOS4X12_GPM1(3), 1);
    		gpio_free(EXYNOS4X12_GPM1(3));
	}
	else
	{
		axp229_set_voltage(LDOIO0,1800,0);
		axp229_set_voltage(ELDO2,1500,0);
		axp229_set_voltage(ELDO3,2800,0);
    		gpio_request(EXYNOS4X12_GPM2(4), "GPM2_1");
    		gpio_direction_output(EXYNOS4X12_GPM2(4), 0);
    		gpio_free(EXYNOS4X12_GPM2(4));
    		gpio_request(EXYNOS4X12_GPM1(3), "GPM1");
    		gpio_direction_output(EXYNOS4X12_GPM1(3), 0);
    		gpio_free(EXYNOS4X12_GPM1(3));
	}

	return 0;
}

static struct ut2055_platform_data ut2055_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 24000000,
	.is_mipi = 0,
};
static struct i2c_board_info ut2055_i2c_info = {
	I2C_BOARD_INFO("UT2055", 0x24), //0x24是HM2057的I2C地址
	.platform_data = &ut2055_plat,
};

static struct s3c_platform_camera ut2055 = {
	.id		= CAMERA_PAR_A,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &ut2055_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1600,
	.height		= 1200,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1600,
		.height	= 1200,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 1,
	.initialized	= 0,
	.cam_power	= camera_powerctrl,
};


/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
	.camera		= {	
            &ut2055,
            &ut2055
	},
	.hw_ver		= 0x51,
};

static int exynos4_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd)
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;

	__raw_writel(mode, REG_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block exynos4_reboot_notifier = {
	.notifier_call = exynos4_notifier_call,
};

static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};

static struct s3c_sdhci_platdata smdk4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};

static struct s3c_sdhci_platdata smdk4x12_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};

static struct s5p_ehci_platdata smdk4x12_ehci_pdata;

static void __init smdk4x12_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdk4x12_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}

static struct exynos4_ohci_platdata origen_ohci_pdata;

static void __init smdk4x12_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &origen_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}
#if 0
/* USB GADGET */
static struct s5p_usbgadget_platdata smdk4x12_usbgadget_pdata;

static void __init smdk4x12_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &smdk4x12_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);
}
#endif

static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};

static struct regulator_consumer_supply wm8994_fixed_voltage0_supplies[] = {
	REGULATOR_SUPPLY("AVDD2", "1-001a"),
	REGULATOR_SUPPLY("CPVDD", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", "1-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage2_supplies =
	REGULATOR_SUPPLY("DBVDD", "1-001a");

static struct regulator_init_data wm8994_fixed_voltage0_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage0_supplies),
	.consumer_supplies	= wm8994_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage1_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage1_supplies),
	.consumer_supplies	= wm8994_fixed_voltage1_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage2_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_fixed_voltage2_supplies,
};

static struct fixed_voltage_config wm8994_fixed_voltage0_config = {
	.supply_name	= "VDD_1.8V",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage1_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage1_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage2_config = {
	.supply_name	= "VDD_3.3V",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage2_init_data,
};

static struct platform_device wm8994_fixed_voltage0 = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage0_config,
	},
};

static struct platform_device wm8994_fixed_voltage1 = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage1_config,
	},
};

static struct platform_device wm8994_fixed_voltage2 = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage2_config,
	},
};

static struct regulator_consumer_supply wm8994_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", "1-001a");

static struct regulator_consumer_supply wm8994_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", "1-001a");

static struct regulator_init_data wm8994_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_dcvdd_supply,
};

static struct wm8994_pdata wm8994_platform_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* If the i2s0 and i2s2 is enabled simultaneously */
	.gpio_defaults[7] = 0x8100, /* GPIO8  DACDAT3 in */
	.gpio_defaults[8] = 0x0100, /* GPIO9  ADCDAT3 out */
	.gpio_defaults[9] = 0x0100, /* GPIO10 LRCLK3  out */
	.gpio_defaults[10] = 0x0100,/* GPIO11 BCLK3   out */
	.ldo[0] = { 0, &wm8994_ldo1_data },
	.ldo[1] = { 0, &wm8994_ldo2_data },
};

static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("axp229", 0x68>> 1),
	},
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("wm8994", 0x1a),
		.platform_data	= &wm8994_platform_data,
	},
};

static struct i2c_board_info i2c_devs2[] __initdata = {
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
};

static struct i2c_board_info i2c_devs3[] __initdata = {
    { I2C_BOARD_INFO("ft5x0x_ts", 0x38),   },
};

static struct i2c_board_info i2c_devs4[] __initdata = {
    {   
        I2C_BOARD_INFO("wm8978", 0x1a),
    },
};

static struct i2c_board_info i2c_devs5[] __initdata = {
    {   
        I2C_BOARD_INFO("mma865x",    0x1D),
    },

};

static struct i2c_board_info i2c_devs7[] __initdata = {

};

static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 250 * 1000000,	/* 200 Mhz */
};

static struct s5p_usbswitch_platdata smdk4x12_usbswitch_pdata;

static void __init smdk4x12_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &smdk4x12_usbswitch_pdata;
	int err;

    /* host detect是指exynos 4412自己作为host端，用于连接U盘等usb外设 */
	pdata->gpio_host_detect = EXYNOS4_GPX3(5); /* low active */
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN, "HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);

    /* device detect是指exynos 4412自己作为device端，当作外设连接到PC */
	pdata->gpio_device_detect = EXYNOS4_GPX3(4); /* high active */
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN, "DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect for\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);


    /* 现在的vbus连接到了exynos 4412的otg功能接口上，没有连接到任何gpio接口 */
    /*
	pdata->gpio_host_vbus = EXYNOS4_GPX1(6);
	gpio_request(EXYNOS4_GPX1(6), "GPX1");
	s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX1(6));
    */

    /*
    pdata->gpio_host_vbus = EXYNOS4_GPL2(0);
    err = gpio_request_one(pdata->gpio_host_vbus, GPIOF_OUT_INIT_LOW, "HOST_VBUS_CONTROL");
    if (err) {
        printk(KERN_ERR "failed to request gpio_host_vbus\n");
        return;
    }

    s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
    gpio_free(pdata->gpio_host_vbus);
    printk(KERN_ERR "gpio host vbus: 0x%x\n",pdata->gpio_host_vbus);
    */

	s5p_usbswitch_set_platdata(pdata);
}

static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};

static struct platform_device *smdk4412_devices[] __initdata = {
	&s3c_device_adc,
};

static struct platform_device totops_rfid = {
	.id = -1,
	.name = "totops-rfid",
};

static struct gpio_switch_platform_data headset_switch_data = {
    .name = "h2w",
    .gpio = EXYNOS4_GPX2(6),
};

static struct resource switch_gpio_resource[] = {
        [0] = {
                .start  = IRQ_EINT(22),
                .end    = IRQ_EINT(22),
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device headset_switch_device = {
    .name = "switch-gpio",
    .dev = {
        .platform_data = &headset_switch_data,
    },
    .num_resources  = ARRAY_SIZE(switch_gpio_resource),
    .resource = switch_gpio_resource,
};

static struct platform_device *smdk4x12_devices[] __initdata = {
#ifdef CONFIG_EXYNOS_DEV_PD
	/* Samsung Power Domain */
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_GPS_ALIVE],
#endif
/* mainline fimd */

	/* legacy fimd */
	//&s3c_device_fb,
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c7,
	&s5p_device_ehci,
	&exynos4_device_ohci,
	//&s3c_device_usbgadget,

	&s3c_device_hsmmc2,

	&s3c_device_hsmmc3,
	&exynos_device_dwmci,
	//&exynos_device_i2s0,

	//&exynos_device_srp,
	//&s5p_device_tvout,
	//&s5p_device_cec,
	//&s5p_device_hpd,

	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_mipi_csis0,
	&s5p_device_mipi_csis1,

	&mipi_csi_fixed_voltage,

	&s5p_device_mfc,
	//&SYSMMU_PLATDEV(g2d_acp),
	&SYSMMU_PLATDEV(fimc0),
	&SYSMMU_PLATDEV(fimc1),
	&SYSMMU_PLATDEV(fimc2),
	&SYSMMU_PLATDEV(fimc3),
	&SYSMMU_PLATDEV(jpeg),
	&SYSMMU_PLATDEV(mfc_l),
	&SYSMMU_PLATDEV(mfc_r),
	&SYSMMU_PLATDEV(tv),

	&s5p_device_fimg2d,

	&s5p_device_jpeg,
	&wm8994_fixed_voltage0,
	&wm8994_fixed_voltage1,
	&wm8994_fixed_voltage2,
	&samsung_asoc_dma,
	&samsung_asoc_idma,

	//&exynos_device_tmu,
	&exynos4_busfreq,
	&totops_rfid,
    &headset_switch_device,
};
#if 0
/* below temperature base on the celcius degree */
struct tmu_data exynos_tmu_data __initdata = {
	.ts = {
		.stop_throttle  = 82,
		.start_throttle = 85,
		.stop_warning  = 102,
		.start_warning = 105,
		.start_tripping = 110,		/* temp to do tripping */
		.start_hw_tripping = 113,	/* temp to do hw_trpping*/
		.stop_mem_throttle = 80,
		.start_mem_throttle = 85,

		.stop_tc = 13,
		.start_tc = 10,
	},
	.cpulimit = {
		.throttle_freq = 800000,
		.warning_freq = 200000,
	},
	.temp_compensate = {
		.arm_volt = 925000, /* vdd_arm in uV for temperature compensation */
		.bus_volt = 900000, /* vdd_bus in uV for temperature compensation */
		.g3d_volt = 900000, /* vdd_g3d in uV for temperature compensation */
	},
	.efuse_value = 55,
	.slope = 0x10008802,
	.mode = 0,
};
#endif

#if 0
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif


#if 0
static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] = {

		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0
		},
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0
		},
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0
		},
		{
			.name = "mfc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{ .alignment = 1 << 17 },
		},

		{
			.size = 0
		},
	};
	struct cma_region *regions_secure = NULL;

	static const char map[] __initconst =
		"s3cfb.0/fimd=fimd;exynos4-fb.0/fimd=fimd;"
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;exynos4210-fimc.2=fimc2;exynos4210-fimc.3=fimc3;"
		"s3c-mfc/A=mfc0,mfc-secure;"
		"s3c-mfc/B=mfc1,mfc-normal;"
		"s3c-mfc/AB=mfc;"
		"samsung-rp=srp;"
		"s5p-jpeg=jpeg;"
		"exynos4-fimc-is/f=fimc_is;"
		"s5p-mixer=tv;"
		"s5p-fimg2d=fimg2d;"
		"ion-exynos=ion,fimd,fimc0,fimc1,fimc2,fimc3,fw,b1,b2;"
		"s5p-smem/mfc=mfc0,mfc-secure;"
		"s5p-smem/fimc=fimc3;"
		"s5p-smem/mfc-shm=mfc1,mfc-normal;"
		"s5p-smem/fimd=fimd;";

	s5p_cma_region_reserve(regions, regions_secure, 0, map);
}
#endif
/* LCD Backlight data */
static struct samsung_bl_gpio_info smdk4x12_bl_gpio_info = {
	.no = EXYNOS4_GPD0(1),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk4x12_bl_data = {
	.pwm_id = 1,
	.pwm_period_ns  = 78770,
	.max_brightness = 255,
	.dft_brightness = 128,//128,
};

static void __init smdk4x12_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk4x12_uartcfgs, ARRAY_SIZE(smdk4x12_uartcfgs));

	//exynos4_reserve_mem();
}
static void __init smdk4x12_reserve(void)
{
	// HACK: This reserved memory will be used for FIMC-IS
	s5p_mfc_reserve_mem(0x58000000, 32<< 20, 0x43000000, 0 << 20);
}


static void __init exynos_sysmmu_init(void)
{
#ifdef CONFIG_EXYNOS_DEV_PD
	ASSIGN_SYSMMU_POWERDOMAIN(fimc0, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc1, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc2, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc3, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(jpeg, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_l, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_r, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(tv, &exynos4_device_pd[PD_TV].dev);

	sysmmu_set_owner(&SYSMMU_PLATDEV(g2d_acp).dev, &s5p_device_fimg2d.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_l).dev, &s5p_device_mfc.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_r).dev, &s5p_device_mfc.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s3c_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s3c_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s3c_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s3c_device_fimc3.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_tvout.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
#endif
}
#define ETC6PUD		(S5P_VA_GPIO2 + 0x228)
#define SMDK4412_REV_0_0_ADC_VALUE 0
#define SMDK4412_REV_0_1_ADC_VALUE 443
#define S5P_ADCCON_SELMUX(x) (((x)&0xF)<<0)
int samsung_board_rev;

static int get_samsung_board_rev(void)
{
	int 		adc_val = 0;
	struct clk	*adc_clk;
	struct resource	*res;
	void __iomem	*adc_regs;
	unsigned int	con;
	int		ret;

	writel((__raw_readl(ETC6PUD) & ~(0x3 << 6)) | (0x3 << 6),
		ETC6PUD);

	if ((soc_is_exynos4412() && samsung_rev() < EXYNOS4412_REV_1_0) ||
		(soc_is_exynos4212() && samsung_rev() < EXYNOS4412_REV_1_0))
		return EXYNOS4412_REV_0;

	adc_clk = clk_get(NULL, "adc");
	if (unlikely(IS_ERR(adc_clk)))
		return EXYNOS4412_REV_0;

	clk_enable(adc_clk);

	res = platform_get_resource(&s3c_device_adc, IORESOURCE_MEM, 0);
	if (unlikely(!res))
		goto err_clk;

	adc_regs = ioremap(res->start, resource_size(res));
	if (unlikely(!adc_regs))
		goto err_clk;

	writel(S5P_ADCCON_SELMUX(3), adc_regs + S5P_ADCMUX);

	con = readl(adc_regs + S3C2410_ADCCON);
	con &= ~S3C2410_ADCCON_MUXMASK;
	con &= ~S3C2410_ADCCON_STDBM;
	con &= ~S3C2410_ADCCON_STARTMASK;
	con |=  S3C2410_ADCCON_PRSCEN;

	con |= S3C2410_ADCCON_ENABLE_START;
	writel(con, adc_regs + S3C2410_ADCCON);

	udelay (50);

	adc_val = readl(adc_regs + S3C2410_ADCDAT0) & 0xFFF;
	writel(0, adc_regs + S3C64XX_ADCCLRINT);

	iounmap(adc_regs);
err_clk:
	clk_disable(adc_clk);
	clk_put(adc_clk);

	ret = (adc_val < SMDK4412_REV_0_1_ADC_VALUE/2) ?
			EXYNOS4412_REV_0 : EXYNOS4412_REV_0_1;

	pr_info ("SMDK MAIN Board Rev 0.%d (ADC value:%d)\n", ret, adc_val);
	return EXYNOS4412_REV_0_1;
}

//  off part GPIO Sleep Control table
//  {pin number,      sleep mode conf,    sleep pullup/down config}
static unsigned int sleep_off_gpio_table[][3] =
{
    {EXYNOS4X12_GPM1(5), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //LCD_BIST

    {EXYNOS4X12_GPM3(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //HUB_RST
    {EXYNOS4X12_GPM3(3), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //HUB_CONNECT

    {EXYNOS4X12_GPM4(1), S3C_GPIO_SLP_OUT0,  S3C_GPIO_PULL_NONE},  //LCD_BIST

    {EXYNOS4_GPD0(1), S3C_GPIO_SLP_OUT0,  S3C_GPIO_PULL_NONE},  //LCD_PWM

    //rfid uart1
    {EXYNOS4_GPA0(4), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //RFID_RXD
    {EXYNOS4_GPA0(5), S3C_GPIO_SLP_OUT0,  S3C_GPIO_PULL_NONE},  //RFID_TXD

    //debug uart2
    {EXYNOS4_GPA1(0), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //Debug_RX
    {EXYNOS4_GPA1(1), S3C_GPIO_SLP_OUT0,  S3C_GPIO_PULL_NONE},  //Debug_TX

    //gps uart3
    {EXYNOS4_GPA1(4), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //GPS_RXD
    {EXYNOS4_GPA1(5), S3C_GPIO_SLP_OUT0,  S3C_GPIO_PULL_NONE},  //GPS_TXD

    //i2c_0
    {EXYNOS4_GPD1(0), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA0
    {EXYNOS4_GPD1(1), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL0

    //i2c_1
    {EXYNOS4_GPD1(1), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA_PMU
    {EXYNOS4_GPD1(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL_PMU

    //i2c_2
    {EXYNOS4_GPA0(6), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA2
    {EXYNOS4_GPA0(7), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL2

    //i2c_3
    {EXYNOS4_GPA1(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA3
    {EXYNOS4_GPA1(3), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL3

    //i2c_4
    {EXYNOS4_GPB(0), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA4
    {EXYNOS4_GPB(1), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL4

    //i2c_5
    {EXYNOS4_GPB(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA5
    {EXYNOS4_GPB(3), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL5

    //i2c_6
    //{EXYNOS4_GPB(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA5
    //{EXYNOS4_GPB(3), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL5

    //i2c_7
    {EXYNOS4_GPD0(2), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SDA5
    {EXYNOS4_GPD0(3), S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},  //I2C_SCL5

};

//  alive part GPIO Sleep Control table
//  {pin number,      sleep mode conf, sleep pin value,      sleep pullup/down config}
static unsigned int sleep_alive_gpio_table[][4] =
{
    {EXYNOS4_GPX2(3), S3C_GPIO_INPUT,   S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN}, //pwr hold
    {EXYNOS4_GPX3(5), S3C_GPIO_INPUT,   S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN}, //pwr hold

    {EXYNOS4_GPX1(3), S3C_GPIO_INPUT,   S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN}, //LIGHT_INT，光感接近中断
    {EXYNOS4_GPX1(5), S3C_GPIO_INPUT,   S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN}, //RFID_ICC，rfid刷卡中断
    {EXYNOS4_GPX1(5), S3C_GPIO_INPUT,   S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN}, //RFID_ICC，rfid刷卡中断
};

//to config the off part gpio to sleep mode
extern int s3c_gpio_slp_cfgpin(unsigned int pin, unsigned int config);
extern int s3c_gpio_slp_setpull_updown(unsigned int pin, unsigned int config);
static void config_sleep_off_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
    u32 i, gpio;

    for (i = 0; i < array_size; i++) {
        gpio = gpio_table[i][0];
        s3c_gpio_slp_cfgpin(gpio, gpio_table[i][1]);
        s3c_gpio_slp_setpull_updown(gpio, gpio_table[i][2]);
    }    
}

//to config the alive part gpio(gpx0 - gpx3)
static void config_gpio_table(int array_size, unsigned int (*gpio_table)[4])
{
    u32 i, gpio;
        
    for (i = 0; i < array_size; i++) {
        gpio = gpio_table[i][0];
        s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
        if (gpio_table[i][2] != S3C_GPIO_SETPIN_NONE)
            gpio_set_value(gpio, gpio_table[i][2]);
        s3c_gpio_setpull(gpio, gpio_table[i][3]);
    }
} 

void exynos4_sleep_gpio_table_set(void)
{
    //alive part gpio in sleep mode
    config_gpio_table(ARRAY_SIZE(sleep_alive_gpio_table), sleep_alive_gpio_table);

    //off part gpio in sleep mode
    config_sleep_off_gpio_table(ARRAY_SIZE(sleep_off_gpio_table), sleep_off_gpio_table);

    return;
}


static void __init smdk4x12_machine_init(void)
{

	pm_power_off = axp229_power_off;
	samsung_board_rev = get_samsung_board_rev();
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos_pd_disable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_ISP].dev);
#endif
	s3c_i2c0_set_platdata(NULL);

	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c4_set_platdata(NULL);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
	s3c_i2c5_set_platdata(NULL);
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	//s3cfb_set_platdata(NULL);

	//s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
	smdk4x12_ehci_init();
	smdk4x12_ohci_init();
	//smdk4x12_usbgadget_init();
	smdk4x12_usbswitch_init();

	samsung_bl_set(&smdk4x12_bl_gpio_info, &smdk4x12_bl_data);

	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);

	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);

	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);


	//exynos_tmu_set_platdata(&exynos_tmu_data);

	//s3c_fimc0_set_platdata(&fimc_plat);
	//s3c_fimc1_set_platdata(&fimc_plat);
	//s3c_fimc2_set_platdata(&fimc_plat);
	//s3c_fimc3_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;


	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
	exynos4_device_pd[PD_TV].dev.parent = &exynos4_device_pd[PD_LCD0].dev;

	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	if (samsung_rev() == EXYNOS4412_REV_2_0)
		exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 176000000);
	else
		exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 160000000);

	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
	if (soc_is_exynos4412() && samsung_rev() >= EXYNOS4412_REV_2_0)
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 220 * MHZ);
	else if ((soc_is_exynos4412() && samsung_rev() >= EXYNOS4412_REV_1_0))
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 200 * MHZ);
	else
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 267 * MHZ);
#endif
	s5p_fimg2d_set_platdata(&fimg2d_data);

	exynos_sysmmu_init();

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	if (soc_is_exynos4412())
		platform_add_devices(smdk4412_devices, ARRAY_SIZE(smdk4412_devices));

	register_reboot_notifier(&exynos4_reboot_notifier);
   	//开启audio的电源
    	gpio_request(EXYNOS4X12_GPM2(1), "GPM2_1");
	s3c_gpio_setpull(EXYNOS4X12_GPM2(1), S3C_GPIO_PULL_UP);
	s3c_gpio_slp_setpull_updown(EXYNOS4X12_GPM2(1), S3C_GPIO_PULL_UP);
	s3c_gpio_slp_cfgpin(EXYNOS4X12_GPM2(1), S3C_GPIO_SLP_OUT0);
    	gpio_direction_output(EXYNOS4X12_GPM2(1), 1);
    	gpio_free(EXYNOS4X12_GPM2(1));
	gpio_request(EXYNOS4_GPX0(0), "GPX0");
	gpio_direction_output(EXYNOS4_GPX0(0), 1);
	gpio_free(EXYNOS4_GPX0(0));

    	//复位usb phy
    	gpio_request(GPIO_HUB_RESET, "GPIO_HUB_RESET");
   	gpio_direction_output(GPIO_HUB_RESET, 1);
    	s3c_gpio_setpull(GPIO_HUB_RESET, S3C_GPIO_PULL_NONE);
    	gpio_free(GPIO_HUB_RESET);

    	gpio_request(GPIO_HUB_CONNECT, "GPIO_HUB_CONNECT");
    	gpio_direction_output(GPIO_HUB_CONNECT, 1);
    	s3c_gpio_setpull(GPIO_HUB_CONNECT, S3C_GPIO_PULL_NONE);
    	gpio_free(GPIO_HUB_CONNECT);

    	//开启USB PHY的电源
    	gpio_request_one(EXYNOS4X12_GPM2(0),GPIOF_OUT_INIT_LOW, "GPM2");
    	gpio_set_value(EXYNOS4X12_GPM2(0), 1);
    	gpio_free(EXYNOS4X12_GPM2(0));
        //开启audio codec的电源
        gpio_request(EXYNOS4_GPX1(4), "GPX1");
        gpio_direction_output(EXYNOS4_GPX1(4), 1);
        gpio_free(EXYNOS4_GPX1(4));
        gpio_request(EXYNOS4_GPX0(6), "GPX0");
        gpio_direction_output(EXYNOS4_GPX0(6), 1);
        gpio_free(EXYNOS4_GPX0(6));

}

MACHINE_START(SMDK4412, "SMDK4412")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	/* Maintainer: Changhwan Youn <chaos.youn@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk4x12_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &smdk4x12_reserve,
MACHINE_END
