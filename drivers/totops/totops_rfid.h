/*
 * linux/drivers/urbetter/totops_rfid.h
 *
 * RFID measurement code declaration for TOTOPS smdk platform.
 *
 * Copyright (C) 2013 Totops Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __TOTOPS_RFID_H__
#define __TOTOPS_RFID_H__

typedef struct rfid_properties_t
{
	/* judge if there's a card */
	int icc;

	/* controlled by UART or ICC. xXx:not used now */
	int sps;

	/* power status */
	int ce;

    /* power of gps */
    int gps_power;

    int email_led;
    int wifi_led;
} rfid_properties;

struct rfid_device_t;

typedef struct rfid_ops_t
{
	/* update status. TODO:really needed? */
	void (*update_status)(struct rfid_device_t *);

	/* check icc status. TRUE:there's a card, FALSE:no card */
	int (*check_icc)(struct rfid_device_t *);

	/* check ce status. */
	int (*check_ce)(struct rfid_device_t *);
} rfid_ops;

typedef struct rfid_device_t
{
	struct device dev;

	rfid_properties props;

	// is update_lock needed?
	struct mutex ops_lock;
	rfid_ops *ops;
} rfid_device;

typedef enum rfid_update_reason_e
{
	RFID_UPDATE_SYSFS=0,	// sysfs update msg
	RFID_UPDATE_KEY,		// keyborad update msg
	RFID_CARD_INCOMING,		// detect RFID card incoming.
	RFID_UPDATE_NUM
} rfid_update_reason;

typedef enum
{
	RFID_CE=0,
	RFID_ICC,
	RFID_SPS,
    GPS_POWER,
    EMAIL_LED,
    WIFI_LED
} RFID_PROPERTY;

typedef struct rfid_device_info_s
{
	/* judge if there's a card */
	int icc;

	/* controlled by UART or ICC. NOTE:not used now */
	int sps;

	/* power status */
	int ce;

    /* power of gps */
    int gps_power;

    int email_led;
    int wifi_led;
} rfid_device_info;

#define to_rfid_device(member) container_of(member, rfid_device, dev)

#define TOTOPS_RFID_ATTR(_name)			\
{							\
	.attr = { .name = #_name, \
			  .mode = (S_IRUGO | S_IWUSR),}, \
	.show = totops_rfid_show_property,		\
	.store = totops_rfid_store_property,	\
}

#endif

