/*
 * driver/misc/rt8973880.c - rt8973 micro USB switch device driver
 *
 * Copyright (C) 2013 Samsung Electronics
 * Deukkyu Oh <deukkyu.oh@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c/rt8973.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pmic8058.h>
#include <linux/input.h>
#include <linux/switch.h>
//#include <linux/sii9234.h>
#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define INT_MASK1					0x0
#define INT_MASK2					0x0

/* DEVICE ID */
#define fsa880_DEV_ID				0x0A
#define fsa880_DEV_ID_REV			0x12

/* fsa880 I2C registers */
#define REG_DEVICE_ID				0x01
#define REG_CONTROL				0x02
#define REG_INT1					0x03
#define REG_ADC					0x07
#define REG_DEVICE_TYPE1			0x0a
#define REG_DEVICE_TYPE2			0x0b
#define REG_MANUAL_SW1			0x13
#define REG_MANUAL_SW2			0x14
#define REG_RESET			0x1B


#define DATA_NONE					0x00

/* Control */
#define CON_SWITCH_OPEN		(0 << 4)
#define CON_MANUAL_SW		(1 << 2)
#define CON_INT_MASK		(1 << 0)
#define CON_MASK		(CON_SWITCH_OPEN | CON_MANUAL_SW)

/* Device Type 1 */
#define DEV_DEDICATED_CHG		(1 << 6)
#define DEV_USB_CHG			(1 << 5)
#define DEV_CAR_KIT				(1 << 4)
#define DEV_USB				(1 << 2)

#define DEV_T1_USB_MASK		(DEV_USB_CHG | DEV_USB)
#define DEV_T1_CHARGER_MASK	(DEV_DEDICATED_CHG | DEV_CAR_KIT)

/* Device Type 2 */
#define DEV_JIG_UART_OFF			(1 << 3)
#define DEV_JIG_UART_ON			(1 << 2)
#define DEV_JIG_USB_OFF			(1 << 1)
#define DEV_JIG_USB_ON			(1 << 0)

#define DEV_T2_USB_MASK		(DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define DEV_T2_UART_MASK		(DEV_JIG_UART_OFF)
#define DEV_T2_JIG_MASK			(DEV_JIG_USB_OFF | DEV_JIG_USB_ON | DEV_JIG_UART_OFF)
#define DEV_T2_JIG_ALL_MASK		(DEV_JIG_USB_OFF | DEV_JIG_USB_ON | DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

/*
 * Manual Switch
 * D- [7:5] / D+ [4:2]
 * 000: Open all / 001: USB /  011: UART / 
 */
#define SW_UART		((3 << 5) | (3 << 2))
#define SW_USB		((1 << 5) | (1 << 2) | (1 << 0))
#define SW_AUTO		((0 << 5) | (0 << 2))
#define SW_USB_OPEN	(1 << 0)
#define SW_ALL_OPEN	(0)

/* Interrupt 1 */
#define INT_DETACH				(1 << 1)
#define INT_ATTACH				(1 << 0)

/* ADC VALUE */
#if 0 //KBJ temp block ( this source is based on fsa9280, but following value doesn't match fsa880 )
#define	ADC_MHL					0x01
#define 	ADC_SMART_DOCK			0x10
#define 	ADC_AUDIO_DOCK			0x12
#define	ADC_DESKDOCK			0x1a
#endif

#define	ADC_JIG_USB_OFF			0x18
#define	ADC_JIG_USB_ON			0x19
#define	ADC_JIG_UART_OFF			0x1c
#define	ADC_JIG_UART_ON			0x1d
#define	ADC_CARDOCK				0x1d
#define	ADC_OPEN					0x1f

extern unsigned int system_rev;

int rt_uart_connecting;
EXPORT_SYMBOL(rt_uart_connecting);
int rt_detached_status;
EXPORT_SYMBOL(rt_detached_status);

static int jig_state;


struct rt8973_usbsw {
	struct i2c_client		*client;
	struct rt8973_platform_data	*pdata;
	int				dev1;
	int				dev2;	
	int				mansw;
	int				dock_attached;
	int				dev_id;

	struct delayed_work	init_work;
	struct mutex		mutex;
	int				adc;
};

static struct rt8973_usbsw *local_usbsw;

static int rt8973_write_reg(struct i2c_client *client, int reg, int val)
{
        int ret;
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0)
        {
                dev_err(&client->dev,
                        "%s, i2c write error %d\n",__func__, ret);
        }
        return ret;
}

static int rt8973_read_reg(struct i2c_client *client, int reg)
{
        int ret;
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0)
        {
                dev_err(&client->dev,
                        "%s, i2c read error %d\n",__func__, ret);
        }
        return ret;
}





static void rt8973_disable_interrupt(void)
{
	struct i2c_client *client = local_usbsw->client;
	int value, ret;

	value = i2c_smbus_read_byte_data(client, REG_CONTROL);
	value |= CON_INT_MASK;

	ret = i2c_smbus_write_byte_data(client, REG_CONTROL, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

}

static void rt8973_enable_interrupt(void)
{
	struct i2c_client *client = local_usbsw->client;
	int value, ret;

	value = i2c_smbus_read_byte_data(client, REG_CONTROL);
	value &= (~CON_INT_MASK);

	ret = i2c_smbus_write_byte_data(client, REG_CONTROL, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

}


static void rt8973_dock_control(struct rt8973_usbsw *usbsw,
	int dock_type, int state, int path)
{
	struct i2c_client *client = usbsw->client;
	struct rt8973_platform_data *pdata = usbsw->pdata;
	int ret;

	if (state) {
		usbsw->mansw = path;
		pdata->callback(dock_type, state);
		ret = i2c_smbus_write_byte_data(client, REG_MANUAL_SW1, path);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		ret = i2c_smbus_read_byte_data(client, REG_CONTROL);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		else {
			ret = i2c_smbus_write_byte_data(client,
					REG_CONTROL, ret & ~CON_MANUAL_SW);
		}
		if (ret < 0)
			dev_err(&client->dev, "%s: err %x\n", __func__, ret);
	} else {
		pdata->callback(dock_type, state);
		ret = i2c_smbus_read_byte_data(client, REG_CONTROL);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		ret = i2c_smbus_write_byte_data(client, REG_CONTROL,
			ret | CON_MANUAL_SW);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}
}

static void rt8973_reg_init(struct rt8973_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	unsigned int ctrl = CON_MASK;
	int ret;
#ifdef CONFIG_USB_SWITCH_RT8973
	u8 value;

	pr_info("rt8973_reg_init is called\n");

	 i2c_smbus_write_byte_data(client, REG_RESET, 0x1);
	msleep(10);

	value = i2c_smbus_read_byte_data(client, REG_CONTROL);

	ctrl = (value & (~0x1));
	ret =  i2c_smbus_write_byte_data(client, REG_CONTROL, ctrl);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
#else
	pr_info("fsa880_reg_init is called\n");

	usbsw->dev_id = i2c_smbus_read_byte_data(client, REG_DEVICE_ID);
	local_usbsw->dev_id = usbsw->dev_id;
	if (usbsw->dev_id < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, usbsw->dev_id);

	dev_info(&client->dev, " fsa880_reg_init dev ID: 0x%x\n",
			usbsw->dev_id);

	usbsw->mansw = i2c_smbus_read_byte_data(client, REG_MANUAL_SW1);
	if (usbsw->mansw < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, usbsw->mansw);

	if (usbsw->mansw)
		ctrl &= ~CON_MANUAL_SW;	/* Manual Switching Mode */
	else
		ctrl &= ~(CON_INT_MASK);

	ret = i2c_smbus_write_byte_data(client, REG_CONTROL, ctrl);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
#endif
}

static ssize_t rt8973_show_control(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int value;

	value = i2c_smbus_read_byte_data(client, REG_CONTROL);
	if (value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, value);

	return snprintf(buf, 13, "CONTROL: %02x\n", value);
}

static ssize_t rt8973_show_device_type(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
#if 0
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int value;

	value = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE1);
	if (value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, value);

	return snprintf(buf, 11, "DEV_TYP %02x\n", value);
#endif
	return 0;
}

static ssize_t rt8973_show_manualsw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#if 0
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int value;

	value = i2c_smbus_read_byte_data(client, REG_MANUAL_SW1);
	if (value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, value);

	if (value == SW_UART)
		return snprintf(buf, 5, "UART\n");
	else if (value == SW_USB)
		return snprintf(buf, 6, "USB\n");
	else if (value == SW_AUTO)
		return snprintf(buf, 5, "AUTO\n");
	else
		return snprintf(buf, 4, "%x", value);
#endif
	return 0;
}

static ssize_t rt8973_set_manualsw(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
#if 0
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int value, ret;
	unsigned int path = 0;

	value = i2c_smbus_read_byte_data(client, REG_CONTROL);
	if (value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, value);

	if ((value & ~CON_MANUAL_SW) != CON_SWITCH_OPEN )
		return 0;

	if (!strncmp(buf, "UART", 4)) {
		path = SW_UART;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "USB", 5)) {
		path = SW_USB;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "AUTO", 4)) {
		path = SW_AUTO;
		value |= CON_MANUAL_SW;
	} else {
		dev_err(dev, "Wrong command\n");
		return 0;
	}

	usbsw->mansw = path;

	ret = i2c_smbus_write_byte_data(client, REG_MANUAL_SW1, path);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	ret = i2c_smbus_write_byte_data(client, REG_CONTROL, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return count;
#endif
	return 0;
}
static ssize_t rt8973_show_usb_state(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int device_type1, device_type2;

	device_type1 = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE1);
	if (device_type1 < 0) {
		dev_err(&client->dev, "%s: err %d ", __func__, device_type1);
		return (ssize_t)device_type1;
	}
	device_type2 = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE2);
	if (device_type2 < 0) {
		dev_err(&client->dev, "%s: err %d ", __func__, device_type2);
		return (ssize_t)device_type2;
	}

	if (device_type1 & DEV_T1_USB_MASK || device_type2 & DEV_T2_USB_MASK)
		return snprintf(buf, 22, "USB_STATE_CONFIGURED\n");

	return snprintf(buf, 25, "USB_STATE_NOTCONFIGURED\n");
}

static ssize_t rt8973_show_adc(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	int adc;

	adc = i2c_smbus_read_byte_data(client, REG_ADC);
	if (adc < 0) {
		dev_err(&client->dev,
			"%s: err at read adc %d\n", __func__, adc);
		return snprintf(buf, 9, "UNKNOWN\n");
	}

	return snprintf(buf, 4, "%x\n", adc);
}

static ssize_t rt8973_reset(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct rt8973_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	if (!strncmp(buf, "1", 1)) {
		dev_info(&client->dev,
			"rt8973 reset after delay 1000 msec.\n");
		msleep(1000);
		rt8973_write_reg(client, REG_RESET, 0x01);
		dev_info(&client->dev, "rt8973_reset_control done!\n");
	} else {
		dev_info(&client->dev,
			"rt8973_reset_control, but not reset_value!\n");
	}

	rt8973_reg_init(usbsw);

	return count;
}


static DEVICE_ATTR(control, S_IRUGO, rt8973_show_control, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, rt8973_show_device_type, NULL);
static DEVICE_ATTR(switch, S_IRUGO | S_IWUSR,rt8973_show_manualsw, rt8973_set_manualsw);
static DEVICE_ATTR(usb_state, S_IRUGO, rt8973_show_usb_state, NULL);
static DEVICE_ATTR(adc, S_IRUGO, rt8973_show_adc, NULL);
static DEVICE_ATTR(reset_switch, S_IWUSR | S_IWGRP, NULL, rt8973_reset);


static struct attribute *rt8973_attributes[] = {
	&dev_attr_control.attr,
	&dev_attr_device_type.attr,
	&dev_attr_switch.attr,
	NULL
};

static const struct attribute_group rt8973_group = {
	.attrs = rt8973_attributes,
};


#if defined(CONFIG_VIDEO_MHL_V2)
int dock_det(void)
{
	return local_usbsw->dock_attached;
}
EXPORT_SYMBOL(dock_det);
#endif

int rt_check_jig_state(void)
{
	return jig_state;
}
EXPORT_SYMBOL(rt_check_jig_state);





static int rt8973_attach_dev(struct rt8973_usbsw *usbsw)
{
	int adc;
	int val1, val2;
	struct rt8973_platform_data *pdata = usbsw->pdata;
	struct i2c_client *client = usbsw->client;
#if defined(CONFIG_VIDEO_MHL_V2)
	/*u8 mhl_ret = 0;*/
#endif
	val1 = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE1);
	if (val1 < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, val1);
		return val1;
	}

	val2 = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE2);
	if (val2 < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, val2);
		return val2;
	}
	jig_state =  (val2 & DEV_T2_JIG_ALL_MASK) ? 1 : 0;

	adc = i2c_smbus_read_byte_data(client, REG_ADC);

#if 0 //KBJ temp block ( this source is based on tsu6721, but following value doesn't match fsa880 )
	if (adc == ADC_SMART_DOCK) {

		val2 = DEV_SMARTDOCK;

		val1 = 0;
	}
#endif

#if 0 // block the usb_host notify in RT MUIC temp.
#if defined(CONFIG_USB_HOST_NOTIFY)
	if (adc == 0x11 || adc == ADC_AUDIO_DOCK) {
		val2 = DEV_AUDIO_DOCK;
		val1 = 0;
	}
#endif
#endif
	dev_err(&client->dev,
			"dev1: 0x%x, dev2: 0x%x, ADC: 0x%x Jig:%s\n",
			val1, val2, adc,
			(rt_check_jig_state() ? "ON" : "OFF"));

	/* USB */
	if (val1 & DEV_USB || val2 & DEV_T2_USB_MASK) {
		pr_info("[MUIC] USB Connected\n");
		pdata->callback(CABLE_TYPE_USB, FSA880_ATTACHED);
	/* USB_CDP */
	} else if (val1 & DEV_USB_CHG) {
		pr_info("[MUIC] CDP Connected\n");
		pdata->callback(CABLE_TYPE_CDP, FSA880_ATTACHED);
	/* UART */
	} else if (val2 & DEV_T2_UART_MASK) {
		rt_uart_connecting = 1;
		pr_info("[MUIC] UART Connected\n");
		pdata->callback(CABLE_TYPE_UARTOFF, FSA880_ATTACHED);
	/* CHARGER */
	} else if ((val1 & DEV_T1_CHARGER_MASK)) {
		pr_info("[MUIC] Charger Connected\n");
		pdata->callback(CABLE_TYPE_AC, FSA880_ATTACHED);
	/* JIG */
	} else if (val2 & DEV_T2_JIG_MASK) {
		pr_info("[MUIC] JIG Connected\n");
		pdata->callback(CABLE_TYPE_JIG, FSA880_ATTACHED);
#if 0 //KBJ temp block ( this source is based on tsu6721, but following value doesn't match fsa880 )
	/* Desk Dock */
	} else if ((val2 & DEV_AV)) {
		pr_info("[MUIC] Deskdock Connected\n");
		local_usbsw->dock_attached = FSA880_ATTACHED;
		rt8973_dock_control(usbsw, CABLE_TYPE_DESK_DOCK,
			FSA880_ATTACHED, SW_AUDIO);
	/* Car Dock */
	} else if (val2 & DEV_JIG_UART_ON) {
		pr_info("[MUIC] Cardock Connected\n");
		local_usbsw->dock_attached = FSA880_ATTACHED;
		rt8973_dock_control(usbsw, CABLE_TYPE_CARDOCK,
			FSA880_ATTACHED, SW_AUDIO);

	/* SmartDock */
	}else if (val2 & DEV_SMARTDOCK) {
		pr_info("[MUIC] Smartdock Connected\n");
		rt8973_dock_control(usbsw, CABLE_TYPE_SMART_DOCK,
			FSA880_ATTACHED, SW_USB;
#if defined(CONFIG_VIDEO_MHL_V2)
		/*mhl_onoff_ex(1); support can be added once mhl is up*/
#endif
#endif

#if 0 // block the usb_host notify in RT MUIC temp.
#if defined(CONFIG_USB_HOST_NOTIFY)
	/* Audio Dock */
	} else if (val2 & DEV_AUDIO_DOCK) {
		pr_info("[MUIC] Audiodock Connected\n");
		rt8973_dock_control(usbsw, CABLE_TYPE_AUDIO_DOCK,
			FSA880_ATTACHED, SW_USB);
#endif
#endif
	/* Incompatible */
	}
	usbsw->dev1 = val1;
	usbsw->dev2 = val2;	
	usbsw->adc = adc;

	return adc;
}



static int rt8973_detach_dev(struct rt8973_usbsw *usbsw)
{
	struct rt8973_platform_data *pdata = usbsw->pdata;

	/* USB */
	if (usbsw->dev1 & DEV_USB ||
			usbsw->dev2 & DEV_T2_USB_MASK) {
		pr_info("[MUIC] USB Disonnected\n");
		pdata->callback(CABLE_TYPE_USB, FSA880_DETACHED);
	} else if (usbsw->dev1 & DEV_USB_CHG) {
		pdata->callback(CABLE_TYPE_CDP, FSA880_DETACHED);

	/* UART */
	} else if (usbsw->dev2 & DEV_T2_UART_MASK) {
		pr_info("[MUIC] USB Disonnected\n");
		pdata->callback(CABLE_TYPE_UARTOFF, FSA880_DETACHED);
		rt_uart_connecting = 0;
	/* CHARGER */
	} else if ((usbsw->dev1 & DEV_T1_CHARGER_MASK) ) {
		pr_info("[MUIC] Charger Disonnected\n");
		pdata->callback(CABLE_TYPE_AC, FSA880_DETACHED);
	/* JIG */
	} else if (usbsw->dev2 & DEV_T2_JIG_MASK) {
		pr_info("[MUIC] JIG Disonnected\n");
		pdata->callback(CABLE_TYPE_JIG, FSA880_DETACHED);
#if 0 //KBJ temp block ( this source is based on tsu6721, but following value doesn't match fsa880 )
	/* Desk Dock */
	} else if ((usbsw->dev2 & DEV_AV) ) {
		pr_info("[MUIC] Deskdock Disonnected\n");
		local_usbsw->dock_attached = FSA880_DETACHED;
		rt8973_dock_control(usbsw, CABLE_TYPE_DESK_DOCK,
			FSA880_DETACHED, SW_ALL_OPEN);
#endif
	/* Car Dock */
	} else if (usbsw->dev2 & DEV_JIG_UART_ON) {
		pr_info("[MUIC] Cardock Disonnected\n");
		local_usbsw->dock_attached = FSA880_DETACHED;
		rt8973_dock_control(usbsw, CABLE_TYPE_CARDOCK,
			FSA880_DETACHED, SW_ALL_OPEN);
#if 0 //KBJ temp block ( this source is based on tsu6721, but following value doesn't match fsa880 )
	/* Smart Dock */
	} else if (usbsw->dev2 == DEV_SMARTDOCK) {
		pr_info("[MUIC] Smartdock Disonnected\n");
		rt8973_dock_control(usbsw, CABLE_TYPE_SMART_DOCK,
			FSA880_DETACHED, SW_ALL_OPEN);
#if defined(CONFIG_VIDEO_MHL_V2)
		//mhl_onoff_ex(false);
#endif
#endif
#if 0 // block the usb_host notify in RT MUIC temp.
#if defined(CONFIG_USB_HOST_NOTIFY)
	/* Audio Dock */
	} else if (usbsw->dev2 == DEV_AUDIO_DOCK) {
		pr_info("[MUIC] Audiodock Disonnected\n");
		rt8973_dock_control(usbsw, CABLE_TYPE_AUDIO_DOCK,
			FSA880_DETACHED, SW_ALL_OPEN);
#endif
#endif
	/* Incompatible */
	} 
	usbsw->dev1 = 0;
	usbsw->dev2 = 0;	
	usbsw->adc = 0;

	return 0;

}
static irqreturn_t rt8973_irq_thread(int irq, void *data)
{
	struct rt8973_usbsw *usbsw = data;
	struct i2c_client *client = usbsw->client;
	int intr1;
	int val1, adc;
	/* rt8973 : Read interrupt -> Read Device */
	pr_info("rt8973_irq_thread is called\n");

	rt8973_disable_interrupt();
	intr1 = i2c_smbus_read_byte_data(client, REG_INT1);
	
	dev_info(&client->dev, "%s: intr : 0x%x \n",
		__func__, intr1 );

	/* device detection */
	mutex_lock(&usbsw->mutex);

	/* interrupt both attach and detach */
	if (intr1 == (INT_ATTACH + INT_DETACH)) {
		val1 = i2c_smbus_read_byte_data(client, REG_DEVICE_TYPE1);		
		adc = i2c_smbus_read_byte_data(client, REG_ADC);
		if ((adc == ADC_OPEN) && (val1 == DATA_NONE))
			rt8973_detach_dev(usbsw);
		else
			rt8973_attach_dev(usbsw);
	/* interrupt attach */
	} else if (intr1 & INT_ATTACH)
		rt8973_attach_dev(usbsw);
	/* interrupt detach */
	else if (intr1 & INT_DETACH)
		rt8973_detach_dev(usbsw);
	mutex_unlock(&usbsw->mutex);

	rt8973_enable_interrupt();

	return IRQ_HANDLED;
}

static int rt8973__irq_init(struct rt8973_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	int ret;

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
			rt8973_irq_thread, IRQF_TRIGGER_FALLING,
			"rt8973 micro USB", usbsw);
		if (ret) {
			dev_err(&client->dev, "failed to reqeust IRQ\n");
			return ret;
		}
		enable_irq_wake(client->irq);
	}

	return 0;
}

static void rt8973_init_detect(struct work_struct *work)
{
	struct rt8973_usbsw *usbsw = container_of(work,
			struct rt8973_usbsw, init_work.work);
	int ret;
	int int_reg1;

	dev_info(&usbsw->client->dev, "%s\n", __func__);

	mutex_lock(&usbsw->mutex);
	rt8973_attach_dev(usbsw);
	mutex_unlock(&usbsw->mutex);

	ret = rt8973__irq_init(usbsw);
	if (ret)
		dev_info(&usbsw->client->dev,
				"failed to enable  irq init %s\n", __func__);

	int_reg1 = rt8973_read_reg(usbsw->client, REG_INT1);
	dev_info(&usbsw->client->dev, "%s: intr1 : 0x%x\n",
		__func__, int_reg1);

}



#ifdef CONFIG_OF
static int rt8973_parse_dt(struct device *dev, struct rt8973_platform_data *pdata)
{

        struct device_node *np = dev->of_node;
	/*changes can be added later, when needed*/
	#if 0
        /* regulator info */
	pdata->i2c_pull_up = of_property_read_bool(np, "rt8973,i2c-pull-up");

        /* reset, irq gpio info */
        pdata->gpio_scl = of_get_named_gpio_flags(np, "rt8973,scl-gpio",
                               0, &pdata->scl_gpio_flags);
        pdata->gpio_sda = of_get_named_gpio_flags(np, "rt8973,sda-gpio",
                               0, &pdata->sda_gpio_flags);
	#endif
        pdata->gpio_int = of_get_named_gpio_flags(np, "rt8973,irq-gpio",
                0, &pdata->irq_gpio_flags);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->gpio_int);

        return 0;
}
#endif

static int __devinit rt8973_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rt8973_usbsw *usbsw;
	int ret = 0;
	struct device *switch_dev;
	struct rt8973_platform_data *pdata;
#if 0 //KBJ temp block ( this source is based on tsu6721, but following value doesn't match fsa880 )
	static struct regulator* ldo14;

	dev_info(&client->dev,"%s:rt8973 probe called \n",__func__);

	ldo14 = regulator_get(NULL,"vdd_sexykyu");
	regulator_set_voltage(ldo14,1800000,1800000);
	regulator_enable(ldo14);
#endif

	dev_info(&client->dev,"%s:rt8973 probe called \n",__func__);
	if(client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct rt8973_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
				return -ENOMEM;
		}
//		pdata = &rt8973_pdata;
		ret = rt8973_parse_dt(&client->dev, pdata);
		if (ret < 0)
			return ret;

		pdata->callback = rt8973_callback;
		pdata->dock_init = rt8973_dock_init;
#if 0 //KBJ temp block ( this source is based on fsa9280, but following value doesn't match fsa880 )
		pdata->oxp_callback = rt8973_oxp_callback;
#endif

		pdata->mhl_sel = NULL;
		gpio_tlmm_config(GPIO_CFG(pdata->gpio_int,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
		client->irq = gpio_to_irq(pdata->gpio_int);
	} else
		pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	usbsw = kzalloc(sizeof(struct rt8973_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		kfree(usbsw);
		return -ENOMEM;
	}

	usbsw->client = client;
	if (client->dev.of_node)
		usbsw->pdata = pdata;
	else
		usbsw->pdata = client->dev.platform_data;
	if (!usbsw->pdata)
		goto fail1;

	i2c_set_clientdata(client, usbsw);

	mutex_init(&usbsw->mutex);

	local_usbsw = usbsw;

	rt8973_reg_init(usbsw);

	ret = sysfs_create_group(&client->dev.kobj, &rt8973_group);
	if (ret) {
		dev_err(&client->dev,
				"failed to create rt8973 attribute group\n");
		goto fail2;
	}

	/* make sysfs node /sys/class/sec/switch/usb_state */
	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	if (IS_ERR(switch_dev)) {
		pr_err("[rt8973] Failed to create device (switch_dev)!\n");
		ret = PTR_ERR(switch_dev);
		goto fail2;
	}

		ret = device_create_file(switch_dev, &dev_attr_usb_state);
	if (ret < 0) {
		pr_err("[rt8973] Failed to create file (usb_state)!\n");
		goto err_create_file_state;
	}

	ret = device_create_file(switch_dev, &dev_attr_adc);
	if (ret < 0) {
		pr_err("[rt8973] Failed to create file (adc)!\n");
		goto err_create_file_adc;
	}

	ret = device_create_file(switch_dev, &dev_attr_reset_switch);
	if (ret < 0) {
		pr_err("[rt8973] Failed to create file (reset_switch)!\n");
		goto err_create_file_reset_switch;
	}

	dev_set_drvdata(switch_dev, usbsw);
	/* rt8973 dock init*/
	if (usbsw->pdata->dock_init)
		usbsw->pdata->dock_init();

	/* initial cable detection */
	INIT_DELAYED_WORK(&usbsw->init_work, rt8973_init_detect);
	/* temporary change delay value, qpnp_sec_charger driver is loaded too late, so cable detection making problem. */
	//schedule_delayed_work(&usbsw->init_work, msecs_to_jiffies(2700));
	schedule_delayed_work(&usbsw->init_work, msecs_to_jiffies(3700));

	return 0;

err_create_file_reset_switch:
	device_remove_file(switch_dev, &dev_attr_reset_switch);
err_create_file_adc:
	device_remove_file(switch_dev, &dev_attr_adc);
err_create_file_state:
	device_remove_file(switch_dev, &dev_attr_usb_state);
fail2:
	if (client->irq)
		free_irq(client->irq, usbsw);

fail1:
	mutex_destroy(&usbsw->mutex);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;

}


static int __devexit rt8973_remove(struct i2c_client *client)
{
	struct rt8973_usbsw *usbsw = i2c_get_clientdata(client);
	cancel_delayed_work(&usbsw->init_work);
	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, usbsw);
	}
	mutex_destroy(&usbsw->mutex);
	i2c_set_clientdata(client, NULL);
	sysfs_remove_group(&client->dev.kobj, &rt8973_group);
	kfree(usbsw);
	return 0;
}

static int rt8973_resume(struct i2c_client *client)
{
	struct rt8973_usbsw *usbsw = i2c_get_clientdata(client);

	pr_info("%s: resume \n",__func__);
	i2c_smbus_read_byte_data(client, REG_INT1);	

	/* device detection */
	mutex_lock(&usbsw->mutex);

	rt8973_attach_dev(usbsw);

	mutex_unlock(&usbsw->mutex);
	return 0;

}


static const struct i2c_device_id rt8973_id[] = {
	{"rt8973", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rt8973_id);
static struct of_device_id rt8973_i2c_match_table[] = {
	{ .compatible = "rt8973,i2c",},
	{},
};
MODULE_DEVICE_TABLE(of, rt8973_i2c_match_table);

static struct i2c_driver rt8973_i2c_driver = {
	.driver = {
		.name = "rt8973",
		.owner = THIS_MODULE,
		.of_match_table = rt8973_i2c_match_table,
	},

	.probe = rt8973_probe,
	.remove = __devexit_p(rt8973_remove),
	.resume = rt8973_resume,
	.id_table = rt8973_id,
};

static int __init rt8973_init(void)
{
	return i2c_add_driver(&rt8973_i2c_driver);
}
module_init(rt8973_init);

static void __exit rt8973_exit(void)
{
	i2c_del_driver(&rt8973_i2c_driver);
}
module_exit(rt8973_exit);

MODULE_AUTHOR("Deukkyu Oh <deukkyu.oh@samsung.com>");
MODULE_DESCRIPTION("rt8973 Micro USB Switch driver");
MODULE_LICENSE("GPL");

