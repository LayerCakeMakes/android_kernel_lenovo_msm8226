/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/msm-charger.h>
#include <linux/i2c/bq27520.h> /* use the same platform data as bq27520 */

#define DRIVER_VERSION			"1.1.0"
/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)

#define BQ27541_RECHG_PERIOD	 (msecs_to_jiffies(1000*100))//((HZ) * 150) -->  100s
#define RECHARGE_VOLTAGE 4250000  // 4.25V 
extern int bq24196_is_charger_present(void);
extern void notify_bq24196_send_recharging_event(void);
int  recharging_fake_soc_flag = 0;

/* If the system has several batteries we need a different name for each
 * of them...
 */

extern int charge_bat_status;
extern int bq24196_is_charging;


static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);
/*lenovo-sw: xuwei9 BLADEFHD-226 begin*/
DEFINE_MUTEX(bq27541_i2c_access);
/*lenovo-sw: xuwei9 BLADEFHD-226 end*/
/*lenovo-sw BLADEFHD-427 begin: Avoid i2c transfer before driver resume*/
/*default is wakeup after power on*/
static int wake_flag = 1;
/*lenovo-sw BLADEFHD-427 end: Avoid i2c transfer before driver resume*/
struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};

struct bq27541_device_info {
	struct device			*dev;
	int				id;
	struct bq27541_access_methods	*bus;
	struct i2c_client		*client;
	struct work_struct		counter;
	/* 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work		hw_config;
	struct delayed_work recharge_work;
};

static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */

static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di);

static int bq27541_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
/*lenovo-sw: xuwei9 BLADEFHD-226 begin*/
         int ret = 0;
         mutex_lock(&bq27541_i2c_access);
         ret = di->bus->read(reg, rt_value, b_single, di);
         mutex_unlock(&bq27541_i2c_access);
         return ret;
	//return di->bus->read(reg, rt_value, b_single, di);
/*lenovo-sw: xuwei9 BLADEFHD-226 end*/
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	static int prv_temp = 1;

	if(wake_flag == 1)
	{
		ret = bq27541_read(BQ27541_REG_TEMP, &temp, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading temperature\n");
			return ret;
		}
		prv_temp = temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
		return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}
	else
    		return prv_temp;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_capacity(struct bq27541_device_info *di,int real_flag)
{
	int ret;
	int capacity = 0;
	static int prv_capacity = 1;
	static int old_capacity = 0;//avoid capacity 100 without charge jump 99 when plug charge in.

	if(wake_flag == 1)
	{
		ret = bq27541_read(BQ27541_REG_SOC, &capacity, 0, di);

		printk(KERN_ERR "%s,read from ic: %d",__func__,capacity);
	if(bq24196_is_charging == 1)//charging
	{
		if(capacity == 100) //27541 show full , still need futher check
		{
			if(charge_bat_status == 0x3)//24196 check full, double check with 27541
			{
				capacity = 100;
				old_capacity = 100;
			}
			else//24196 check not full,
			{
				if(old_capacity == 100) // if old capacity 100, keep it for user.
					capacity = 100;
				else//keep the capacity 100 to 99
				{
					capacity = 99;
					old_capacity = capacity;
				}
			}
		}
	}
	else//not charging
	{
		if(old_capacity == 99 && capacity == 100)
			capacity = 99 ;
		else
			old_capacity = capacity; // record the 100 capacity when without charge to avoid  100 jump to 99 when charger in.
	}
	
	
	printk(KERN_ERR "%s,%d",__func__,capacity);

		if (ret) {
			dev_err(di->dev, "error reading temperature\n");
			return ret;
		}
		prv_capacity = capacity;
		return capacity ;
	}
	else
    		return prv_capacity;
}


/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;
	static int prv_volt = 3500000;

	if(wake_flag == 1)
	{
		ret = bq27541_read(BQ27541_REG_VOLT, &volt, 0, di);
		printk(KERN_ERR "%s,%d",__func__,volt*1000);
		if (ret) {
			dev_err(di->dev, "error reading voltage\n");
			return ret;
		}
		prv_volt = volt*1000;
		return volt * 1000;
	}
	else
    		return prv_volt;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}

/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static int bq27541_chip_config(struct bq27541_device_info *di)
{
	int flags = 0, ret = 0;

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading register %02x ret = %d\n",
			 BQ27541_REG_CNTL, ret);
		return ret;
	}
	udelay(66);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27541_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0;
	struct bq27541_device_info *di;
	unsigned long flags;
	int count = 0;

	di = container_of(work, struct bq27541_device_info, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27541_read(BQ27541_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		/* delay 66uS, waiting time between continuous reading
		 * results
		 */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;
		udelay(66);
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error reading datalog register\n");
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

struct bq27541_device_info *bq27541_di;

int bq27541_get_battery_mvolts(void)
{
	return bq27541_battery_voltage(bq27541_di);
}
EXPORT_SYMBOL(bq27541_get_battery_mvolts);

int bq27541_get_battery_temperature(void)
{
	return bq27541_battery_temperature(bq27541_di);
}
EXPORT_SYMBOL(bq27541_get_battery_temperature);

//get the real capacity
int bq27541_get_real_battery_capacity(void)
{
	return bq27541_battery_capacity(bq27541_di,1);
}

int bq27541_get_battery_capacity(void)
{
	return bq27541_battery_capacity(bq27541_di,0);
}
EXPORT_SYMBOL(bq27541_get_battery_capacity);


int bq27541_is_battery_present(void)
{
	return 1;
}
EXPORT_SYMBOL(bq27541_is_battery_present);
static int bq27541_is_battery_temp_within_range(void)
{
	return 1;
}
static int bq27541_is_battery_id_valid(void)
{
	return 1;
}

void bq27541_recharge_work(struct work_struct *bq27541_work)
{
	int current_voltage = bq27541_get_battery_mvolts();

	struct bq27541_device_info *di = bq27541_di;
	printk(KERN_ERR "%s \n",__func__);
	if(di == NULL)
	{
		recharging_fake_soc_flag = 0;
		printk(KERN_ERR "%s null pointer back",__func__);
		return ; 
	}

	if(!bq24196_is_charger_present())
	{	
		recharging_fake_soc_flag = 0;
		return;
	}

	if(current_voltage > RECHARGE_VOLTAGE )
	{
		recharging_fake_soc_flag = 1;
		schedule_delayed_work(&bq27541_di->recharge_work, BQ27541_RECHG_PERIOD);
	}
	else{
		recharging_fake_soc_flag = 1;
		notify_bq24196_send_recharging_event();
		cancel_delayed_work(&bq27541_di->recharge_work);
		printk(KERN_ERR "%s recharging event send out",__func__);
	}
	return;
}

int bq27541_monitor_recharging(void)
{
	printk(KERN_ERR "%s\n ",__func__);
	schedule_delayed_work(&bq27541_di->recharge_work, BQ27541_RECHG_PERIOD);
	return 0;
}
int bq27541_stop_monitor_recharging(void)
{
	printk(KERN_ERR "%s \n",__func__);
	recharging_fake_soc_flag = 0;
	cancel_delayed_work_sync(&bq27541_di->recharge_work);
	return 0;
}

static struct msm_battery_gauge bq27541_batt_gauge = {
	.get_battery_mvolts		= bq27541_get_battery_mvolts,
	.get_battery_temperature	= bq27541_get_battery_temperature,
	.is_battery_present		= bq27541_is_battery_present,
	.is_battery_temp_within_range	= bq27541_is_battery_temp_within_range,
	.is_battery_id_valid		= bq27541_is_battery_id_valid,
	.get_batt_remaining_capacity    = bq27541_get_battery_capacity,
	.monitor_for_recharging         = bq27541_monitor_recharging ,
};
static void bq27541_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;
	struct bq27541_device_info *di;

	di  = container_of(work, struct bq27541_device_info, hw_config.work);
	ret = bq27541_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27541\n");
		return;
	}
	msm_battery_gauge_register(&bq27541_batt_gauge);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &type, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &fw_ver, 0, di);

	dev_info(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			type, fw_ver);
	dev_info(di->dev, "Complete bq27541 configuration 0x%02X\n", flags);
}

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

#ifdef CONFIG_BQ27541_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27541_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27541_REG_ICR && reg > 0x00) {
		ret = bq27541_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27541_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27541_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27541_SUBCMD_FW_VER ||
		 subcmd == BQ27541_SUBCMD_HW_VER ||
		 subcmd == BQ27541_SUBCMD_CHEM_ID) {

		bq27541_cntl_cmd(di, subcmd); /* Retrieve Chip status */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_CNTL, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27541_read_stdcmd,
	bq27541_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27541_read_subcmd,
	bq27541_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name			= "bq27541-test",
	.id			= -1,
	.dev.platform_data	= NULL,
};
#endif

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *di;
	struct bq27541_access_methods *bus;
	int num;
	int retval = 0;

    dev_err(&client->dev, "bq27541_battery_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;

#ifdef CONFIG_BQ27541_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_4;
	} else
		goto batt_failed_4;
#endif

	if (retval) {
		dev_err(&client->dev, "failed to setup bq27541\n");
		goto batt_failed_4;
	}

	if (retval) {
		dev_err(&client->dev, "failed to powerup bq27541\n");
		goto batt_failed_4;
	}

	spin_lock_init(&lock);

	bq27541_di = di;
	INIT_WORK(&di->counter, bq27541_coulomb_counter_work);
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	INIT_DELAYED_WORK(&di->recharge_work, bq27541_recharge_work);
	schedule_delayed_work(&di->hw_config, BQ27541_INIT_DELAY);
	dev_err(&client->dev, "success to probe bq27541\n");
	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}
/*lenovo-sw BLADEFHD-427 begin: Avoid i2c transfer before driver resume*/
static int bq27541_i2c_suspend(struct i2c_client *client,pm_message_t state)
{
	wake_flag = 0;
	printk(KERN_ERR "bq27541 enter suspend");
	return 0;
}

extern struct power_supply msm_psy_batt;
static int bq27541_i2c_resume(struct i2c_client *client)
{
	wake_flag = 1;
	power_supply_changed(&msm_psy_batt);//sync the battery status as soon as bq27541 i2c resume finish.
        printk(KERN_ERR "bq27541 enter resume");
        return 0;
}

/*lenovo-sw BLADEFHD-427 end: Avoid i2c transfer before driver resume*/
static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	msm_battery_gauge_unregister(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);
	cancel_delayed_work_sync(&di->recharge_work);
	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);

static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
			.name = "bq27541-battery",
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
/*lenovo-sw BLADEFHD-427 begin: Avoid i2c transfer before driver resume*/
	.resume         = bq27541_i2c_resume,
	.suspend        = bq27541_i2c_suspend,
/*lenovo-sw BLADEFHD-427 end: Avoid i2c transfer before driver resume*/
	.id_table	= bq27541_id,
};
static int __init bq27541_battery_init(void)
{
	int ret;

	printk(KERN_ERR " BQ27541 driver\n");
	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
