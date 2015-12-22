/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

#define REG_DEVID 0x00
#define REG_FWVERSION 0x01
#define REG_KEYSTATUS 0x02
#define REG_CMD 0x03
#define REG_THRSHOLD 0x04
#define REG_AUTORST_TIME 0x05
#define REG_AUTORST_STATUS 0x06
#define REG_RESERVE3 0x07
#define REG_DBG_DIFF 0x08
#define REG_DBG_RAWDATA 0x0a
#define REG_DBG_BASELINE 0x0c

#define CMD_THRSHOLD 0x01
#define CMD_SLEEP 0x02
#define CMD_AUTO_RST_TIME 0x04

struct cap_switch_data {
	struct switch_dev sdev;
	int irq_gpio;
	u32 irq_gpio_flags;
    struct i2c_client *client;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	struct work_struct wakeup_work;
};

struct cap_switch_data *switch_data;

static int cy8c_i2c_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf;
    int ret = 0;
    
    buf = addr;
    ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
		printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
        return ret;
    }
    ret = i2c_master_recv(client, (char*)&buf, 1);
    if (ret < 0) {
        printk(KERN_ERR "%s():reads data error!!\n", __FUNCTION__);
        return ret;
    }
	else {
    }
    *data = buf;
    return 0;
}

static int cy8c_i2c_write_byte(struct i2c_client *client, u8 addr, u8 data)
{
    u8 buf[] = {addr, data};
    int ret = 0;

    ret = i2c_master_send(client, (const char*)buf, sizeof(buf));
    if (ret < 0) {
        printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
        return ret;
    }
	else {
    }
    return 0;
}

static int cy8c_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;

    if (len == 1)
	{
        err = cy8c_i2c_read_byte(client, addr, data);
    }
	else
	{
		if(len > 16)
		{
			return -EINVAL;
		}
		data[0] = addr;
		err = i2c_master_send(client, data, 1);
		if (err < 0) {
			printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
			return err;
		}
		err = i2c_master_recv(client, data, len);
		if (err != len)
		{
			printk(KERN_ERR "%s():reads data error!!\n", __FUNCTION__);
			return err;
		}
		else
		{
			err = 0;
		}
    }

	return err;
}

static int cy8c_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[16];

    if (!client)
        return -EINVAL;
    else if (len >= 16) {        
        printk(KERN_ERR "%s(): length %d exceeds %d\n", __FUNCTION__, len, 16);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
        buf[num++] = data[idx];

    err = i2c_master_send(client, buf, num);
    if (err < 0) {
        printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
        return -EFAULT;
    } else {
        err = 0;    /*no error*/
    }
    return err;
}


static void cap_wakeup_work(struct work_struct *work)
{
	struct cap_switch_data	*data =
		container_of(work, struct cap_switch_data, wakeup_work);
	int gpio = data->irq_gpio;

	printk(KERN_INFO "%s(), %d\n", __FUNCTION__, gpio);

	gpio_direction_output(gpio,1);
	msleep(5);
	gpio_direction_output(gpio,0);
	msleep(5);
	gpio_direction_input(gpio);

}

/*----------------------------------------------------------------------------*/
static int CY8C_ReadData(struct i2c_client *client, char *buf)
{
	u8 addr=0;
	int err = 0;
	int i;
	int data;

	err = cy8c_i2c_read_block(client, addr, buf, 14);

	if(err)
	{
		printk(KERN_ERR "%s():cap_switch_ReadData error: %d\n", __FUNCTION__, err);
	}
	else
	{
		for(i=0; i<14; i++)
		{
			data = buf[i];
			printk(KERN_INFO "%s():cap_switch_ReadData from(%d) [%d]\n", __FUNCTION__, i, data);
		}
	}

	return err;
}

static void cap_switch_work(struct work_struct *work)
{
	int state;
	char buf[16];
	struct cap_switch_data	*data =
		container_of(work, struct cap_switch_data, work);

	CY8C_ReadData(data->client, buf);
	state = buf[2];
	switch_set_state(&data->sdev, state);
	printk(KERN_INFO "%s():state=%d\n", __FUNCTION__, state);
}

static irqreturn_t cap_irq_handler(int irq, void *dev_id)
{
	struct cap_switch_data *switch_data =
	    (struct cap_switch_data *)dev_id;

	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

#if 0
static ssize_t switch_cap_print_state(struct switch_dev *sdev, char *buf)
{
	struct cap_switch_data	*switch_data =
		container_of(sdev, struct cap_switch_data, sdev);
	const char *state;

	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);

	return -1;
}
#endif

static ssize_t cap_switch_deviceid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_DEVID, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(deviceid, S_IRUGO, cap_switch_deviceid_show,
	NULL);

static ssize_t cap_switch_fwversion_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_FWVERSION, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(fwversion, S_IRUGO, cap_switch_fwversion_show,
	NULL);

static ssize_t cap_switch_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t cap_switch_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = switch_data->client;
	int ret;
	u32 cmd = 0, val = 0;
	u8 temp[2];

	if (sscanf(buf, "%x,%x", &cmd, &val) == 2) {
		printk(KERN_INFO "%s():cmd=0x%02x, val=0x%02x\n", __FUNCTION__, *(u8*)&cmd, *(u8*)&val);
		temp[0] = *(u8*)&cmd;
		temp[1] = *(u8*)&val;
		if(temp[0] == CMD_SLEEP)
			ret = cy8c_i2c_write_block(client, REG_CMD, temp, 1);
		else
			ret = cy8c_i2c_write_block(client, REG_CMD, temp, 2);

		if(ret)
			printk(KERN_ERR "%s():failed,ret = %d\n", __FUNCTION__, ret);

	}

	return count;
}

static DEVICE_ATTR(cmd, S_IRUGO | S_IWUGO, cap_switch_cmd_show,
	cap_switch_cmd_store);

static ssize_t cap_switch_keystatus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_KEYSTATUS, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(keystatus, S_IRUGO | S_IWUSR, cap_switch_keystatus_show,
	NULL);

static ssize_t cap_switch_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_THRSHOLD, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(threshold, S_IRUGO, cap_switch_threshold_show,
	NULL);

static ssize_t cap_switch_autoresettime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_AUTORST_TIME, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(autoresettime, S_IRUGO, cap_switch_autoresettime_show,
	NULL);

static ssize_t cap_switch_autoresetstatus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 value = 0xff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_byte(client, REG_AUTORST_STATUS, &value);
	if(!ret) {
		ret = sprintf(buf, "0x%02x\n", value);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(autoresetstatus, S_IRUGO, cap_switch_autoresetstatus_show,
	NULL);

static ssize_t cap_switch_debugparam_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 diff = 0xffff, rawdata = 0xffff, baseline = 0xffff;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = cy8c_i2c_read_block(client, REG_DBG_DIFF, (u8*)&diff, 2);
	ret |= cy8c_i2c_read_block(client, REG_DBG_RAWDATA, (u8*)&rawdata, 2);
	ret |= cy8c_i2c_read_block(client, REG_DBG_BASELINE, (u8*)&baseline, 2);
	if(!ret) {
		ret = sprintf(buf, "diff = 0x%04x\nrawdata = 0x%04x\nbaseline = 0x%04x\ns",
			diff, rawdata, baseline);
		return ret;
	}

	return 0;
}

static DEVICE_ATTR(debugparam, S_IRUGO, cap_switch_debugparam_show,
	NULL);

static ssize_t cap_switch_pm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = switch_data->client;
	struct cap_switch_data *data = (struct cap_switch_data *)i2c_get_clientdata(client);
	int ret;

	if(buf[0] == 's') {
		ret = cy8c_i2c_write_byte(client, REG_CMD, CMD_SLEEP);
		if(ret)
			printk(KERN_ERR "%s():failed,ret = %d\n", __FUNCTION__, ret);
	}else if(buf[0] == 'r') {
		schedule_work(&data->wakeup_work);
	}

	return count;
}

static DEVICE_ATTR(pm, S_IWUGO, NULL, cap_switch_pm_store);

static struct attribute *cap_switch_attr[] = {
	&dev_attr_deviceid.attr,
	&dev_attr_fwversion.attr,
	&dev_attr_cmd.attr,
	&dev_attr_keystatus.attr,
	&dev_attr_threshold.attr,
	&dev_attr_autoresettime.attr,
	&dev_attr_autoresetstatus.attr,
	&dev_attr_debugparam.attr,
	&dev_attr_pm.attr,
	NULL
};

static struct attribute_group cap_switch_attr_group = {
	.name = NULL,
	.attrs = cap_switch_attr,
};


static int cap_switch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	printk(KERN_INFO "%s(\n", __FUNCTION__);

	switch_data = kzalloc(sizeof(struct cap_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "capsensor";
	switch_data->client = client;
	switch_data->irq_gpio = 46;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->irq_gpio, switch_data->sdev.name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->irq_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, cap_switch_work);
	INIT_WORK(&switch_data->wakeup_work, cap_wakeup_work);

	switch_data->irq = gpio_to_irq(switch_data->irq_gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, cap_irq_handler,
			  IRQF_TRIGGER_MASK, switch_data->sdev.name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Perform initial detection */
	cap_switch_work(&switch_data->work);
	ret = i2c_smbus_read_byte_data(client, 0x00);
        printk(KERN_INFO "cap_switch reg 00 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x01);
        printk(KERN_INFO "cap_switch reg 01 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x09);
        printk(KERN_INFO "cap_switch reg 09 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x0D);
        printk(KERN_INFO "cap_switch reg 0D %d\n", ret);
	//ret = i2c_smbus_read_byte_data(client, 0x0E);
        //printk("cap_switch reg 0E %d\n", ret);
	//ret = i2c_smbus_read_byte_data(client, 0x1F);
        //printk("cap_switch reg 0F %d\n", ret);

	i2c_set_clientdata(client, (void*)switch_data);

	ret = sysfs_create_group(&client->dev.kobj, &cap_switch_attr_group);
	if (ret) {
		printk(KERN_ERR "%s():failed to create sysfs device attributes\n", __FUNCTION__);
		goto err_create_group;
	}

	return 0;

err_create_group:
	sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->irq_gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit cap_switch_remove(struct i2c_client *client)
{
	struct cap_switch_data *data = (struct cap_switch_data *)i2c_get_clientdata(client);

	cancel_work_sync(&data->work);
	gpio_free(data->irq_gpio);
    switch_dev_unregister(&data->sdev);
	kfree(data);
	sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);

	return 0;
}

#ifdef CONFIG_PM
static int cap_switch_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret;

	printk(KERN_INFO "%s()\n", __FUNCTION__);
	ret = cy8c_i2c_write_byte(client, REG_CMD, CMD_SLEEP);
	if(ret)
		printk(KERN_ERR "%s():failed,ret = %d\n", __FUNCTION__, ret);

	return 0;
}

static int cap_switch_resume(struct i2c_client *client)
{
	struct cap_switch_data *data = (struct cap_switch_data *)i2c_get_clientdata(client);

	printk(KERN_INFO "%s()\n", __FUNCTION__);
	schedule_work(&data->wakeup_work);

	return 0;
}
#else
#define cap_switch_suspend NULL
#define cap_switch_resume NULL
#endif

static const struct i2c_device_id capsensor_id[] = {
	{ "capsensor", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, capsensor_id);

#ifdef CONFIG_OF
static struct of_device_id capsensor_match_table[] = {
	{ .compatible = "cypress,capsensor",},
	{ },
};
#else
#define capsensor_match_table NULL
#endif

static struct i2c_driver cap_switch_driver = {
	.driver		= {
		.name	= "capsensor",
		.of_match_table = capsensor_match_table,
	},
	.probe		= cap_switch_probe,
	.remove		= __devexit_p(cap_switch_remove),
	.suspend	= cap_switch_suspend,
	.resume		= cap_switch_resume,
	.id_table   = capsensor_id,
};

static int __init cap_switch_init(void)
{
	return i2c_add_driver(&cap_switch_driver);
}

static void __exit cap_switch_exit(void)
{
	i2c_del_driver(&cap_switch_driver);
}

module_init(cap_switch_init);
module_exit(cap_switch_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("CAP Sensor driver");
MODULE_LICENSE("GPL");
