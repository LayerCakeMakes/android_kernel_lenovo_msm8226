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

struct mrsensor_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

static void mrsensor_switch_work(struct work_struct *work)
{
	int state;
	struct mrsensor_switch_data	*data =
		container_of(work, struct mrsensor_switch_data, work);

	state = gpio_get_value(data->gpio);
printk("mrsensor_switch_work() %d\n", state);
	switch_set_state(&data->sdev, state);
}

static irqreturn_t mrsensor_irq_handler(int irq, void *dev_id)
{
	struct mrsensor_switch_data *switch_data =
	    (struct mrsensor_switch_data *)dev_id;
printk("mrsensor_irq_handler() \n");

	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_mrsensor_print_state(struct switch_dev *sdev, char *buf)
{
	struct mrsensor_switch_data	*switch_data =
		container_of(sdev, struct mrsensor_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int mrsensor_switch_probe(struct platform_device *pdev)
{
	//struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct mrsensor_switch_data *switch_data;
	int ret = 0;
printk("mrsensor_switch_probe() \n");

	//if (!pdata)
	//	return -EBUSY;

	switch_data = kzalloc(sizeof(struct mrsensor_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "mrsensor";
	switch_data->gpio = 108;//pdata->gpio;
	//switch_data->name_on = pdata->name_on;
	//switch_data->name_off = pdata->name_off;
	//switch_data->state_on = pdata->state_on;
	//switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_mrsensor_print_state;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, switch_data->sdev.name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, mrsensor_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, mrsensor_irq_handler,
			  IRQF_TRIGGER_MASK, switch_data->sdev.name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Perform initial detection */
	mrsensor_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit mrsensor_switch_remove(struct platform_device *pdev)
{
	struct mrsensor_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mrsensor_match_table[] = {
	{ .compatible = "qcom,mrsensor",},
	{ },
};
#else
#define mrsensor_match_table NULL
#endif

MODULE_DEVICE_TABLE(of, mrsensor_match_table);

static struct platform_driver mrsensor_switch_driver = {
	.probe		= mrsensor_switch_probe,
	.remove		= __devexit_p(mrsensor_switch_remove),
	.driver		= {
		.name	= "mrsensor",
		.owner	= THIS_MODULE,
                .of_match_table = mrsensor_match_table,
	},
};

static int __init mrsensor_switch_init(void)
{
	return platform_driver_register(&mrsensor_switch_driver);
}

static void __exit mrsensor_switch_exit(void)
{
	platform_driver_unregister(&mrsensor_switch_driver);
}

module_init(mrsensor_switch_init);
module_exit(mrsensor_switch_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("CAP Sensor driver");
MODULE_LICENSE("GPL");
