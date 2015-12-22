#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <asm/io.h>

#include "leds-lp5560.h"

#define GPIO_CTRL_PIN   GPIO74
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

#define DEV_NAME    "notifylight"


/* ----------------------------------------------------------------------------*/
#define LED_DEBUG

#if defined(LED_DEBUG)
#define LED_TAG                  "[LED5560] "
#define LED_FUN(f)               printk(KERN_INFO LED_TAG"%s\n", __FUNCTION__)
#define LED_ERR(fmt, args...)    printk(KERN_ERR  LED_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define LED_LOG(fmt, args...)    printk(KERN_ERR LED_TAG fmt, ##args)
#define LED_DBG(fmt, args...)    printk(KERN_INFO LED_TAG fmt, ##args) 
#else
#define LED_FUN(f)
#define LED_ERR(fmt, args...)    printk(KERN_ERR  LED_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define LED_LOG(fmt, args...)
#define LED_DBG(fmt, args...)
#endif

#define CUST_TIME   32
u32 time_onms[CUST_TIME] = {13, 26, 52, 105, 158, 211, 264, 316, 369, 435, 501, 594, 699, 805, 910, 1016, 1122, 1227, 1353, 1478,
                            1603, 1729, 1854, 1980, 2105, 2230, 2356, 2481, 2613, 2745, 2877, 3009};
u32 time_offms[CUST_TIME] = {26, 52, 105, 211, 316, 422, 528, 633, 739, 871, 1001, 1188, 1399, 1610, 1821, 2032, 2244, 2455, 2706, 2956,
                            3207, 3458, 3709, 3960, 4210, 4461, 4712, 4963, 5227, 5491, 5755, 6019};

DEFINE_SPINLOCK(lp_lock);

static long led_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
static unsigned long onMS;
static unsigned long offMS;
static int ctrl_gpio;


void ctrl_gpio_output(int level)
{
	if (level)
		gpio_set_value_cansleep(ctrl_gpio,GPIO_OUT_ONE);
	else
		gpio_set_value_cansleep(ctrl_gpio,GPIO_OUT_ZERO);
}

void ctrl_output_pulse(int t_high, int t_low)
{
	ctrl_gpio_output(GPIO_OUT_ONE);
	udelay(t_high);
	ctrl_gpio_output(GPIO_OUT_ZERO);
	udelay(t_low);
}

void enter_command(int cmd)
{
	switch(cmd)
	{
		case CMD_ONCE:
			ctrl_output_pulse(TC_ON, TC_OFF);
		case CMD_TEND:
			ctrl_output_pulse(TC_ON, TC_OFF);
		case CMD_TSTART:
			ctrl_output_pulse(TC_ON, TC_OFF);
			ctrl_output_pulse(TC_ON, TC_OFF);
            udelay(TENTER - cmd * (TC_ON + TC_OFF));
            udelay(TBLANK);
			break;
		case CMD_RUN:
			ctrl_gpio_output(GPIO_OUT_ONE);
			break;
		default:
			break;
	}
}

void calibration_pulse(void)
{
	udelay(50);
	ctrl_output_pulse(TCAL, 50);
}

void set_current_pulse(u32 mA)
{
	ctrl_output_pulse(mA + TADJ, mA + TADJ);
}

void reset_default(void)
{
	spin_lock(&lp_lock);
	enter_command(CMD_TSTART);
	enter_command(CMD_TEND);
	spin_unlock(&lp_lock);
}

int set_train_pulse(sequence *seq)
{
	spin_lock(&lp_lock);
	enter_command(CMD_TSTART);
	calibration_pulse();
	set_current_pulse(seq->cur);

	ctrl_output_pulse(seq->rise1, seq->on1);
	ctrl_output_pulse(seq->fall1, seq->off1);
	ctrl_output_pulse(seq->rise2, seq->on2);
	ctrl_output_pulse(seq->fall2, seq->off2);
	ctrl_output_pulse(seq->rise3, seq->on3);
	ctrl_output_pulse(seq->fall3, seq->off3);
	
	enter_command(CMD_TEND);
	spin_unlock(&lp_lock);

	return 0;
}

void set_current(u32 cur)
{
	spin_lock(&lp_lock);
	enter_command(CMD_TSTART);
	calibration_pulse();
	set_current_pulse(cur);
	enter_command(CMD_TEND);
	spin_unlock(&lp_lock);
}

void set_run_mode(void)
{
	ctrl_gpio_output(GPIO_OUT_ONE);
	//enter_command(CMD_RUN);
}

void set_runonce_mode(void)
{
	enter_command(CMD_ONCE);
	ctrl_gpio_output(GPIO_OUT_ZERO);
}

void set_follow_mode(void)
{
	spin_lock(&lp_lock);
	enter_command(CMD_TSTART);
	calibration_pulse();
	enter_command(CMD_TEND);
	spin_unlock(&lp_lock);
}

void set_standby_mode(void)
{
	ctrl_gpio_output(GPIO_OUT_ZERO);
	mdelay(10);
}

void lp5560_normally_on(u32 cur)
{
	set_standby_mode();
	set_follow_mode();
	set_standby_mode();

	set_current(cur);
	set_standby_mode();
	set_run_mode();
}

void lp5560_flash(struct seq_config *config)
{
    sequence seq;
    
    if (!config->use_def) {
        seq.rise1 = seq.rise2 = seq.rise3 = TADJ + config->rise;
        seq.fall1 = seq.fall2 = seq.fall3 = TADJ + config->fall;
        seq.on1 = seq.on2 = seq.on3 = TADJ + config->on;
        seq.off1 = seq.off2 = seq.off3 = TADJ + config->off;
        seq.cur = config->cur;
    }

    LED_LOG("seq.rise1:%d seq.fall1:%d seq.on1:%d seq.off1:%d seq.cur:%d\n",
            seq.rise1, seq.fall1, seq.on1, seq.off1, seq.cur);

	set_standby_mode();
    reset_default();
	set_standby_mode();

    if (!config->use_def) {
        set_train_pulse(&seq);
    }

	set_run_mode();
}

void lp5560_flash_once(void)
{
	set_standby_mode();
	//set_train_pulse(seq);
	set_runonce_mode();
}

void lp5560_turnoff(void)
{
	set_standby_mode();
	reset_default();
	set_standby_mode();
}


static ssize_t show_delay_on(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", onMS);
}

static ssize_t store_delay_on(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    LED_LOG("onMS : %s\n", buf);
    if (strict_strtoul(buf, 0, &onMS))
        return -EINVAL;

    return len;
}

static ssize_t show_delay_off(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", offMS);
}

static ssize_t store_delay_off(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    LED_LOG("offMS : %s\n", buf);
    if (strict_strtoul(buf, 0, &offMS))
        return -EINVAL;
    
    return len;
}

static LP5560_ATTR_RW(delay_on);
static LP5560_ATTR_RW(delay_off);

static struct attribute *lp5560_led_attributes[] = {
    &dev_attr_delay_on.attr,
    &dev_attr_delay_off.attr,
    NULL,
};

static struct attribute_group lp5560_led_attribute_group = {
    .attrs      = lp5560_led_attributes
};


static int led_open(struct inode *i, struct file *f)
{
    return 0;
}

static int led_close(struct inode *i, struct file *f)
{
    return 0;
}

static long led_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct seq_config seq;

    switch (cmd) {
        case LED_TON:
            lp5560_normally_on((u32)arg);
            break;
        case LED_TOFF:
            lp5560_turnoff();
            break;
        case LED_FLASH:
            if (copy_from_user(&seq, (void __user *)arg, sizeof(seq)))
                ret = -EFAULT;
            LED_LOG("rise = %d on = %d off = %d fall = %d cur = %d def = %d\n",
                    seq.rise, seq.on, seq.off, seq.fall, seq.cur, seq.use_def);
            lp5560_flash(&seq);
            break;
        case LED_FLASH_TEST:
            memcpy(&seq, (struct seq_config *)arg, sizeof(seq));
            LED_LOG("sysfs:rise = %d on = %d off = %d fall = %d cur = %d def = %d\n",
                    seq.rise, seq.on, seq.off, seq.fall, seq.cur, seq.use_def);
            lp5560_flash(&seq);
            break;
        case LED_ONCE:
            lp5560_flash_once();
            break;
        case LED_CUR:
            set_current(arg);
            break;
    }

    return ret;
}

static const struct file_operations led_fops = { 
    .open    = &led_open,
    .release = &led_close,
    .unlocked_ioctl = &led_ioctl,
    .owner   = THIS_MODULE,
};

static struct miscdevice led_misc_dev = { 
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "led",
    .fops       = &led_fops,
};

static void lp5560_led_white_set(struct led_classdev *led_cdev,
                   enum led_brightness value)
{
    int idx, pluse_onms, pluse_offms;
    struct seq_config seq = {
                .rise = RAMP_528,
                .fall = RAMP_528,
                .cur = CUR_028,
                .use_def = 0
            };
	

    LED_LOG("led_brightness: %d\n", value);

    switch (value) {
        case 0:
            led_ioctl(NULL, LED_TOFF, 0);
            break;
        case 255:
            led_ioctl(NULL, LED_TON, CUR_195);
            break;
        default:
            if ((onMS == 0) && (offMS == 0))
            {
                LED_LOG("turn off\n");
                led_ioctl(NULL, LED_TOFF, 0);
                break;
            } 
            else if ((onMS != 0) && (offMS == 0))
            {
                LED_LOG("offMS = 0\n");
                led_ioctl(NULL, LED_TON, (value / 32) * 1000);
                break;
            }

            for (idx = 0; idx < CUST_TIME; idx++)
            {
                if (onMS < time_onms[idx])
                { 
                    pluse_onms = idx * 1000;
                    break;
                }
            }

            for (idx = 0; idx < CUST_TIME; idx++)
            {
                if (offMS < time_offms[idx])
                {
                    pluse_offms = idx * 1000;
                    break;
                }
            }
    
	    seq.on = pluse_onms;
	    seq.off = pluse_offms;
            seq.cur = (value / 32) * 1000,

            LED_LOG("led_brightness: onMS = %ld offMS = %ld pluse_onms = %d pluse_offms = %d\n",
                    onMS, offMS, pluse_onms, pluse_offms);
            

            led_ioctl(NULL, LED_FLASH_TEST, (unsigned long)&seq);
            break;
    } 
}

static struct led_classdev lp5560_white_led = { 
    .name           = "white",
    .default_trigger    = "ide-disk",
    .brightness_set     = lp5560_led_white_set,
};

static const struct of_device_id of_leds_match[] = {
        { .compatible = "leds-lp5560", },
        {},
};

static int lp5560_led_probe(struct platform_device *pdev)
{
    int ret;
    enum of_gpio_flags flags;
    struct device_node *np = pdev->dev.of_node;

    ret = led_classdev_register(&pdev->dev, &lp5560_white_led);
    if (ret < 0)
        led_classdev_unregister(&lp5560_white_led);

    ctrl_gpio = of_get_gpio_flags(np, 0, &flags);
    gpio_request(ctrl_gpio, "white-led");
    gpio_direction_output(ctrl_gpio, GPIO_OUT_ZERO);

    ret = sysfs_create_group(&lp5560_white_led.dev->kobj,
                        &lp5560_led_attribute_group);
    if (ret) {
        led_classdev_unregister(&lp5560_white_led);
        LED_ERR("create ledflash file error\n");
        return -1;
    }

    ret = misc_register(&led_misc_dev);
    if (ret)
        return ret;

    lp5560_turnoff();

    return ret;
}

static int lp5560_led_remove(struct platform_device *pdev)
{
    lp5560_turnoff();
    misc_deregister(&led_misc_dev);
    sysfs_remove_group(&lp5560_white_led.dev->kobj, &lp5560_led_attribute_group);
    led_classdev_unregister(&lp5560_white_led);

    return 0;
}

static struct platform_driver led_driver= {
        .probe      = lp5560_led_probe,
        .remove     = lp5560_led_remove,
        .driver     = {
        .name       = "led-sensor",
	.owner  = THIS_MODULE,
	.of_match_table = of_leds_match,
        },
};

static s32 __init led_init(void)
{
    s32 ret;

    ret = platform_driver_register(&led_driver);
    if (ret != 0){
        LED_ERR("Unable to register led sensor device (%d)\n", ret);
            return ret;
    }

    LED_LOG("led_mod_init Done \n");

    return 0;
}

static void __exit led_exit(void)
{
    platform_driver_unregister(&led_driver);
    LED_LOG("led_mod_exit Done \n");
}

module_init(led_init);
module_exit(led_exit);
MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("Lenovo LED Sensor Driver");
MODULE_LICENSE("GPL");

