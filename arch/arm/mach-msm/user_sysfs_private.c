/*
* Copyright (C) 2012 lenovo, Inc.
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

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/uaccess.h> /* sys_sync */
#include <linux/rtc.h> /* sys_sync */
/* yangjq, 2011-12-16, Add for vreg, START */
#include <linux/platform_device.h>
#include <mach/vreg.h>
/* yangjq, 2011-12-16, Add for vreg, END */
#include <linux/err.h>
#include <asm/gpio.h>
#include <mach/mpp.h>
////#include <mach/pmic.h>
#include <linux/regulator/consumer.h>
#include <mach/restart.h>

//yangjq, Add for acpuclk_get_rate()
#include "acpuclock.h"

#define private_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define TLMM_NUM_GPIO 146

extern int tlmm_dump_info(char* buf, int tlmm_num);
static int tlmm_num = -1;
static ssize_t tlmm_num_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += sprintf(p, "A single gpio[0, 145] to be checked by cat tlmm\n");
	p += sprintf(p, "-1 to check all 146 gpios by cat tlmm\n");
	p += sprintf(p, "%d\n", tlmm_num);
	return p - buf;
}

static ssize_t tlmm_num_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int gpio;
	int res;
	
	res = sscanf(buf, "%d", &gpio);
	printk("res=%d. %d\n", res, gpio);
	
	if(res != 1)
		goto tlmm_num_store_wrong_para;

	if(gpio >= TLMM_NUM_GPIO)
		goto tlmm_num_store_wrong_para;

	tlmm_num = gpio;
	printk("tlmm_num: %d\n", tlmm_num);

	goto tlmm_num_store_ok;
		
tlmm_num_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Format: gpio_num\n");
	printk("      gpio_num: 0 ~ 145\n");

tlmm_num_store_ok:	
	return n;
}

static ssize_t tlmm_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += tlmm_dump_info(buf, tlmm_num);
	return p - buf;
}

#ifdef CONFIG_MSM_SECURE_MODE
static ssize_t secure_boot_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	int efuse_state;
	efuse_state = get_efuse_state();
	
	p += sprintf(p, "%d\n",efuse_state);
	return p - buf;
}

static ssize_t secure_boot_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int num;
	int is_efuse;
	int efuse_state;
	
	num = sscanf(buf, "%d", &is_efuse);
	printk("is_efuse=%d\n", is_efuse);
	
	if(is_efuse != 1)
	{
	    printk("wrong input value,echo 1 for secure boot efuse blow\n");
            return n;
	}

	efuse_state = get_efuse_state();
        if(  efuse_state== BOOT_EFUSE_NOBLOW )
        {
            set_efuse_mode();
            printk("set_efuse_mode = true\n");
        }
	else
            printk("can not blow efuse,the efuse state is %d\n",efuse_state);

	return n;
}
#endif

static ssize_t tlmm_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	char pull_c, dir_c;
	int gpio, func, pull, dir, drvstr, output_val;
	unsigned cfg;
	int res;
	
	res = sscanf(buf, "%d %d %c %c %d %d", &gpio, &func, &pull_c, &dir_c, &drvstr, &output_val);
	printk("res=%d.  %d %d %c %c %d %d\n", res, gpio, func, pull_c, dir_c, drvstr, output_val);
	
	if((res != 5) && (res != 6)) 
		goto tlmm_store_wrong_para;

	if(gpio >= TLMM_NUM_GPIO)
		goto tlmm_store_wrong_para;
		
	if('N' == pull_c)
		pull = 0;
	else if('D' == pull_c)
		pull = 1;
	else if('K' == pull_c)
		pull = 2;
	else if('U' == pull_c)
		pull = 3;
	else 
		goto tlmm_store_wrong_para;

	if('I' == dir_c)
		dir = 0;
	else if('O' == dir_c)
		dir = 1;
	else 
		goto tlmm_store_wrong_para;
	
	drvstr = drvstr/2 - 1; // 2mA -> 0, 4mA -> 1, 6mA -> 2, ...	
	if(drvstr > 7)
		goto tlmm_store_wrong_para;
	
	if(output_val > 1)
		goto tlmm_store_wrong_para;
		
	printk("final set: %d %d %d %d %d %d\n", gpio, func, pull, dir, drvstr, output_val);

	cfg = GPIO_CFG(gpio, func, dir, pull, drvstr);	
	res = gpio_tlmm_config(cfg, GPIO_CFG_ENABLE);
	if(res < 0) {
		printk("Error: Config failed.\n");
		goto tlmm_store_wrong_para;
	}
	
	if((func == 0)  && (dir == 1)) // gpio output
		gpio_set_value(gpio, output_val);
	
	goto tlmm_store_ok;
		
tlmm_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Format: gpio_num  function  pull  direction  strength [output_value]\n");
	printk("      gpio_num: 0 ~ 145\n");
	printk("      function: number, where 0 is GPIO\n");	
	printk("      pull: 'N': NO_PULL, 'D':PULL_DOWN, 'K':KEEPER, 'U': PULL_UP\n");	
	printk("      direction: 'I': Input, 'O': Output\n");	
	printk("      strength:  2, 4, 6, 8, 10, 12, 14, 16\n");	
	printk("      output_value:  Optional. 0 or 1. vaild if GPIO output\n");	
	printk(" e.g.  'echo  20 0 D I 2'  ==> set pin 20 as GPIO input \n");	
	printk(" e.g.  'echo  20 0 D O 2 1'  ==> set pin 20 as GPIO output and the output = 1 \n");	

tlmm_store_ok:	
	return n;
}

/* Set GPIO's sleep config from sysfs */
extern int tlmm_before_sleep_table_dump_info(char* buf);
static ssize_t tlmm_before_sleep_table_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += tlmm_before_sleep_table_dump_info(buf);
	return p - buf;
}

extern int tlmm_before_sleep_table_set_cfg(unsigned gpio, unsigned cfg);
static ssize_t tlmm_before_sleep_table_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	char pull_c, dir_c;
	int gpio, func = 0, pull = 0, dir = 0, drvstr = 0, output_val = 0;
	int ignore;
	unsigned cfg;
	int res;
	
	res = sscanf(buf, "%d %d %c %c %d %d", &gpio, &func, &pull_c, &dir_c, &drvstr, &output_val);
	printk("res=%d.  %d %d %c %c %d %d\n", res, gpio, func, pull_c, dir_c, drvstr, output_val);
	
	if(1 == res) { // if only gpio, means ingore(disable) the gpio's sleep config 
		ignore = 1;
		printk("final set: to disable gpio %d sleep config\n", gpio);
	}
	else {
		ignore = 0;
	
		if((res != 5) && (res != 6)) 
			goto tlmm_before_sleep_table_store_wrong_para;

		if(gpio >= TLMM_NUM_GPIO)
			goto tlmm_before_sleep_table_store_wrong_para;
			
		if('N' == pull_c)
			pull = 0;
		else if('D' == pull_c)
			pull = 1;
		else if('K' == pull_c)
			pull = 2;
		else if('U' == pull_c)
			pull = 3;
		else 
			goto tlmm_before_sleep_table_store_wrong_para;

		if('I' == dir_c)
			dir = 0;
		else if('O' == dir_c)
			dir = 1;
		else 
			goto tlmm_before_sleep_table_store_wrong_para;
		
		drvstr = drvstr/2 - 1; // 2mA -> 0, 4mA -> 1, 6mA -> 2, ...	
		if(drvstr > 7)
			goto tlmm_before_sleep_table_store_wrong_para;
				
		printk("final set: %d %d %d %d %d\n", gpio, func, pull, dir, drvstr);
	}
		 
	cfg = GPIO_CFG(ignore ? 0xff : gpio, func, dir, pull, drvstr);
	res = tlmm_before_sleep_table_set_cfg(gpio, cfg | (output_val << 30));
	if(res < 0) {
		printk("Error: Config failed.\n");
		goto tlmm_before_sleep_table_store_wrong_para;
	}
	
	goto tlmm_before_sleep_table_store_ok;
		
tlmm_before_sleep_table_store_wrong_para:
	printk("Wrong Input.\n");	
	printk("Format: refer to tlmm's format except  'echo gpio_num > xxx' to disable the gpio's setting\n");	

tlmm_before_sleep_table_store_ok:
	return n;
}

extern int tlmm_before_sleep_dump_info(char* buf);
static ssize_t tlmm_before_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += tlmm_before_sleep_dump_info(buf);
	return p - buf;
}

static ssize_t tlmm_before_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}

extern int vreg_dump_info(char* buf);
static ssize_t vreg_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += vreg_dump_info(buf);
	return p - buf;
}

//extern void vreg_config(struct vreg *vreg, unsigned on, unsigned mv);
#if 0
extern void regulator_config(struct regulator *reg, unsigned on, unsigned mv);
#endif
static ssize_t vreg_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}

extern int vreg_before_sleep_dump_info(char* buf);
static ssize_t vreg_before_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char* p = buf;
	p += vreg_before_sleep_dump_info(buf);
	return p - buf;
}

static ssize_t vreg_before_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);
	return n;
}

static ssize_t clk_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	extern int clk_dump_info(char* buf);
	char *s = buf;

	// show all enabled clocks
	//s += sprintf(s, "\nEnabled Clocks:\n");
	s += clk_dump_info(s);
	
	return (s - buf);
}

static ssize_t clk_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support.\n", __func__);

	return -EPERM;
}

extern int wakelock_dump_info(char* buf);

static ssize_t pm_status_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	char *s = buf;
	unsigned long rate; // khz
	int cpu;

	// show CPU clocks
	for (cpu = 0; cpu < nr_cpu_ids; cpu++) {
		s += sprintf(s, "APPS[%d]:", cpu);
		if (cpu_online(cpu)) {
			rate = acpuclk_get_rate(cpu); // khz
			s += sprintf(s, "(%3lu MHz); \n", rate / 1000);
		} else {
			s += sprintf(s, "sleep; \n");
		}
	}

	s += wakelock_dump_info(s);
	
	return (s - buf);
}

static ssize_t pm_status_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}

static unsigned pm_wakeup_fetched = true;
static ssize_t pm_wakeup_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	char *s = buf;

	if (!pm_wakeup_fetched) {
		pm_wakeup_fetched = true;
		s += sprintf(s, "true");
	} else
		s += sprintf(s, "false");
	
	return (s - buf);
}

static ssize_t pm_wakeup_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}

private_attr(tlmm_num);
private_attr(tlmm);
private_attr(tlmm_before_sleep_table);
private_attr(tlmm_before_sleep);
private_attr(vreg_before_sleep);
private_attr(vreg);
private_attr(clk);
private_attr(pm_status);
private_attr(pm_wakeup);
#ifdef CONFIG_MSM_SECURE_MODE
private_attr(secure_boot);
#endif

static struct attribute *g_private_attr[] = {
	&tlmm_num_attr.attr,
	&tlmm_attr.attr,
	&tlmm_before_sleep_table_attr.attr,
	&tlmm_before_sleep_attr.attr,
	&vreg_attr.attr,
	&vreg_before_sleep_attr.attr,
	&clk_attr.attr,
	&pm_status_attr.attr,
	&pm_wakeup_attr.attr,
#ifdef CONFIG_MSM_SECURE_MODE
	&secure_boot_attr.attr,
#endif
	NULL,
};

static struct attribute_group private_attr_group = {
	.attrs = g_private_attr,
};

static struct kobject *sysfs_private_kobj;

#define SLEEP_LOG
#ifdef SLEEP_LOG
#define WRITE_SLEEP_LOG
#define MAX_WAKEUP_IRQ 8

enum {
	DEBUG_SLEEP_LOG = 1U << 0,
	DEBUG_WRITE_LOG = 1U << 1,
	DEBUG_WAKEUP_IRQ = 1U << 2,
};
static int debug_mask;// = DEBUG_WRITE_LOG;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct sleep_log_t {
	char time[18];
	long timesec;
	unsigned int log;
	uint32_t maoints[2];
	int wakeup_irq[MAX_WAKEUP_IRQ];
	int wakeup_gpip;
//31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//bit1-0=00 :try to sleep; bit 1-0 = 01 : leave from sleep    ;bit1-0=10:fail to sleep
//bit31-bit24 : return value
};

struct rpm_smem_state_t {
	uint32_t wakeup_ints[2];
};
struct rpm_smem_state_t rpm_smem_state_data;

#define TRY_TO_SLEEP  (0)
#define LEAVE_FORM_SLEEP  (1)
#define FAIL_TO_SLEEP  (2)

#define SLEEP_LOG_LENGTH 80

struct sleep_log_t sleep_log_array[SLEEP_LOG_LENGTH];
int sleep_log_pointer = 0;
int sleep_log_count = 0;
int enter_times = 0;

static int irq_wakeup_saved = MAX_WAKEUP_IRQ;
static int irq_wakeup_irq[MAX_WAKEUP_IRQ];
static int irq_wakeup_gpip;

char sleep_log_name[60];
struct file *sleep_log_file = NULL;

#ifdef WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	char buf[256];
	char *p, *p0;
	int i, j, pos;
	mm_segment_t old_fs;
	p = buf;
	p0 = p;

	if (sleep_log_file == NULL)
		sleep_log_file = filp_open(sleep_log_name, O_RDWR | O_APPEND | O_CREAT,
				0644);
	if (IS_ERR(sleep_log_file)) {
		printk("error occured while opening file %s, exiting...\n",
				sleep_log_name);
		return 0;
	}

	if (sleep_log_count > 1) {
		for (i = 0; i < 2; i++) {
			if (sleep_log_pointer == 0)
				pos = SLEEP_LOG_LENGTH - 2 + i;
			else
				pos = sleep_log_pointer - 2 + i;
			switch (sleep_log_array[pos].log & 0xF) {
			case TRY_TO_SLEEP:
				p += sprintf(p, ">[%ld]%s\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time);
				break;
			case LEAVE_FORM_SLEEP:
				p += sprintf(p, "<[%ld]%s(0x%x,0x%x,",
						sleep_log_array[pos].timesec,
						sleep_log_array[pos].time,
						sleep_log_array[pos].maoints[0],
						sleep_log_array[pos].maoints[1]);
				for (j = 0; j < MAX_WAKEUP_IRQ && sleep_log_array[pos].wakeup_irq[j]; j++)
					p += sprintf(p, " %d", sleep_log_array[pos].wakeup_irq[j]);

				if (sleep_log_array[pos].wakeup_gpip)
					p += sprintf(p, ", gpio %d", sleep_log_array[pos].wakeup_gpip);

				p += sprintf(p, ")\n");
				break;
			case FAIL_TO_SLEEP:
				p += sprintf(p, "^[%ld]%s(%d)\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time,
						(char) (sleep_log_array[pos].log >> 24));
				break;
			}
		}
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sleep_log_file->f_op->write(sleep_log_file, p0, p - p0,
			&sleep_log_file->f_pos);
	set_fs(old_fs);

	if (sleep_log_file != NULL) {
		filp_close(sleep_log_file, NULL);
		sleep_log_file = NULL;
	}
	return 0;
}
#else //WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	return 0;
}
#endif //WRITE_SLEEP_LOG

static int save_irq_wakeup_internal(int irq)
{
	int i;
	int ret;

	ret = 0;
	if (irq_wakeup_saved < MAX_WAKEUP_IRQ) {
		for (i = 0; i < irq_wakeup_saved; i++) {
			if (irq == irq_wakeup_irq[i])
				break;
		}
		if (i == irq_wakeup_saved)
			ret = irq_wakeup_irq[irq_wakeup_saved++] = irq;
	}
	return ret;
}

int save_irq_wakeup_gpio(int irq, int gpio)
{
	struct irq_desc *desc;
	int ret;

	ret = 0;
	if (debug_mask & DEBUG_WAKEUP_IRQ) {
		desc = irq_to_desc(irq);
		if (desc != NULL) {
			if (irqd_is_wakeup_set(&desc->irq_data)) {
				ret = save_irq_wakeup_internal(irq);
				if (ret) {
					if (gpio != 0 && irq_wakeup_gpip == 0) {
						irq_wakeup_gpip = gpio;
						irq_wakeup_saved = MAX_WAKEUP_IRQ;
					}
#ifdef CONFIG_KALLSYMS
					printk("%s(), irq=%d, %s, handler=(%pS)\n", __func__, irq,
						desc->action && desc->action->name ? desc->action->name : "",
						desc->action ? (void *)desc->action->handler : 0);
#else
					printk("%s(), irq=%d, %s, handler=0x%08x\n", __func__, irq,
						desc->action && desc->action->name ? desc->action->name : "",
						desc->action ? (unsigned int)desc->action->handler : 0);
#endif
				}
			}
		}
	}

	return ret;
}

static void clear_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ) {
		memset(irq_wakeup_irq, 0, sizeof(irq_wakeup_irq));
		irq_wakeup_gpip = 0;
		irq_wakeup_saved = 0;
	}
}

static void set_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ)
		irq_wakeup_saved = MAX_WAKEUP_IRQ;
}

void log_suspend_enter(void)
{
	extern void smem_set_reserved(int index, int data);
	struct timespec ts_;
	struct rtc_time tm_;

	//Turn on/off the share memory flag to inform RPM to record spm logs
	smem_set_reserved(6, debug_mask & DEBUG_WAKEUP_IRQ ? 1 : 0);

	if (debug_mask & DEBUG_SLEEP_LOG) {
		printk("%s(), APPS try to ENTER sleep mode>>>\n", __func__);

		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);

		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		if (strlen(sleep_log_name) < 1) {
			sprintf(sleep_log_name,
					"/data/local/log/aplog/sleeplog%d%02d%02d_%02d%02d%02d.txt",
					tm_.tm_year + 1900, tm_.tm_mon + 1, tm_.tm_mday, tm_.tm_hour,
					tm_.tm_min, tm_.tm_sec);
			printk("%s(), sleep_log_name = %s \n", __func__, sleep_log_name);
		}

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;
		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log = TRY_TO_SLEEP;
		sleep_log_pointer++;
		sleep_log_count++;
		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;
	}

	clear_irq_wakeup_saved();
	pm_wakeup_fetched = false;
}

void log_suspend_exit(int error)
{
	extern int smem_get_reserved(int index);
	struct timespec ts_;
	struct rtc_time tm_;
	int i;

	if (debug_mask & DEBUG_SLEEP_LOG) {
		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);
		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;

		if (error == 0) {
			rpm_smem_state_data.wakeup_ints[0] = smem_get_reserved(0);
			rpm_smem_state_data.wakeup_ints[1] = smem_get_reserved(1);

			printk("%s(), APPS Exit from sleep<<<: wakeup ints=0x%x, 0x%x\n", __func__ ,
					rpm_smem_state_data.wakeup_ints[0],
					rpm_smem_state_data.wakeup_ints[1]);

			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					LEAVE_FORM_SLEEP;
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].maoints[0] =
					rpm_smem_state_data.wakeup_ints[0];
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].maoints[1] =
					rpm_smem_state_data.wakeup_ints[1];
			for (i = 0; i < (irq_wakeup_gpip == 0 ? irq_wakeup_saved : 1); i++)
				sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_irq[i] =
					irq_wakeup_irq[i];
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_gpip =
					irq_wakeup_gpip;
		} else {
			printk("%s(), APPS FAIL to enter sleep^^^\n", __func__);

			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					FAIL_TO_SLEEP | (error << 24);
		}

		sleep_log_pointer++;
		sleep_log_count++;

		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;

		if (debug_mask & DEBUG_WRITE_LOG) {
			enter_times++;
			if (enter_times < 5000)
				sleep_log_write();
		}
	}

	set_irq_wakeup_saved();
}
#else //SLEEP_LOG
void log_suspend_enter(void)
{
	clear_irq_wakeup_saved();
	pm_wakeup_fetched = false;
}

void log_suspend_exit(int error)
{
	set_irq_wakeup_saved();
}
#endif //SLEEP_LOG

static int __init sysfs_private_init(void)
{
	int result;

	printk("%s(), %d\n", __func__, __LINE__);

	sysfs_private_kobj = kobject_create_and_add("private", NULL);
	if (!sysfs_private_kobj)
		return -ENOMEM;

	result = sysfs_create_group(sysfs_private_kobj, &private_attr_group);
	printk("%s(), %d, result=%d\n", __func__, __LINE__, result);

#ifdef SLEEP_LOG
	strcpy (sleep_log_name, "");
	sleep_log_pointer = 0;
	sleep_log_count = 0;
	enter_times = 0;
#endif

	return result;
}

static void __exit sysfs_private_exit(void)
{
	printk("%s(), %d\n", __func__, __LINE__);
	sysfs_remove_group(sysfs_private_kobj, &private_attr_group);

	kobject_put(sysfs_private_kobj);
}

module_init(sysfs_private_init);
module_exit(sysfs_private_exit);
