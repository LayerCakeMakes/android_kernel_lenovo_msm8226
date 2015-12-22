/**
  * This file is to create/delete awb proc file, for upper-layer module to get awb info.
  * Author: tam
  * Created Date: 2013-11-28.
  * Modifications: 
  */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>

#define AWB_PROC_NAME "awb_info"

typedef struct awb_proc_item
{
    int color_temp;
    int r_gain; // Mutiply by base_multiplier;
    int g_gain; // Mutiply by base_multiplier;
    int b_gain; // Mutiply by base_multiplier;
	int base_multiplier;
} AWB_PROC_ITEM;

static AWB_PROC_ITEM awb_item_exported;

static ssize_t awb_write(struct file* filp, const char __user* buf, size_t len, loff_t* pos)
{
    memset(&awb_item_exported, 0, sizeof(awb_item_exported));

    if (len > sizeof(awb_item_exported))
        len = sizeof(awb_item_exported);

    if (copy_from_user((char*) &awb_item_exported, buf, len))
    {
        printk(KERN_ERR"tam:::awb_write:copy_from_user failed.\n");
        return -1;
    }

#if 0
    printk(KERN_ERR"tam::awb_write:ct=%d, len=%d, AWB_BLOCK_ITEM size=%d, r_gain=%d, g_gain=%d, b_gain=%d, base_multiplier=%d\n",
           awb_item_exported.color_temp,     
           len,
           sizeof(awb_item_exported),
           awb_item_exported.r_gain,
           awb_item_exported.g_gain,
           awb_item_exported.b_gain,
           awb_item_exported.base_multiplier);
#endif

    return len;
}

ssize_t awb_read(struct file* filp, char __user* buf, size_t len, loff_t* pos)
{
    int data_size = sizeof(awb_item_exported);
    if (data_size <= len)
        len = data_size;

    return len - copy_to_user(buf, (char*) &awb_item_exported, len);
}

static const struct file_operations proc_awb_operations = {
	.read		= awb_read,
	.write		= awb_write,
};

static int __init proc_awb_init(void)
{
    proc_create(AWB_PROC_NAME, 0666, NULL, &proc_awb_operations);

    return 0;
}

static void __exit proc_awb_exit(void)
{
    remove_proc_entry(AWB_PROC_NAME, NULL);
}

module_init(proc_awb_init);
module_exit(proc_awb_exit);

