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

#define AWB_PROC_NAME "awb_block_info"

typedef struct awb_proc_item
{
    double    r_gain;
    double    g_gain;
    double    b_gain;
    int       color_temp;
} AWB_PROC_ITEM;

static AWB_PROC_ITEM awb_item_exported;

typedef struct awb_block_item
{
    int color_temp;	
    int r_gain; // Multiply by base_multiplier;
    int g_gain; // multiply by base_multiplier;
    int b_gain; // multiply by base_multiplier;
    int base_multiplier;
} AWB_BLOCK_ITEM;

static AWB_BLOCK_ITEM awb_block;

/*
 * /proc/awb
 */
static void *awb_seq_start(struct seq_file *f, loff_t *pos)
{
    if (*pos == 0)	
        return &awb_item_exported;
    else
        return NULL;
}

static void *awb_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
    (*pos)++;
    if (*pos > 0)
        return NULL;
    return pos;
}

static void awb_seq_stop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static int awb_seq_show_awb(struct seq_file *f, void *v)
{
    char msg[256];
    sprintf(msg, 
		   "r_gain=%d, g_gain=%d, b_gain=%d, color_temp=%d, base_multiplier=%d\n",
           awb_block.r_gain,
           awb_block.g_gain,
           awb_block.b_gain,
           awb_block.color_temp,
		   awb_block.base_multiplier);

    seq_printf(f, "%s", msg);
    return 0;
}

static const struct seq_operations int_seq_ops = {
	.start = awb_seq_start,
	.next  = awb_seq_next,
	.stop  = awb_seq_stop,
	.show  = awb_seq_show_awb
};

static int awb_open(struct inode *inode, struct file *filp)
{
    return seq_open(filp, &int_seq_ops);
}



static ssize_t awb_write(struct file* filp, const char __user* buf, size_t len, loff_t* pos)
{
    memset(&awb_block, 0, sizeof(awb_block));

    if (len > sizeof(awb_block))
        len = sizeof(awb_block);

    if (copy_from_user((char*) &awb_block, buf, len))
    { 
        printk(KERN_ERR"tam:::awb_write:copy_from_user failed.\n");
        return -1;
    } 
#if 0
    printk(KERN_ERR"tam::awb_write:ct=%d, len=%d, AWB_BLOCK_ITEM size=%d, r_gain=%d, g_gain=%d, b_gain=%d, base_mutiplier=%d\n",
		   awb_block.color_temp,     
           len,
           sizeof(awb_block),
		   awb_block.r_gain,
		   awb_block.g_gain,
		   awb_block.b_gain,
		   awb_block.base_multiplier);
#endif
    return len;
}

static const struct file_operations proc_awb_operations = {
	.open		= awb_open,
	.read		= seq_read,
	.write		= awb_write,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_awb_init(void)
{
    proc_create(AWB_PROC_NAME, 0666, NULL, &proc_awb_operations);

    memset(&awb_item_exported, 0, sizeof(awb_item_exported));

    return 0;
}

static void __exit proc_awb_exit(void)
{
    remove_proc_entry(AWB_PROC_NAME, NULL);
}

module_init(proc_awb_init);
module_exit(proc_awb_exit);

