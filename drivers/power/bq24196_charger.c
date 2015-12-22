/*
 *
 * TI bq24196 charger driver  xuwei9@lenovo.com
 *
 */
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c/bq24196.h>
#include <linux/power_supply.h>
#include <linux/msm-charger.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/reboot.h>

//---------config begin---------------------------------
#define HIGH_BATTERY_VOLTAGE_SUPPORT    //4.35 or normal 4.2V

#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
#define RECHARGING_VOLTAGE (4250*1000)
#else
#define RECHARGING_VOLTAGE (4110*1000)
#endif

#define MAX_CHARGING_TEMPERATURE  50     // c
#define AC_CHARGING_CURRENT     1908 //2000   //mA
#define USB_CHARGING_CURRENT      500    //mA
#define BQ24196_CHG_PERIOD	 (msecs_to_jiffies(1000*10))//((HZ) * 150) -->  10s
#define BQ24196_OTG_PERIOD       (msecs_to_jiffies(200))//((HZ) * 150) -->  2s
#define BQ24196_OTG_PWR_PERIOD       (msecs_to_jiffies(200))//((HZ) * 150) -->  2s

extern struct power_supply msm_psy_batt;

//suspend and resum
//#define CONFIG_PM
#define CHARGING_LOG 1

//---------config end  ---------------------------------
static DEFINE_MUTEX(bq24196_i2c_access);

extern  struct mutex bq27541_i2c_access;
extern int bq27541_get_real_battery_capacity(void);
extern int bq27541_stop_monitor_recharging(void);
#define ANDROID_BQ24196_PRINT_INFO (1U << 0)
#define ANDROID_BQ24196_PRINT_IO (1U << 1)

static int debug_mask = ANDROID_BQ24196_PRINT_INFO | ANDROID_BQ24196_PRINT_IO;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DEBUG  //print log
#ifdef DEBUG
#define pr_bq24196(debug_level_mask, args...) \
	do { \
		if (debug_mask & ANDROID_BQ24196_PRINT_##debug_level_mask) { \
			pr_info(args); \
		} \
	} while (0)
#endif

#define BQ24196_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

#define CHG_CURRENT_REG		0x00
#define FAST_CHG_CURRENT_MASK		BQ24196_MASK(3, 5)
#define PRE_CHG_CURRENT_MASK		BQ24196_MASK(2, 3)
#define TERM_CHG_CURRENT_MASK		BQ24196_MASK(2, 1)

#define INPUT_CURRENT_LIMIT_REG	0x01
#define IN_CURRENT_MASK			BQ24196_MASK(3, 5)
#define IN_CURRENT_LIMIT_EN_BIT		BIT(2)
#define IN_CURRENT_DET_THRESH_MASK	BQ24196_MASK(2, 0)

#define FLOAT_VOLTAGE_REG	0x02
#define STAT_OUT_POLARITY_BIT		BIT(7)
#define FLOAT_VOLTAGE_MASK		BQ24196_MASK(7, 0)

#define CONTROL_A_REG		0x03
#define AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CURR_CYCLE_TERM_BIT		BIT(6)
#define PRE_TO_FAST_V_MASK		BQ24196_MASK(3, 3)
#define TEMP_BEHAV_BIT			BIT(2)
#define THERM_NTC_CURR_MODE_BIT		BIT(1)
#define THERM_NTC_47KOHM_BIT		BIT(0)

#define CONTROL_B_REG		0x04
#define STAT_OUTPUT_MODE_MASK		BQ24196_MASK(2, 6)
#define BATT_OV_ENDS_CYCLE_BIT		BIT(5)
#define AUTO_PRE_TO_FAST_DIS_BIT	BIT(4)
#define SAFETY_TIMER_EN_BIT		BIT(3)
#define OTG_LBR_WD_EN_BIT		BIT(2)
#define CHG_WD_TIMER_EN_BIT		BIT(1)
#define IRQ_OP_MASK			BIT(0)

#define PIN_CTRL_REG		0x05
#define AUTO_CHG_EN_BIT			BIT(7)
#define AUTO_LBR_EN_BIT			BIT(6)
#define OTG_LBR_BIT			BIT(5)
#define I2C_PIN_BIT			BIT(4)
#define PIN_EN_CTRL_MASK		BQ24196_MASK(2, 2)
#define OTG_LBR_PIN_CTRL_MASK		BQ24196_MASK(2, 0)

#define OTG_LBR_CTRL_REG	0x06
#define BATT_MISSING_DET_EN_BIT		BIT(7)
#define AUTO_RECHARGE_THRESH_MASK	BIT(6)
#define USB_DP_DN_DET_EN_MASK		BIT(5)
#define OTG_LBR_BATT_CURRENT_LIMIT_MASK	BQ24196_MASK(2, 3)
#define OTG_LBR_UVLO_THRESH_MASK	BQ24196_MASK(3, 0)

#define FAULT_INTR_REG		0x07
#define SAFETY_TIMER_EXP_MASK		BQ24196_MASK(1, 7)
#define BATT_TEMP_UNSAFE_MASK		BQ24196_MASK(1, 6)
#define INPUT_OVLO_IVLO_MASK		BQ24196_MASK(1, 5)
#define BATT_OVLO_MASK			BQ24196_MASK(1, 4)
#define INTERNAL_OVER_TEMP_MASK		BQ24196_MASK(1, 2)
#define ENTER_TAPER_CHG_MASK		BQ24196_MASK(1, 1)
#define CHG_MASK			BQ24196_MASK(1, 0)

#define CELL_TEMP_MON_REG	0x08
#define THERMISTOR_CURR_MASK		BQ24196_MASK(2, 6)
#define LOW_TEMP_CHG_INHIBIT_MASK	BQ24196_MASK(3, 3)
#define HIGH_TEMP_CHG_INHIBIT_MASK	BQ24196_MASK(3, 0)

#define	SAFETY_TIMER_THERMAL_SHUTDOWN_REG	0x09
#define DCIN_OVLO_SEL_MASK		BQ24196_MASK(2, 7)
#define RELOAD_EN_INPUT_VOLTAGE_MASK	BQ24196_MASK(1, 6)
#define THERM_SHUTDN_EN_MASK		BQ24196_MASK(1, 5)
#define STANDBY_WD_TIMER_EN_MASK		BQ24196_MASK(1, 4)
#define COMPLETE_CHG_TMOUT_MASK		BQ24196_MASK(2, 2)
#define PRE_CHG_TMOUT_MASK		BQ24196_MASK(2, 0)

#define	VSYS_REG	0x0A
#define	VSYS_MASK			BQ24196_MASK(3, 4)

#define IRQ_RESET_REG	0x30

#define COMMAND_A_REG	0x31
#define	VOLATILE_REGS_WRITE_PERM_BIT	BIT(7)
#define	POR_BIT				BIT(6)
#define	FAST_CHG_SETTINGS_BIT		BIT(5)
#define	BATT_CHG_EN_BIT			BIT(4)
#define	USBIN_MODE_500_BIT		BIT(3)
#define	USBIN_MODE_HCMODE_BIT		BIT(2)
#define	OTG_LBR_EN_BIT			BIT(1)
#define	STAT_OE_BIT			BIT(0)

#define STATUS_A_REG	0x32
#define INTERNAL_TEMP_IRQ_STAT		BIT(4)
#define DCIN_OV_IRQ_STAT		BIT(3)
#define DCIN_UV_IRQ_STAT		BIT(2)
#define USBIN_OV_IRQ_STAT		BIT(1)
#define USBIN_UV_IRQ_STAT		BIT(0)

#define STATUS_B_REG	0x33
#define USB_PIN_STAT			BIT(7)
#define USB51_MODE_STAT			BIT(6)
#define USB51_HC_MODE_STAT		BIT(5)
#define INTERNAL_TEMP_LIMIT_B_STAT	BIT(4)
#define DC_IN_OV_STAT			BIT(3)
#define DC_IN_UV_STAT			BIT(2)
#define USB_IN_OV_STAT			BIT(1)
#define USB_IN_UV_STAT			BIT(0)

#define	STATUS_C_REG	0x34
#define AUTO_IN_CURR_LIMIT_MASK		BQ24196_MASK(4, 4)
#define AUTO_IN_CURR_LIMIT_STAT		BIT(3)
#define AUTO_SOURCE_DET_COMP_STAT_MASK	BQ24196_MASK(2, 1)
#define AUTO_SOURCE_DET_RESULT_STAT	BIT(0)

#define	STATUS_D_REG	0x35
#define	VBATT_LESS_THAN_VSYS_STAT	BIT(7)
#define	USB_FAIL_STAT			BIT(6)
#define	BATT_TEMP_STAT_MASK		BQ24196_MASK(2, 4)
#define	INTERNAL_TEMP_LIMIT_STAT	BIT(2)
#define	OTG_LBR_MODE_EN_STAT		BIT(1)
#define	OTG_LBR_VBATT_UVLO_STAT		BIT(0)

#define	STATUS_E_REG	0x36
#define	CHARGE_CYCLE_COUNT_STAT		BIT(7)
#define	CHARGER_TERM_STAT		BIT(6)
#define	SAFETY_TIMER_STAT_MASK		BQ24196_MASK(2, 4)
#define	CHARGER_ERROR_STAT		BIT(3)
#define	CHARGING_STAT_E			BQ24196_MASK(2, 1)
#define	CHARGING_EN			BIT(0)

#define	STATUS_F_REG	0x37
#define	WD_IRQ_ACTIVE_STAT		BIT(7)
#define	OTG_OVERCURRENT_STAT		BIT(6)
#define	BATT_PRESENT_STAT		BIT(4)
#define	BATT_OV_LATCHED_STAT		BIT(3)
#define	CHARGER_OVLO_STAT		BIT(2)
#define	CHARGER_UVLO_STAT		BIT(1)
#define	BATT_LOW_STAT			BIT(0)

#define	STATUS_G_REG	0x38
#define	CHARGE_TIMEOUT_IRQ_STAT		BIT(7)
#define	PRECHARGE_TIMEOUT_IRQ_STAT	BIT(6)
#define	BATT_HOT_IRQ_STAT		BIT(5)
#define	BATT_COLD_IRQ_STAT		BIT(4)
#define	BATT_OV_IRQ_STAT		BIT(3)
#define	TAPER_CHG_IRQ_STAT		BIT(2)
#define	FAST_CHG_IRQ_STAT		BIT(1)
#define	CHARGING_IRQ_STAT		BIT(0)

#define	STATUS_H_REG	0x39
#define	CHARGE_TIMEOUT_STAT		BIT(7)
#define	PRECHARGE_TIMEOUT_STAT		BIT(6)
#define	BATT_HOT_STAT			BIT(5)
#define	BATT_COLD_STAT			BIT(4)
#define	BATT_OV_STAT			BIT(3)
#define	TAPER_CHG_STAT			BIT(2)
#define	FAST_CHG_STAT			BIT(1)
#define	CHARGING_STAT_H			BIT(0)

#define DEV_ID_REG	0x3B

#define COMMAND_B_REG	0x3C
#define	THERM_NTC_CURR_VERRIDE		BIT(7)


#define INPUT_CURRENT_REG_DEFAULT	0xE1
#define INPUT_CURRENT_REG_MIN		0x01
#define	COMMAND_A_REG_DEFAULT		0xA0
#define	COMMAND_A_REG_OTG_MODE		0xA2

#define	PIN_CTRL_REG_DEFAULT		0x00
#define	PIN_CTRL_REG_CHG_OFF		0x04

#define	FAST_CHG_E_STATUS 0x2

#define BQ24196_DEFAULT_BATT_RATING   950
enum CHG_TYPE{
	CHARGER_NONE = 0,
	CHARGER_USB ,
	CHARGER_NORMAL,
};

struct bq24196_data {
	struct i2c_client *client;
	struct delayed_work charge_work;

	bool charging;
	int chgcurrent;
	int cur_charging_mode;
	int max_system_voltage;
	int min_system_voltage;

	int valid_n_gpio;
	int ce_gpio;
	int batt_status;
	int batt_chg_type;
	int batt_present;
	int min_design;
	int max_design;
//	int batt_mah_rating;

	int usb_status;
	unsigned char charging_temp_flag;
	u8 dev_id_reg;
	struct wake_lock charging_lock;
	enum CHG_TYPE charger_type;

	struct msm_hardware_charger adapter_hw_chg;
	int otg_irq;
	struct delayed_work otg_work;
	struct delayed_work otg_power_work;
	int    otg_power_status;
	bool suspend_flag;
	bool otg_interrupt_flag;
	struct mutex lock;
};

static int bq24196_manual_control = 0;
static int bq24196_manual_temperature = 0;
void bq24196_clear_manual_control_flag(void);

static unsigned int disabled;
static DEFINE_MUTEX(init_lock);
static unsigned int init_otg_power = 0;
int charge_bat_status = 0x03;
int bq24196_is_charging = 0;


enum charger_stat {
	BQ24196_ABSENT = 0,
	BQ24196_PRESENT,
	BQ24196_ENUMERATED,
};

//static enum CHG_TYPE g_charger_type = 0;

void pchr_turn_off_charging_bq24196 (struct bq24196_data *chip);
void bq24196_otg_power(int on);
void pchr_turn_on_charging_bq24196 (struct bq24196_data *chip);

static struct bq24196_data *usb_bq24196_chg = NULL;
#if 1 
static int bq24196_read_reg(struct i2c_client *client, int reg,
	u8 *val)
{
	s32 ret;
	struct bq24196_data *bq24196_chg;
	
	bq24196_chg = i2c_get_clientdata(client);
	
	mutex_lock(&bq27541_i2c_access);
	ret = i2c_smbus_read_byte_data(bq24196_chg->client, reg);
	mutex_unlock(&bq27541_i2c_access);
	if (ret < 0) {
		dev_err(&bq24196_chg->client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else
		*val = ret;
	
	return 0;
}

static int bq24196_write_reg(struct i2c_client *client, int reg,
	u8 val)
{
	s32 ret;
	struct bq24196_data *bq24196_chg;
	

	bq24196_chg = i2c_get_clientdata(client);
	mutex_lock(&bq27541_i2c_access);
	ret = i2c_smbus_write_byte_data(bq24196_chg->client, reg, val);
	mutex_unlock(&bq27541_i2c_access);
	if (ret < 0) {
		dev_err(&bq24196_chg->client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}

	return 0;
}
#else
static int bq24196_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;
	/*pr_bq24196(IO, "%s, reg=%x,val=%x\n",__func__, reg, value);*/

	ret = i2c_smbus_write_byte_data(client, reg, value); 

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int bq24196_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg); 

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	
	/*pr_bq24196(IO, "%s, reg=%x,val_src=%x\n",__func__, reg, ret);*/
	return ret;
}

#endif

#if 1 
int g_bq24196_log_en = 1;

/*bq24196 register register function begin*/
kal_uint32 bq24196_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24196_reg = 0;
    int ret = 0;
    struct i2c_client *client = usb_bq24196_chg->client ;
//    printk("--------------------------------------------------\n");

    ret = bq24196_read_reg(client,RegNum, &bq24196_reg);
    printk("[bq24196_read_interface] Reg[%x]=0x%x\n", RegNum, bq24196_reg);
    
    bq24196_reg &= (MASK << SHIFT);
    *val = (bq24196_reg >> SHIFT);    
    printk("[bq24196_read_interface] Val=0x%x\n", *val);

    return ret;
}

kal_uint32 bq24196_config_interface (/*struct i2c_client *client,*/kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24196_reg = 0;
    int ret = 0;
    struct i2c_client *client = usb_bq24196_chg->client ;

//    printk("--------------------------------------------------\n");

    ret = bq24196_read_reg(client,RegNum, &bq24196_reg);
    printk("[bq24196_config_interface]read Reg[%x]=0x%x\n", RegNum, bq24196_reg);
    
    bq24196_reg &= ~(MASK << SHIFT);
    bq24196_reg |= (val << SHIFT);
	
    ret = bq24196_write_reg(client,RegNum, bq24196_reg);
	
    printk("[bq24196_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24196_reg);

    return ret;
}

void bq24196_set_en_hiz(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    /*bq24196 pg pin for vubs mos chip enbalbe signal,so never set hig-impedance mode */
    if(val != 0)
    {
        if(CHARGING_LOG)
            printk(KERN_ERR "%s,not allow set non-zero value,return\n",__func__);

        return ;
    }

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_EN_HIZ_MASK),
                                    (kal_uint8)(CON0_EN_HIZ_SHIFT)
                                    );

if(CHARGING_LOG)
	printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
	
}

void bq24196_set_vindpm(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_VINDPM_MASK),
                                    (kal_uint8)(CON0_VINDPM_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_iinlim(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_IINLIM_MASK),
                                    (kal_uint8)(CON0_IINLIM_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

//CON1----------------------------------------------------

void bq24196_set_reg_rst(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_REG_RST_MASK),
                                    (kal_uint8)(CON1_REG_RST_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_wdt_rst(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_WDT_RST_MASK),
                                    (kal_uint8)(CON1_WDT_RST_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_chg_config(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_CHG_CONFIG_MASK),
                                    (kal_uint8)(CON1_CHG_CONFIG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_sys_min(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_SYS_MIN_MASK),
                                    (kal_uint8)(CON1_SYS_MIN_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_boost_lim(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_BOOST_LIM_MASK),
                                    (kal_uint8)(CON1_BOOST_LIM_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

//CON2----------------------------------------------------

void bq24196_set_ichg(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_ICHG_MASK),
                                    (kal_uint8)(CON2_ICHG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

//CON3----------------------------------------------------

void bq24196_set_iprechg(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON3), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON3_IPRECHG_MASK),
                                    (kal_uint8)(CON3_IPRECHG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_iterm(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON3), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON3_ITERM_MASK),
                                    (kal_uint8)(CON3_ITERM_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

//CON4----------------------------------------------------

void bq24196_set_vreg(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_VREG_MASK),
                                    (kal_uint8)(CON4_VREG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_batlowv(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_BATLOWV_MASK),
                                    (kal_uint8)(CON4_BATLOWV_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_vrechg(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_VRECHG_MASK),
                                    (kal_uint8)(CON4_VRECHG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

//CON5----------------------------------------------------

void bq24196_set_en_term(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_EN_TERM_MASK),
                                    (kal_uint8)(CON5_EN_TERM_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_term_stat(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_TERM_STAT_MASK),
                                    (kal_uint8)(CON5_TERM_STAT_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_watchdog(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_WATCHDOG_MASK),
                                    (kal_uint8)(CON5_WATCHDOG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

void bq24196_set_en_timer(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_EN_TIMER_MASK),
                                    (kal_uint8)(CON5_EN_TIMER_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

void bq24196_set_chg_timer(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_CHG_TIMER_MASK),
                                    (kal_uint8)(CON5_CHG_TIMER_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);

}

//CON6----------------------------------------------------

void bq24196_set_treg(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_TREG_MASK),
                                    (kal_uint8)(CON6_TREG_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

//CON7----------------------------------------------------

void bq24196_set_tmr2x_en(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_TMR2X_EN_MASK),
                                    (kal_uint8)(CON7_TMR2X_EN_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

void bq24196_set_batfet_disable(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_BATFET_Disable_MASK),
                                    (kal_uint8)(CON7_BATFET_Disable_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

void bq24196_set_int_mask(struct i2c_client *client,kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24196_config_interface(   (kal_uint8)(bq24196_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_INT_MASK_MASK),
                                    (kal_uint8)(CON7_INT_MASK_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
}

//CON8----------------------------------------------------

kal_uint32 bq24196_get_system_status(struct i2c_client *client)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON8), 
                                    (&val),
                                    (kal_uint8)(0xFF),
                                    (kal_uint8)(0x0)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
    return val;
}

kal_uint32 bq24196_get_vbus_stat(struct i2c_client *client)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_VBUS_STAT_MASK),
                                    (kal_uint8)(CON8_VBUS_STAT_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
    return val;
}

kal_uint32 bq24196_get_chrg_stat(struct i2c_client *client)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;
	
    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_CHRG_STAT_MASK),
                                    (kal_uint8)(CON8_CHRG_STAT_SHIFT)
                                    );
	
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
    return val;
}

kal_uint32 bq24196_get_vsys_stat(struct i2c_client *client)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_VSYS_STAT_MASK),
                                    (kal_uint8)(CON8_VSYS_STAT_SHIFT)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
    return val;
}

 int bq24196_get_chrg_current(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;
    int current_chg=0;

    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON2), 
				    (&val),
                                    (kal_uint8)(0x1f),
                                    (kal_uint8)(2)
                                    );
	current_chg = 64  * val;//ma
	current_chg= 500 + current_chg;
    if(0)
        printk(KERN_ERR "%s,ret=%d ,val=%d,current=%d\n",__func__,ret,val,current_chg);

    return current_chg;
}
//CON9--------------------------------------------------------------
kal_uint32 bq24196_get_otg_fault(struct i2c_client *client)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24196_read_interface(     (kal_uint8)(bq24196_CON9),
                                    (&val),
                                    (kal_uint8)(0x01),
                                    (kal_uint8)(0x6)
                                    );
    if(CHARGING_LOG)
        printk(KERN_ERR "%s,ret=%d ,val=%d\n",__func__,ret,val);
    return val;
}


/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
#if 0
void bq24196_dump_register(void)
{
    U8 chip_slave_address = bq24196_SLAVE_ADDR_WRITE;
    U8 cmd = 0x0;
    int cmd_len = 1;
    U8 data = 0xFF;
    int data_len = 1;
    int i=0;
    for (i=0;i<bq24196_REG_NUM;i++)
    {
        cmd = i;
        bq24196_i2c_read(chip_slave_address, &cmd, cmd_len, &data, data_len);
        bq24196_reg[i] = data;
        printf("[bq24196_dump_register] Reg[0x%X]=0x%X\n", i, bq24196_reg[i]);
    }	
}
#endif

/*bq24196 register register function end*/

#ifdef REG_POWER_SUPPLY
static int bq24196_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct bq24196_chip *chip = container_of(psy,
				struct bq24196_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif //#ifdef REG_POWER_SUPPLY

static int bq24196_default_setting(struct i2c_client *client)
{
	int ret = 0;

	if(client == NULL)
		printk(KERN_ERR "%s null pointer client",__func__);

	bq24196_set_en_hiz(client,0x0);
	bq24196_set_vindpm(client,0x08); //VIN DPM check 4.54Vs
	bq24196_set_reg_rst(client,0x0);
	bq24196_set_wdt_rst(client,0x1); //Kick watchdog	
	bq24196_set_sys_min(client,0x5); //Minimum system voltage 3.5V	
	bq24196_set_iprechg(client,0x3); //Precharge current 512mA
	bq24196_set_iterm(client,0x1);   //Termination current 128mA->256
	bq24196_set_vreg(client,0x36);   //VREG 4.35V->4.368 for bat to battery cell resistance
	bq24196_set_batlowv(client,0x1); //BATLOWV 3.0V
	bq24196_set_vrechg(client,0x0);  //VRECHG 0.1V (4.108V)
	bq24196_set_en_term(client,0x1); //Enable termination
	bq24196_set_term_stat(client,0x0); //Match ITERM
	bq24196_set_watchdog(client,0x1); //WDT 40s
	bq24196_set_en_timer(client,0x0); //Disable charge timer
	bq24196_set_int_mask(client,0x0); //Disable fault interrupt
	

	return ret;
}

#endif
//bq24196 function end


static ssize_t id_reg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct bq24196_data *bq24196_chg;

	bq24196_chg = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%02x\n", bq24196_chg->dev_id_reg);
}
static DEVICE_ATTR(id_reg, S_IRUGO | S_IWUSR, id_reg_show, NULL);

int bq24196_usb_temperature_control(struct bq24196_data *chip,int temp)
{
	int tend = 0;
	static int old_temp = 0;
	int STEP_TEMP = 1 ;

	if(temp!=old_temp) 
	{
		if(temp>old_temp)
			tend = 1;
		else
			tend = -1;
		
		old_temp = temp;
	}
	
	if(temp >= (60+tend*STEP_TEMP))
	{
			//shutdown machine
		printk(KERN_ERR "%s: over temperaure ,shutdown",__func__);
                //over tem shutdown have been remove in framework system UI layter
		//kernel_power_off();
	}		
		return 0;
}

int bq24196_ac_temperature_control(struct bq24196_data *chip,int temp)
{
	unsigned char flag = 0;
	int tend = 0;
	int charging_flag = 0;
	int charging_current = 0;
	static int old_temp = 0;
	int STEP_TEMP = 1 ;
	if(temp!=old_temp) 
	{
		if(temp>old_temp)
			tend = 1;
		else
			tend = -1;
		
		old_temp = temp;
	}

	if(temp < (0+tend*STEP_TEMP)) //disable charging below zero temperature degree
	{
		//stop charging
		charging_flag = CHARGING_STOP;
	}
	else if((temp>=(0+tend*STEP_TEMP))&&(temp<(10+tend*STEP_TEMP)))
	{
		charging_flag = CHARGING_COLD;
		charging_current = 884;
	}else if((temp>=(10+tend*STEP_TEMP))&&(temp<(45+tend*STEP_TEMP)))
	{
		charging_flag = CHARGING_NORMAL;
		charging_current = AC_CHARGING_CURRENT;
	}else if((temp>=(45+tend*STEP_TEMP))&&(temp<(50+tend*STEP_TEMP)))
	{
		charging_flag = CHARGING_HOT;
		charging_current = 884;
	}else if((temp >= (50+tend*STEP_TEMP))&&(temp<(60+tend*STEP_TEMP)))
	{
		//stop charging but keep powerpth on
		charging_flag = CHARGING_STOP;
		charging_current = 0;
	}else if(temp >= (60+tend*STEP_TEMP))
	{
		//shutdown machine
		charging_flag = CHARGING_STOP;
		charging_current = 0;
		printk(KERN_ERR "%s: over temperaure ,shutdown",__func__);
                //over tem shutdown have been remove in framework system UI layter
		//kernel_power_off();
		return 0;
	}

	pr_bq24196(INFO, "%s: temperature = %d, charging current = %d charging_temp:%d, charging_flag:%d\r\n",__func__, temp, charging_current,chip->charging_temp_flag,charging_flag);	
	if (CHARGING_STOP == charging_flag)
	{
		pr_bq24196(INFO, "temp_control start charging \n");
		pchr_turn_off_charging_bq24196(chip);
	}
	else
	{
		bq24196_set_en_hiz(chip->client,0x0); //powerpath on	        	
		bq24196_set_chg_config(chip->client,0x1); // charger enable
		bq24196_set_ichg(chip->client,(charging_current-500)/64);
	}
	chip->charging_temp_flag = charging_flag;

	pr_bq24196(INFO, "%s: flag = 0x%x charging_temp_flag=%d\r\n",__func__, flag, chip->charging_temp_flag);	
	return charging_flag;
}

void bq24196_dump_register(struct i2c_client *client)
{
	u8 ret0, ret1, ret2, ret3, ret4, ret5,ret6,ret7,ret8,ret9;

	 bq24196_read_reg(client, 0x00,&ret0);
	 bq24196_read_reg(client, 0x01,&ret1);
	 bq24196_read_reg(client, 0x02,&ret2);
	 bq24196_read_reg(client, 0x03,&ret3);
	 bq24196_read_reg(client, 0x04,&ret4);
	 bq24196_read_reg(client, 0x05,&ret5);
	bq24196_read_reg(client, 0x06,&ret6);
	bq24196_read_reg(client, 0x07,&ret7);
	bq24196_read_reg(client, 0x08,&ret8);
	bq24196_read_reg(client, 0x09,&ret9);

	dev_err(&client->dev, "bq24196 [REG 00]%x, [1]%x, [2]%x, [3]%x, [4]%x, [5]%x,[6]%x,[7]%x,[8]%x,[9]%x\n", ret0,ret1,ret2,ret3,ret4,ret5,ret6,ret7,ret8,ret9);
}

enum CHG_TYPE bq24196_get_chg_type(void)
{
	return usb_bq24196_chg->charger_type;
}

//0 not  1 present
int bq24196_is_charger_present(void)
{
	if(usb_bq24196_chg == NULL)
		return 0;

	return usb_bq24196_chg->usb_status ;
	
}
void notify_bq24196_send_recharging_event(void)
{
	if(usb_bq24196_chg == NULL)
		return ;
	
	msm_charger_notify_event(&usb_bq24196_chg->adapter_hw_chg,
					CHG_BATT_NEEDS_RECHARGING);
}


void notify_chg_inserted_event(enum charger_stat state)
{
    struct bq24196_data * bq24196_chg = usb_bq24196_chg;

	if(bq24196_chg == NULL)
		return ;

	if(CHARGING_LOG)
        printk(KERN_ERR "%s :0absent,1present,2enum: %d",__func__, state);

	if(bq24196_chg != NULL)
	{
		switch(state){
			case BQ24196_ABSENT:
				if(bq24196_chg->usb_status == BQ24196_ABSENT){
				    printk(KERN_ERR "%s : have been in absent status",__func__);
					break;
				}
				//remove the manual control flag while plug in/out charger/usb
				bq24196_clear_manual_control_flag();
				msm_charger_notify_event(&bq24196_chg->adapter_hw_chg,CHG_REMOVED_EVENT);
				bq27541_stop_monitor_recharging();
				bq24196_is_charging = 0;
				bq24196_chg->usb_status = BQ24196_ABSENT;
				break;
			case BQ24196_PRESENT:
			case BQ24196_ENUMERATED:
				if(bq24196_chg->usb_status == BQ24196_PRESENT){
				    printk(KERN_ERR "%s : have been in present status",__func__);
					break;
				}
				//remove the manual control flag while plug in/out charger/usb
				//bq24196_clear_manual_control_flag();
				msm_charger_notify_event(&bq24196_chg->adapter_hw_chg,CHG_INSERTED_EVENT);
				bq24196_chg->usb_status = BQ24196_PRESENT;
				break;
			default:
				break;
		}
		wake_lock_timeout(&bq24196_chg->charging_lock, 2 * HZ);
	}
}

//notify usb type in msm_otg.c
void bq24196_otg_notify_power_supply_type(enum power_supply_type type)
{
    struct bq24196_data * bq24196_chg = usb_bq24196_chg;
	if(bq24196_chg == NULL)
		return;	

	if(CHARGING_LOG)
		printk(KERN_ERR "bq24196_otg_notify_power_supply_type:0invald 1usb 2charger: %d",type);

	switch(type){
	case POWER_SUPPLY_TYPE_USB_DCP://charger
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_ACA:
		bq24196_chg->charger_type = CHARGER_NORMAL;
		notify_chg_inserted_event(BQ24196_PRESENT);
		break;
	case POWER_SUPPLY_TYPE_USB: //usb
		bq24196_chg->charger_type  = CHARGER_USB;
		notify_chg_inserted_event(BQ24196_PRESENT);
 		break;
	case POWER_SUPPLY_TYPE_UNKNOWN: //plug out
		bq24196_chg->charger_type  = CHARGER_NONE ;
		notify_chg_inserted_event(BQ24196_ABSENT);
		break;
	default: //plug out
		bq24196_chg->charger_type  = CHARGER_NONE ;
		notify_chg_inserted_event(BQ24196_ABSENT);
		break;
	}
}
EXPORT_SYMBOL(bq24196_otg_notify_power_supply_type);	

void pchr_turn_off_charging_bq24196 (struct bq24196_data *chip)
{

	printk(KERN_ERR "%s",__func__);

	bq24196_set_wdt_rst(chip->client,0x1); //Kick watchdog
	bq24196_set_watchdog(chip->client,0x0);//disable watchdog timer
	bq24196_set_chg_config(chip->client,0x0);  //no charging

	if(chip->usb_status == BQ24196_ABSENT)
	{
		bq24196_set_en_hiz(chip->client,0x1);        //power path shutdown
	}else {
		bq24196_set_en_hiz(chip->client,0x0);  		//keep power path on
	}

	chip->charging_temp_flag = CHARGING_STOP;

}

// keep chg and enhiz on after plug out usb/chareger
void pchr_turn_off_charging_bq24196_plugout_usb(struct bq24196_data *chip)
{
        printk(KERN_ERR "%s",__func__);

        bq24196_set_wdt_rst(chip->client,0x1); //Kick watchdog
	bq24196_set_watchdog(chip->client,0x0);//disable watchdog timer
	bq24196_set_chg_config(chip->client,0x0);

        bq24196_set_en_hiz(chip->client,0x0);        //power path on

        chip->charging_temp_flag = CHARGING_STOP;

        chip->charging = false;
        chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
        chip->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
}

void pchr_turn_on_charging_bq24196 (struct bq24196_data *chip)
{
	enum CHG_TYPE chg_type = CHARGER_NONE;
	int bat_temp = 0 ;
	int val = 0;
	bq24196_dump_register(chip->client);
	//enable ce pin
	gpio_set_value_cansleep(chip->ce_gpio,0);
	if (val < 0) {
		dev_err(&chip->client->dev,
			"%s gpio_get_value failed for %d ret=%d\n", __func__,
			chip->ce_gpio, val);
		return;
	}
	
	//bq24196 init register
	bq24196_default_setting(chip->client);
	chg_type = bq24196_get_chg_type();
    dev_err(&chip->client->dev, "%s 0 non, 1 usb, 2 charger, chg_type = %d\n", __func__, chg_type);

	bat_temp = bq27541_get_battery_temperature();
	bat_temp = (bat_temp/10) ;
	printk(KERN_ERR "Battery Temperature :%d\n",bat_temp);

	if(chg_type == CHARGER_NORMAL)
	{
		bq24196_set_iinlim(chip->client,0x6); // 2A
		bq24196_ac_temperature_control(chip, bat_temp);
	}else if(chg_type == CHARGER_USB)
	{
		bq24196_set_iinlim(chip->client,0x2); // 500 mA
		bq24196_set_ichg(chip->client,0); //500mA
		bq24196_usb_temperature_control(chip, bat_temp);
	}else //plug out
	{
		pchr_turn_off_charging_bq24196(chip);
		chip->charging = false;
		chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return;
	}
    bq24196_set_en_hiz(chip->client,0x0); //powerpath on	        	
    bq24196_set_chg_config(chip->client,0x1); // charger enable

	//dump 24196 register
	bq24196_dump_register(chip->client);
	
}


static int bq24196_start_charging(struct msm_hardware_charger *hw_chg,
		int chg_voltage, int chg_current)
{
	int ret = 0;
	struct bq24196_data *bq24196_chg = usb_bq24196_chg;
	int bat_status = 0;

	if(CHARGING_LOG)
		printk(KERN_ERR "%s",__func__);

	if(bq24196_chg == NULL)
		return -1;

	if (bq24196_chg->charging == true)
		/* we are already charging with the same current*/
		 dev_err(&bq24196_chg->client->dev,"%s charge with same current called again\n", __func__);

	//turn on charger
	pchr_turn_on_charging_bq24196(bq24196_chg);
	
	bq24196_chg->charging = true;
	bq24196_chg->batt_status = POWER_SUPPLY_STATUS_CHARGING;
	bq24196_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

    bat_status = bq24196_get_chrg_stat(bq24196_chg->client);
		
	if (bq24196_chg->charging ) 
	{
		if (bat_status > 0x01) { //fast cc
			bq24196_chg->batt_chg_type
				= POWER_SUPPLY_CHARGE_TYPE_FAST;
			msm_charger_notify_event(
				&bq24196_chg->adapter_hw_chg,
				CHG_BATT_BEGIN_FAST_CHARGING);
		} else {
			bq24196_chg->batt_chg_type
				= POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		}
	  }

	/*schedule charge_work to keep track of battery charging state*/
	schedule_delayed_work(&bq24196_chg->charge_work, BQ24196_CHG_PERIOD);
	charge_bat_status = 0;
	bq24196_is_charging = 1;

	return ret;
}

static int bq24196_stop_charging(struct msm_hardware_charger *hw_chg)
{
	int ret = 0;
	struct bq24196_data *bq24196_chg;

	bq24196_chg = container_of(hw_chg, struct bq24196_data, adapter_hw_chg);

	dev_err(&bq24196_chg->client->dev, "%s\n", __func__);
	if (bq24196_chg->charging == false)
		dev_err(&bq24196_chg->client->dev, "%s have been in stop charging status\n",__func__);


	bq24196_chg->charging = false;
	bq24196_chg->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	bq24196_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;

	cancel_delayed_work_sync(&bq24196_chg->charge_work);
	//pchr_turn_off_charging_bq24196(bq24196_chg);
	pchr_turn_off_charging_bq24196_plugout_usb(bq24196_chg);	
	if (ret)
		dev_err(&bq24196_chg->client->dev,
			"%s couldn't write to pin ctrl reg\n", __func__);

	charge_bat_status = 3;
	bq24196_is_charging = 0;
	
	return ret;
}

static int bq24196_charger_switch(struct msm_hardware_charger *hw_chg)
{
	struct bq24196_data *bq24196_chg;

	bq24196_chg = container_of(hw_chg, struct bq24196_data, adapter_hw_chg);
	dev_dbg(&bq24196_chg->client->dev, "%s\n", __func__);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;
static int debug_fs_otg;
static int otg_get(void *data, u64 *value)
{
	*value = debug_fs_otg;
	return 0;
}
static int otg_set(void *data, u64 value)
{
	bq24196_otg_power(debug_fs_otg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq24196_otg_fops, otg_get, otg_set, "%llu\n");

static void bq24196_create_debugfs_entries(struct bq24196_data *bq24196_chg)
{
	dent = debugfs_create_dir("bq24196", NULL);
	if (dent) {
		debugfs_create_file("otg", 0644, dent, NULL, &bq24196_otg_fops);
	}
}
static void bq24196_destroy_debugfs_entries(void)
{
	if (dent)
		debugfs_remove_recursive(dent);
}
#else
static void bq24196_create_debugfs_entries(struct bq24196_data *bq24196_chg)
{
}
static void bq24196_destroy_debugfs_entries(void)
{
}
#endif


static ssize_t bq24196_temperature_control_show(struct device* dev,
				struct device_attribute *attr, char* buf)
{
	ssize_t count;
	int bat_temp = 0;
	
	bat_temp = bq27541_get_battery_temperature();
	
	bat_temp = (bat_temp/10) ;
	count = sprintf(buf, "%d\n", bat_temp);

	printk(KERN_ERR "%s,%d",__func__,bat_temp);

	return count;
}

void bq24196_clear_manual_control_flag(void)
{
	bq24196_manual_control = 0;
	bq24196_manual_temperature = 20;//bq27541_get_battery_temperature();
}
static ssize_t bq24196_temperature_control_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	int error = 0;
	unsigned long temp = 0;

	bq24196_manual_control = 1;

	printk(KERN_ERR "%s",__func__);

	error = kstrtoul(buf,10,&temp);
	if(error)
		return error;
	
	bq24196_manual_temperature = (int)temp;

	return size;
}


static DEVICE_ATTR(bq24196_temperature_control, S_IRUGO|S_IWUSR, bq24196_temperature_control_show, bq24196_temperature_control_store);


void bq24196_control_charger_status(int value)
{
	struct bq24196_data *bq24196_chg = usb_bq24196_chg;

	if(bq24196_chg == NULL)
		return;
	printk(KERN_ERR "control:%d",value);
	if(value <= 0 ) 
	{
		//shutdown charging
		bq24196_set_chg_config(bq24196_chg->client,0x0);
		bq24196_set_en_hiz(bq24196_chg->client,0x1);
		bq24196_chg->charging = false;
	}
	else
	{
		//start charging
		bq24196_set_en_hiz(bq24196_chg->client,0x0);
                bq24196_set_chg_config(bq24196_chg->client,0x1);
		bq24196_chg->charging = true;
	}
}

int bq24196_read_charger_status(void)
{
        struct bq24196_data *bq24196_chg = usb_bq24196_chg;
	int temp = 0;

        if(bq24196_chg == NULL)
                return -1;
	if(bq24196_chg->charging == true)
		temp = 1;
	else
		temp = 0;

	printk(KERN_ERR "read ce pin:%d",temp);
	return temp;
}

static ssize_t bq24196_charger_enabled_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{
        int error = 0;
        unsigned long temp = 0;

        printk(KERN_ERR "%s",__func__);

        error = kstrtoul(buf,10,&temp);
        if(error)
                return error;

	bq24196_control_charger_status(temp);

        return size;
}

static ssize_t bq24196_charger_enabled_show(struct device* dev,
                                struct device_attribute *attr, char* buf)
{
        ssize_t count;
        int temp = 0;

        temp = bq24196_read_charger_status();

        count = sprintf(buf, "%d\n", temp);

        printk(KERN_ERR "%s,%d",__func__,temp);

        return count;
}

static DEVICE_ATTR(bq24196_charger_enabled, S_IRUGO|S_IWUSR, bq24196_charger_enabled_show, bq24196_charger_enabled_store);

static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (usb_bq24196_chg && disabled)
		msm_charger_notify_event(&usb_bq24196_chg->adapter_hw_chg,
				CHG_DONE_EVENT);

	pr_debug("%s disabled =%d\n", __func__, disabled);
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&disabled, 0644);

static void bq24196_charge_sm(struct work_struct *bq24196_work)
{
	struct bq24196_data *bq24196_chg = usb_bq24196_chg;
	int bat_temp = 0;
	enum CHG_TYPE chg_type = CHARGER_NONE ;
	
	if(bq24196_chg == NULL)
                return;

	/*if not charging, exit bq24196 charging state transition*/
        if (!bq24196_chg->charging)
                return;

	bq24196_set_wdt_rst(bq24196_chg->client,0x1); //Kick watchdog
	bq24196_set_watchdog(bq24196_chg->client,0x1); //40s
	if(CHARGING_LOG)
		dev_err(&bq24196_chg->client->dev, "----------------------%s begin ----------------------\n", __func__);

        if(CHARGING_LOG)
                bq24196_dump_register(bq24196_chg->client);

	bq24196_chg->batt_present  = bq27541_is_battery_present();

	if (!bq24196_chg->batt_present){
		dev_err(&bq24196_chg->client->dev, "%s battery not present\n", __func__);
		bq24196_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return ;
	}

	//check the battery real full
       charge_bat_status = bq24196_get_chrg_stat(bq24196_chg->client);

	if(CHARGING_LOG)
        printk(KERN_ERR "%s, charging stat 1 pre, 2 cc, 3 full :%d \n",__func__ , charge_bat_status );

    if(100 ==  bq27541_get_real_battery_capacity()&& charge_bat_status == 0x3)
    {
		msm_charger_notify_event(&bq24196_chg->adapter_hw_chg,
					CHG_DONE_EVENT);
		dev_err(&bq24196_chg->client->dev, "%s battery real full ,stop charging\n", __func__);
     }

     if (bq24196_chg->batt_present && bq24196_chg->charging && (bq24196_chg->batt_chg_type!= POWER_SUPPLY_CHARGE_TYPE_FAST)) 
	 {
		if (charge_bat_status > 0x01) { //fast cc
			bq24196_chg->batt_chg_type
				= POWER_SUPPLY_CHARGE_TYPE_FAST;
			msm_charger_notify_event(
				&bq24196_chg->adapter_hw_chg,
				CHG_BATT_BEGIN_FAST_CHARGING);
		} else {
			bq24196_chg->batt_chg_type
				= POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		}
	  }

	chg_type = bq24196_get_chg_type();

	//manual control the temperatur for testing
	if(bq24196_manual_control == 1) //test mode
		bat_temp = bq24196_manual_temperature ;
	else //real mode
		bat_temp = bq27541_get_battery_temperature();

	bat_temp = (bat_temp/10) ;
	printk(KERN_ERR "%s Battery Temperature :%d \n",__func__,bat_temp);

	if(chg_type == CHARGER_NORMAL)
	{
		bq24196_set_iinlim(bq24196_chg->client,0x6);//inlim 2A
		bq24196_ac_temperature_control(bq24196_chg, bat_temp);
	}else if(chg_type == CHARGER_USB)
	{
		bq24196_set_iinlim(bq24196_chg->client,0x2);//inlim 500mA
		bq24196_usb_temperature_control(bq24196_chg, bat_temp);
	}

	if(CHARGING_LOG)
		bq24196_dump_register(bq24196_chg->client);

	if(CHARGING_LOG)
		dev_err(&bq24196_chg->client->dev, "----------------------%s end ----------------------\n", __func__);

	schedule_delayed_work(&bq24196_chg->charge_work,
					BQ24196_CHG_PERIOD);
}

static void __bq24196_otg_power(struct i2c_client *client, int on)
{
    printk(KERN_ERR "%s ,%d",__func__,on);

    if(on)
    {
	
        bq24196_set_chg_config(client,0x3); //OTG
	#ifdef LENOVO_BQ24296_SUPPORT
//	bq24196_set_boost_V(0x8);
	#endif
        bq24196_set_boost_lim(client,0x0); //500mA on VBUS
        bq24196_set_en_hiz(client,0x0);     
	bq24196_set_watchdog(client,0x00); //disable timer
	// from the schematic , the OTG pin connect to battery. so it is always high
        //OTG pin pull high, maybe can consider OTG pin always high
//        mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN,GPIO_MODE_GPIO);  
//        mt_set_gpio_dir(GPIO_OTG_DRVVBUS_PIN,GPIO_DIR_OUT);
//        mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN,GPIO_OUT_ONE);           
    }
    else
    {
        bq24196_set_chg_config(client,0x0); //OTG & Charge disabled
        //OTG pin pull low, maybe can consider OTG pin always high
//        mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN,GPIO_MODE_GPIO);  
//        mt_set_gpio_dir(GPIO_OTG_DRVVBUS_PIN,GPIO_DIR_OUT);
//        mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN,GPIO_OUT_ZERO);                   
    }

}

void bq24196_otg_poweroff_stop(int on)
{
	struct i2c_client *client = usb_bq24196_chg->client ;
	bq24196_set_chg_config(client,0x0); //to stop otg charge when power off 
}


static void bq24196_otg_power_work(struct work_struct *bq24196_work)
{
        struct bq24196_data *data = container_of(bq24196_work,struct bq24196_data,otg_power_work.work);

        if(data == NULL)
                return ;

        if(CHARGING_LOG)
        printk(KERN_ERR "%s ,%d",__func__,data->otg_power_status);

        __bq24196_otg_power(data->client,data->otg_power_status);

        cancel_delayed_work(&data->otg_power_work);
}

static void bq24196_otg_check(struct bq24196_data *data)
{
	if(CHARGING_LOG)
		printk(KERN_ERR "%s",__func__);
        if(bq24196_get_otg_fault(data->client) == 0x01)
        {
                 __bq24196_otg_power(data->client,0);
        }
}

static void bq24196_otg_protect(struct work_struct *bq24196_work)
{
	struct bq24196_data *data = container_of(bq24196_work,struct bq24196_data,otg_work.work);

	if(data == NULL)
		return ;

	if(CHARGING_LOG)
        printk(KERN_ERR "%s ,%d",__func__,bq24196_get_otg_fault(data->client));

        bq24196_otg_check(data);

	cancel_delayed_work(&data->otg_work);
}

static irqreturn_t bq24196_otg_handler(int irq, void *dev_id)
{
	struct bq24196_data *data = dev_id;
	if(CHARGING_LOG)
	printk(KERN_ERR "%s",__func__);
//	schedule_delayed_work(&data->otg_work, BQ24196_OTG_PERIOD);
	if(data == NULL)
              return IRQ_HANDLED ;
 
	if(data->suspend_flag && !data->otg_interrupt_flag) {
		if(CHARGING_LOG)
			pr_info("%s: device not resume\n", __func__);
		mutex_lock(&data->lock);
		data->otg_interrupt_flag = true;
		mutex_unlock(&data->lock);
	} else {
		printk(KERN_ERR "%s ,%d",__func__,bq24196_get_otg_fault(data->client));
		bq24196_otg_check(data);
	}

	return IRQ_HANDLED;
}

static int __devinit bq24196_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct bq24196_data *bq24196_chg = NULL;
	int ret = 0;
	struct device_node *dev_node = client->dev.of_node;

	printk(KERN_ERR "bq24196_probe");

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto out;
	}

	bq24196_chg = kzalloc(sizeof(*bq24196_chg), GFP_KERNEL);
	if (!bq24196_chg) {
		ret = -ENOMEM;
		goto out;
	}

	INIT_DELAYED_WORK(&bq24196_chg->charge_work, bq24196_charge_sm);
	INIT_DELAYED_WORK(&bq24196_chg->otg_work, bq24196_otg_protect);
	INIT_DELAYED_WORK(&bq24196_chg->otg_power_work, bq24196_otg_power_work);
	bq24196_chg->client = client;

	//It supports charger and PC USB charger
	bq24196_chg->adapter_hw_chg.type = CHG_TYPE_USB;
	bq24196_chg->adapter_hw_chg.name = "bq24196-charger";
	bq24196_chg->adapter_hw_chg.start_charging = bq24196_start_charging;
	bq24196_chg->adapter_hw_chg.stop_charging = bq24196_stop_charging;
	bq24196_chg->adapter_hw_chg.charging_switched = bq24196_charger_switch;
	bq24196_chg->adapter_hw_chg.rating = 950;

#if 0
	if (pdata->chg_detection_config)
		ret = pdata->chg_detection_config();
	if (ret) {
		dev_err(&client->dev, "%s valid config failed ret=%d\n",
			__func__, ret);
		goto free_bq24196_chg;
	}
#endif

	bq24196_chg->ce_gpio =of_get_named_gpio(dev_node, "qcom,bq24196_chg_ce", 0);

	if(CHARGING_LOG)
	    dev_err(&client->dev, "%s chg_en_gpio:%d\n", __func__,bq24196_chg->ce_gpio );

	if(bq24196_chg->ce_gpio >= 0)
	{
        ret = gpio_request(bq24196_chg->ce_gpio, "bq24196_chg_ce");
        if(ret){
             dev_err(&client->dev, "%s request gpio ce ret=%d\n",__func__, ret);
                 goto free_valid_gpio ;
        }
    }

	i2c_set_clientdata(client, bq24196_chg);

	ret = msm_charger_register(&(bq24196_chg->adapter_hw_chg));
	if (ret) {
		dev_err(&client->dev, "%s msm_charger_register failed for ret=%d\n", __func__, ret);
		goto unregister_charger;
	}

#if 0
	ret = irq_set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
			 __func__, client->irq, ret);
		goto unregister_charger;
	}

	ret = request_threaded_irq(client->irq, NULL,
				   bq24196_valid_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "bq24196_charger_valid", client);
	if (ret) {
		dev_err(&client->dev,
			"%s request_threaded_irq failed for %d ret =%d\n",
			__func__, client->irq, ret);
		goto disable_irq_wake;
	}
#endif
	bq24196_chg->otg_irq =of_get_named_gpio(dev_node, "qcom,bq24196_status_int", 0);

        if(CHARGING_LOG)
            dev_err(&client->dev, "%s otg_irq:%d\n", __func__,bq24196_chg->otg_irq );

        if(bq24196_chg->otg_irq >= 0)
        {
        ret = gpio_request(bq24196_chg->otg_irq, "bq24196_status_int");
        if(ret){
             dev_err(&client->dev, "%s request gpio otg_irq ret=%d\n",__func__, ret);
                 goto free_otg_irq ;
               }
        }
	usb_bq24196_chg = bq24196_chg;
	ret = request_threaded_irq(gpio_to_irq(bq24196_chg->otg_irq), NULL,
				   bq24196_otg_handler,
				   IRQF_TRIGGER_LOW ,
				   "bq24196_status_int", bq24196_chg);
	if(CHARGING_LOG)
	printk(KERN_ERR "bq24196 irq:%d",gpio_to_irq(bq24196_chg->otg_irq));
	if (ret) {
		dev_err(&client->dev,
			"%s request_threaded_irq failed for %d ret =%d\n",
			__func__,gpio_to_irq(bq24196_chg->otg_irq), ret);
		goto free_otg_irq ;
	}
        //otg
        enable_irq_wake(gpio_to_irq(bq24196_chg->otg_irq));

	bq24196_read_reg(bq24196_chg->client, DEV_ID_REG,&(bq24196_chg->dev_id_reg ) );
	ret = device_create_file(&bq24196_chg->client->dev, &dev_attr_id_reg);
	dev_err(&client->dev,"%s dev_id : %d",__func__,bq24196_chg->dev_id_reg);

	/* TODO read min_design and max_design from chip registers */
	bq24196_chg->min_design = 3400;
	bq24196_chg->max_design = 4200;
	bq24196_chg->charging_temp_flag = CHARGING_STOP ;
	bq24196_chg->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	bq24196_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
	bq24196_chg->charger_type = CHARGER_NONE;

	device_init_wakeup(&client->dev, 1);
	mutex_init(&bq24196_chg->lock);
	bq24196_chg->suspend_flag = false;
	bq24196_chg->otg_interrupt_flag = false;

	mutex_lock(&init_lock);

	usb_bq24196_chg = bq24196_chg;
	if (init_otg_power)
		__bq24196_otg_power(bq24196_chg->client,init_otg_power);
	mutex_unlock(&init_lock);

	bq24196_create_debugfs_entries(bq24196_chg);

	if ((ret = device_create_file(&client->dev, &dev_attr_bq24196_temperature_control)))
	{
		dev_err(&client->dev, "failed: device_create_file temperature_control\n");
		goto unregister_debug_file;
	}

        if ((ret = device_create_file(&client->dev, &dev_attr_bq24196_charger_enabled)))
        {
                dev_err(&client->dev, "failed: device_create_file charger_enabled\n");
                goto unregister_debug_charger_enabled_file;
        }
        //enable chip path avoid chip in close status
        bq24196_set_en_hiz(bq24196_chg->client,0x0);

	dev_err(&client->dev,
		"%s OK device_id = %x chg_state=%d\n", __func__,
		bq24196_chg->dev_id_reg, bq24196_chg->usb_status);
	wake_lock_init(&bq24196_chg->charging_lock, WAKE_LOCK_SUSPEND, "charging wakelock");

	return 0;

//disable_irq_wake:
//	irq_set_irq_wake(client->irq, 0);
unregister_debug_charger_enabled_file:
	device_remove_file(&client->dev, &dev_attr_bq24196_temperature_control);

unregister_debug_file:
	msm_charger_unregister(&bq24196_chg->adapter_hw_chg);

unregister_charger:
	gpio_free(bq24196_chg->otg_irq);
free_otg_irq:
	gpio_free(bq24196_chg->ce_gpio);
free_valid_gpio:
	kfree(bq24196_chg);
out:
	return ret;
}

void bq24196_otg_power(int on)
{
	pr_debug("%s Enter on=%d\n", __func__, on);
	printk(KERN_ERR "%s,%d",__func__,on);
	mutex_lock(&init_lock);
	if (!usb_bq24196_chg) {
		init_otg_power = !!on;
		pr_warning("%s called when not initialized\n", __func__);
		mutex_unlock(&init_lock);
		return;
	}
	usb_bq24196_chg->otg_power_status = on ;
	if(on)
		__bq24196_otg_power(usb_bq24196_chg->client,on);
	else{
		schedule_delayed_work( &usb_bq24196_chg->otg_power_work , BQ24196_OTG_PWR_PERIOD );
	}
	mutex_unlock(&init_lock);
}

static int __devexit bq24196_remove(struct i2c_client *client)
{
	struct bq24196_data *bq24196_chg = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);
	//irq_set_irq_wake(client->irq, 0);
	//free_irq(client->irq, client);
	device_remove_file(&client->dev, &dev_attr_bq24196_temperature_control);
	device_remove_file(&client->dev, &dev_attr_bq24196_charger_enabled);
	gpio_free(bq24196_chg->ce_gpio);
	gpio_free(bq24196_chg->otg_irq);
	cancel_delayed_work_sync(&bq24196_chg->charge_work);
	cancel_delayed_work_sync(&bq24196_chg->otg_work);
	cancel_delayed_work_sync(&bq24196_chg->otg_power_work);
	msm_charger_notify_event(&bq24196_chg->adapter_hw_chg,
			 CHG_REMOVED_EVENT);
	msm_charger_unregister(&bq24196_chg->adapter_hw_chg);
	kfree(bq24196_chg);
	bq24196_destroy_debugfs_entries();

	return 0;
}
static void bq24196_shutdown(struct i2c_client *client)
{
	struct bq24196_data *bq24196_chg = i2c_get_clientdata(client);
	printk(KERN_ERR "bq24196_shutdown enter");
	bq24196_set_sys_min(bq24196_chg->client,0x7);
	bq24196_set_watchdog(bq24196_chg->client,0x0);
	bq24196_set_iinlim(bq24196_chg->client,0x2);
	bq24196_set_chg_config(bq24196_chg->client,0x1);
	printk(KERN_ERR "bq24196_shutdown exit");
}

#ifdef CONFIG_PM
static int bq24196_suspend(struct device *dev)
{
	struct bq24196_data *bq24196_chg = dev_get_drvdata(dev);

	mutex_lock(&bq24196_chg->lock);
	bq24196_chg->suspend_flag = true;
	mutex_unlock(&bq24196_chg->lock);
	if(CHARGING_LOG)
		dev_dbg(&bq24196_chg->client->dev, "%s\n", __func__);

 	return 0;
}

static int bq24196_resume(struct device *dev)
{
	struct bq24196_data *bq24196_chg = dev_get_drvdata(dev);

	if(CHARGING_LOG)
		pr_info("%s: enter\n", __func__);

	if (bq24196_chg->otg_interrupt_flag && bq24196_chg->suspend_flag) {
		if(CHARGING_LOG)
			pr_info("%s: has happened otg interrupt\n", __func__);
		bq24196_otg_check(bq24196_chg);
	}
	mutex_lock(&bq24196_chg->lock);
	bq24196_chg->otg_interrupt_flag = false;
	bq24196_chg->suspend_flag = false;
	mutex_unlock(&bq24196_chg->lock);
	if(CHARGING_LOG)
		dev_dbg(&bq24196_chg->client->dev, "%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops bq24196_pm_ops = {
	.suspend = bq24196_suspend,
	.resume = bq24196_resume,
};
#endif

static const struct i2c_device_id bq24196_id[] = {
	{"bq24196", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24196_id);
static struct i2c_driver bq24196_driver = {
	.driver = {
		   .name = "bq24196",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &bq24196_pm_ops,
#endif
	},
	.probe = bq24196_probe,
	.remove = __devexit_p(bq24196_remove),
	.id_table = bq24196_id,
	.shutdown = bq24196_shutdown,
};
static int __init bq24196_init(void)
{

    printk(KERN_ERR "bq24196");

	return i2c_add_driver(&bq24196_driver);
}
module_init(bq24196_init);

static void __exit bq24196_exit(void)
{
	return i2c_del_driver(&bq24196_driver);
}
module_exit(bq24196_exit);

MODULE_AUTHOR("xuwei9@lenovo.com");
MODULE_DESCRIPTION("Driver for BQ24196 Charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq24196");

