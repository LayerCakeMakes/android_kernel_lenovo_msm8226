

#ifndef __BQ24196_H__
#define __BQ24196_H__

/**
 * struct bq24196_platform_data
 * structure to pass board specific information to the bq24196 charger driver
 * @chgcurrent:	max current the bq24196 chip can draw
 * @valid_n_gpio:		gpio to debounce insertion/removal
 * @chg_detection_config:	machine specific func to configure
 *				insertion/removal gpio line
 * @batt_mah_rating:		the battery current rating
 */
struct bq24196_platform_data {
	int valid_n_gpio;
	int batt_mah_rating;
        int ce_gpio;

};

void bq24196_otg_power(int on);

//charging temperature status 
enum{
CHARGING_STOP = 0,
CHARGING_NORMAL,
CHARGING_HOT_WARNING,
CHARGING_COLD_WARNING,
CHARGING_HOT,
CHARGING_COLD,

};


//bq24196 definition begin

typedef unsigned char kal_uint8 ;
typedef unsigned short kal_uint16 ;
typedef unsigned int  kal_uint32 ;
//#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define bq24196_CON0      0x00
#define bq24196_CON1      0x01
#define bq24196_CON2      0x02
#define bq24196_CON3      0x03
#define bq24196_CON4      0x04
#define bq24196_CON5      0x05
#define bq24196_CON6      0x06
#define bq24196_CON7      0x07
#define bq24196_CON8      0x08
#define bq24196_CON9      0x09
#define bq24196_CON10      0x0A

/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
//CON0
#define CON0_EN_HIZ_MASK   0x01
#define CON0_EN_HIZ_SHIFT  7

#define CON0_VINDPM_MASK       0x0F
#define CON0_VINDPM_SHIFT      3

#define CON0_IINLIM_MASK   0x07
#define CON0_IINLIM_SHIFT  0

//CON1
#define CON1_REG_RST_MASK     0x01
#define CON1_REG_RST_SHIFT    7

#define CON1_WDT_RST_MASK     0x01
#define CON1_WDT_RST_SHIFT    6

#define CON1_CHG_CONFIG_MASK        0x03
#define CON1_CHG_CONFIG_SHIFT       4

#define CON1_SYS_MIN_MASK        0x07
#define CON1_SYS_MIN_SHIFT       1

#define CON1_BOOST_LIM_MASK   0x01
#define CON1_BOOST_LIM_SHIFT  0

#ifdef LENOVO_BQ24296_SUPPORT
#define CON6_BOOST_V_MASK   0x0F
#define CON6_BOOST_V_SHIFT  4
#endif

//CON2
#ifdef LENOVO_BQ24296_SUPPORT
#define CON2_ICHG_MASK    0x3F
#else
#define CON2_ICHG_MASK    0x1F
#endif
#define CON2_ICHG_SHIFT   2

//CON3
#define CON3_IPRECHG_MASK   0xF
#define CON3_IPRECHG_SHIFT  4

#define CON3_ITERM_MASK           0x0F
#define CON3_ITERM_SHIFT          0

//CON4
#define CON4_VREG_MASK     0x3F
#define CON4_VREG_SHIFT    2

#define CON4_BATLOWV_MASK     0x01
#define CON4_BATLOWV_SHIFT    1

#define CON4_VRECHG_MASK    0x01
#define CON4_VRECHG_SHIFT   0

//CON5
#define CON5_EN_TERM_MASK      0x01
#define CON5_EN_TERM_SHIFT     7

#define CON5_TERM_STAT_MASK      0x01
#define CON5_TERM_STAT_SHIFT     6

#define CON5_WATCHDOG_MASK     0x03
#define CON5_WATCHDOG_SHIFT    4

#define CON5_EN_TIMER_MASK      0x01
#define CON5_EN_TIMER_SHIFT     3

#define CON5_CHG_TIMER_MASK           0x03
#define CON5_CHG_TIMER_SHIFT          1

//CON6
#define CON6_TREG_MASK     0x03
#define CON6_TREG_SHIFT    0

//CON7
#define CON7_TMR2X_EN_MASK      0x01
#define CON7_TMR2X_EN_SHIFT     6

#define CON7_BATFET_Disable_MASK      0x01
#define CON7_BATFET_Disable_SHIFT     5

#define CON7_INT_MASK_MASK     0x03
#define CON7_INT_MASK_SHIFT    0

//CON8
#define CON8_VBUS_STAT_MASK      0x03
#define CON8_VBUS_STAT_SHIFT     6

#define CON8_CHRG_STAT_MASK           0x03
#define CON8_CHRG_STAT_SHIFT          4

#define CON8_DPM_STAT_MASK           0x01
#define CON8_DPM_STAT_SHIFT          3

#define CON8_PG_STAT_MASK           0x01
#define CON8_PG_STAT_SHIFT          2

#define CON8_THERM_STAT_MASK           0x01
#define CON8_THERM_STAT_SHIFT          1

#define CON8_VSYS_STAT_MASK           0x01
#define CON8_VSYS_STAT_SHIFT          0

//CON9
#define CON9_WATCHDOG_FAULT_MASK      0x01
#define CON9_WATCHDOG_FAULT_SHIFT     7

#define CON9_OTG_FAULT_MASK           0x01
#define CON9_OTG_FAULT_SHIFT          6

#define CON9_CHRG_FAULT_MASK           0x03
#define CON9_CHRG_FAULT_SHIFT          4

#define CON9_BAT_FAULT_MASK           0x01
#define CON9_BAT_FAULT_SHIFT          3

#define CON9_NTC_FAULT_MASK           0x07
#define CON9_NTC_FAULT_SHIFT          0

//CON10
#define CON10_PN_MASK      0x07
#define CON10_PN_SHIFT     3

#define CON10_Rev_MASK           0x07
#define CON10_Rev_SHIFT          0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------
extern void bq24196_set_en_hiz(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_vindpm(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_iinlim(struct i2c_client *client,kal_uint32 val);
//CON1----------------------------------------------------
extern void bq24196_set_reg_rst(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_wdt_rst(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_chg_config(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_sys_min(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_boost_lim(struct i2c_client *client,kal_uint32 val);
//CON2----------------------------------------------------
extern void bq24196_set_ichg(struct i2c_client *client,kal_uint32 val);
//CON3----------------------------------------------------
extern void bq24196_set_iprechg(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_iterm(struct i2c_client *client,kal_uint32 val);
//CON4----------------------------------------------------
extern void bq24196_set_vreg(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_batlowv(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_vrechg(struct i2c_client *client,kal_uint32 val);
//CON5----------------------------------------------------
extern void bq24196_set_en_term(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_term_stat(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_watchdog(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_en_timer(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_chg_timer(struct i2c_client *client,kal_uint32 val);
//CON6----------------------------------------------------
extern void bq24196_set_treg(struct i2c_client *client,kal_uint32 val);
//CON7----------------------------------------------------
extern void bq24196_set_tmr2x_en(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_batfet_disable(struct i2c_client *client,kal_uint32 val);
extern void bq24196_set_int_mask(struct i2c_client *client,kal_uint32 val);
//CON8----------------------------------------------------
extern kal_uint32 bq24196_get_system_status(struct i2c_client *client);
extern kal_uint32 bq24196_get_vbus_stat(struct i2c_client *client);
extern kal_uint32 bq24196_get_chrg_stat(struct i2c_client *client);
extern kal_uint32 bq24196_get_vsys_stat(struct i2c_client *client);
//---------------------------------------------------------
extern void bq24196_dump_register( struct i2c_client *client);
extern  int bq27541_get_battery_temperature(void);
extern  int bq27541_get_battery_mvolts(void);
extern  int bq27541_get_battery_capacity(void);
extern  int bq27541_is_battery_present(void);

#endif
