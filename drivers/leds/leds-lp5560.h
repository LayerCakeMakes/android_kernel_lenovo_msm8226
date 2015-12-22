
#ifndef _INC_LEDS_LP5560_H_
#define _INC_LEDS_LP5560_H_

#include <linux/types.h>
/*
 *     TCON            |         | TCAL        TTOFF               |
 *     +--+  +--+      |         | +--+  +----+    +------------+  +--+  +--+  +--+
 *     |  |  |  |      |         | |  |  |    |    |            |  |  |  |  |  |  |
 *     |  |  |  |      |         | | C|  | I  |    |            |  |  |  |  |  |  |
 *  ---+  +--+  +------+---------+-+  +--+    +----+            +--+  +--+  +--+  +------
 *     |  TCOFF        |         |         TTON                    |
 *     |    TENTER     |  TBLANK |                                 |
 *
 * TENTER + TBLANK < 1500us                                        
 * TCAL 350 ~ 8000us
 * ndelay unit (us)
 */

#define TC_ON   20
#define TC_OFF  40
#define TT_ON   500
#define TT_OFF  500
#define TCAL    1000
#define TENTER  1000
#define TBLANK  200
#define TADJ    500

#define MIN_PULSE_TIME     200

#define CMD_RUN     1
#define CMD_TSTART   2
#define CMD_TEND    3
#define CMD_ONCE    4


#define LP5560_ATTR_RO(_name) \
        DEVICE_ATTR(_name, S_IRUGO, show_##_name, NULL)
#define LP5560_ATTR_RW(_name) \
        DEVICE_ATTR(_name, S_IRUGO | S_IWUSR , show_##_name, store_##_name)


typedef enum LED_TAG
{
    LED_TON = 0,
    LED_TOFF,
    LED_FLASH,
    LED_FLASH_TEST,
    LED_ONCE,
    LED_CUR,
}LED;

typedef enum CURRENT_TAG
{
    CUR_028 = 0,
    CUR_053 = 1000,
    CUR_078 = 2000,
    CUR_102 = 3000,
    CUR_126 = 4000,
    CUR_150 = 5000,
    CUR_173 = 6000,
    CUR_195 = 7000,
}CURRENT;

typedef enum RISE_FALL_TAG
{
    RAMP_000 = 0,
    RAMP_105 = 1000,
    RAMP_211 = 2000,
    RAMP_216 = 3000,
    RAMP_422 = 4000,
    RAMP_528 = 5000,
    RAMP_633 = 6000,
    RAMP_739 = 7000,
    RAMP_844 = 8000,
    RAMP_950 = 9000,
    RAMP_1056 = 10000,
    RAMP_1161 = 11000,
    RAMP_1267 = 12000,
    RAMP_1372 = 13000,
    RAMP_1478 = 14000,
    RAMP_1584 = 15000,
}RISE_FALL;

typedef enum ON_TIME_TAG
{
    ON_13 = 0,
    ON_26 = 1000,
    ON_52 = 2000,
    ON_105 = 3000,
    ON_158 = 4000,
    ON_211 = 5000,
    ON_264 = 6000,
    ON_316 = 7000,
    ON_369 = 8000,
    ON_435 = 9000,
    ON_501 = 10000,
    ON_594 = 11000,
    ON_699 = 12000,
    ON_805 = 13000,
    ON_910 = 14000,
    ON_1016 = 15000,
    ON_1122 = 16000,
    ON_1227 = 17000,
    ON_1353 = 18000,
    ON_1478 = 19000,
    ON_1603 = 20000,
    ON_1729 = 21000,
    ON_1854 = 22000,
    ON_1980 = 23000,
    ON_2105 = 24000,
    ON_2230 = 25000,
    ON_2356 = 26000,
    ON_2481 = 27000,
    ON_2613 = 28000,
    ON_2745 = 29000,
    ON_2877 = 30000,
    ON_3009 = 31000,
}ON_TIME;

typedef enum OFF_TIME_TAG
{
    OFF_26 = 0,
    OFF_52 = 1000,
    OFF_105 = 2000,
    OFF_211 = 3000,
    OFF_316 = 4000,
    OFF_422 = 5000,
    OFF_528 = 6000,
    OFF_633 = 7000,
    OFF_739 = 8000,
    OFF_871 = 9000,
    OFF_1003 = 10000,
    OFF_1188 = 11000,
    OFF_1399 = 12000,
    OFF_1610 = 13000,
    OFF_1821 = 14000,
    OFF_2032 = 15000,
    OFF_2244 = 16000,
    OFF_2455 = 17000,
    OFF_2706 = 18000,
    OFF_2956 = 19000,
    OFF_3207 = 20000,
    OFF_3458 = 21000,
    OFF_3709 = 22000,
    OFF_3960 = 23000,
    OFF_4210 = 24000,
    OFF_4461 = 25000,
    OFF_4712 = 26000,
    OFF_4963 = 27000,
    OFF_5227 = 28000,
    OFF_5491 = 29000,
    OFF_5755 = 30000,
    OFF_6019 = 31000,
}OFF_TIME;

typedef struct sequence {
    int rise1;
    int rise2;
    int rise3;
    int on1;
    int on2;
    int on3;
    int fall1;
    int fall2;
    int fall3;
    int off1;
    int off2;
    int off3;
    u32 cur;
}sequence, *psequence;

struct seq_config {
    u32 rise;
    u32 on;
    u32 fall;
    u32 off;
    u32 cur;
    u32 use_def;
};

#endif //_INC_LEDS_LP5560_H_
