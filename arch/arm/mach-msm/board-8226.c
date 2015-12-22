/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/memory.h>
#include <linux/regulator/qpnp-regulator.h>
#include <linux/msm_tsens.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/restart.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/socinfo.h>
#include <mach/board.h>
#include <mach/clk-provider.h>
#include <mach/msm_smd.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/msm_smem.h>
#include <linux/msm_thermal.h>
#include "board-dt.h"
#include "clock.h"
#include "platsmp.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#ifdef CONFIG_BATTERY_BQ24196
#include <linux/i2c/bq24196.h>
#endif
#ifdef CONFIG_BATTERY_MSM8X60
#include <linux/msm-charger.h>
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/persistent_ram.h>
#endif

#include "lephone_nv.h"

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
#include <linux/input/synaptics_dsx.h>
#define TM_SAMPLE1      (1)     // 2D only
#define TM_SAMPLE2      (2)     // 2D + 0D x 2
#define TM_SAMPLE3      (3)     // 2D + 0D x 4
#define SYNAPTICS_MODULE TM_SAMPLE1
#endif

static struct memtype_reserve msm8226_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm8226_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct of_dev_auxdata msm8226_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),

	{}
};

#ifdef CONFIG_BATTERY_BQ24196
static struct bq24196_platform_data bq24196_platformdata __initdata = {
    .valid_n_gpio = 0,
    .batt_mah_rating = 0,
    .ce_gpio = 0,
};

static struct i2c_board_info i2c_bq24196[] __initdata = {
        {
                I2C_BOARD_INFO("bq24196", 0x6b),
                .platform_data = &bq24196_platformdata,
        },
};

static struct i2c_board_info i2c_bq27541[] __initdata = {
        {
                I2C_BOARD_INFO("bq27541", 0x55),
        },
};
#endif
#ifdef CONFIG_BATTERY_MSM8X60
static struct msm_charger_platform_data msm_charger_data = {
        .safety_time = 1440,//24h
        .update_time = 1,
        .max_voltage = 4350,
        .min_voltage = 3400,
};

static struct platform_device msm_charger_device = {
        .name = "msm-charger",
        .id = -1,
        .dev = {
                .platform_data = &msm_charger_data,
        }
};
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_START 0X78800000
#define RAM_CONSOLE_SIZE SZ_1M

static struct persistent_ram_descriptor msm_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = RAM_CONSOLE_SIZE,
	},
};

static struct persistent_ram msm_pr __initdata = {
	.descs = msm_prd,
	.num_descs = ARRAY_SIZE(msm_prd),
	.start = RAM_CONSOLE_START,
	.size = RAM_CONSOLE_SIZE,
};
static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	//.num_resources = ARRAY_SIZE(ram_console_resource),
	//.resource = ram_console_resource,
};
#endif

/* Synaptics changes for msm8226 Board */
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C

#if (SYNAPTICS_MODULE == TM_SAMPLE1)
#define TM_SAMPLE1_ADDR 0x38
#define TM_SAMPLE1_ATTN 17
#define TM_RESET_PIN 16

static unsigned char TM_SAMPLE1_f1a_button_codes[] = {};

static struct synaptics_dsx_cap_button_map TM_SAMPLE1_cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_SAMPLE1_f1a_button_codes),
	.map = TM_SAMPLE1_f1a_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.irq_gpio = TM_SAMPLE1_ATTN,
	.reset_gpio = TM_RESET_PIN,
	.cap_button_map = &TM_SAMPLE1_cap_button_map,
};

static struct i2c_board_info s7300_i2c_devices[] = {
 	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c", TM_SAMPLE1_ADDR),
 		.platform_data = &dsx_platformdata,
     	},	
};

#elif (SYNAPTICS_MODULE == TM_SAMPLE2)
#define TM_SAMPLE2_ADDR 0x38
#define TM_SAMPLE2_ATTN 17
#define TM_RESET_PIN 16

static unsigned char TM_SAMPLE2_f1a_button_codes[] = {KEY_MENU, KEY_BACK};

static struct synaptics_dsx_cap_button_map TM_SAMPLE2_cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_SAMPLE2_f1a_button_codes),
	.map = TM_SAMPLE2_f1a_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags = IRQF_TRIGGER_FALLING,
	.irq_gpio = TM_SAMPLE2_ATTN,
	.reset_gpio = TM_RESET_PIN,
 	.cap_button_map = &TM_SAMPLE2_cap_button_map,
};

static struct i2c_board_info s7300_i2c_devices[] = {
 	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c", TM_SAMPLE2_ADDR),
 		.platform_data = &dsx_platformdata,
     	},	
};

#elif (SYNAPTICS_MODULE == TM_SAMPLE3)
#define TM_SAMPLE3_ADDR	0x20
#define TM_SAMPLE3_ATTN	17
#define TM_RESET_PIN 16

static unsigned char TM_SAMPLE3_f1a_button_codes[] = {KEY_MENU, KEY_HOME,KEY_BACK,KEY_SEARCH};

static struct synaptics_dsx_cap_button_map TM_SAMPLE3_cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_SAMPLE3_f1a_button_codes),
	.map = TM_SAMPLE3_f1a_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags = IRQF_TRIGGER_FALLING,
	.irq_gpio = TM_SAMPLE3_ATTN,
	.reset_gpio = TM_RESET_PIN,
	.cap_button_map = &TM_SAMPLE3_cap_button_map,
};

static struct i2c_board_info s7300_i2c_devices[] = {
 	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c", TM_SAMPLE3_ADDR),
 		.platform_data = &dsx_platformdata,
     	},	
};
#endif
#endif

static struct reserve_info msm8226_reserve_info __initdata = {
	.memtype_reserve_table = msm8226_reserve_table,
	.paddr_to_memtype = msm8226_paddr_to_memtype,
};

static void __init msm8226_early_memory(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, msm8226_reserve_table);
}

static void __init msm8226_reserve(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8226_reserve_table);
	msm_reserve();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	persistent_ram_early_init(&msm_pr);
#endif
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8226_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	qpnp_regulator_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8226_rumi_clock_init_data);
	else
		msm_clock_init(&msm8226_clock_init_data);
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

static struct i2c_board_info __initdata tf9887L_info = {
	I2C_BOARD_INFO("tfa9887L", 0x34),
};

static struct i2c_board_info __initdata tf9887R_info = {
	I2C_BOARD_INFO("tfa9887R", 0x35),
};


void __init msm8226_init(void)
{
	struct of_dev_auxdata *adata = msm8226_auxdata_lookup;

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8226_init_gpiomux();
	board_dt_populate(adata);
	msm8226_add_drivers();
#ifdef CONFIG_BATTERY_BQ24196
        i2c_register_board_info(4,i2c_bq24196,1);
        i2c_register_board_info(4,i2c_bq27541,1);
#endif
#ifdef CONFIG_BATTERY_MSM8X60
       platform_device_register(&msm_charger_device);
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
       platform_device_register(&ram_console_device);
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
	i2c_register_board_info(5,s7300_i2c_devices,1);
#endif
	/* we enable both channels */
	i2c_register_board_info(5, &tf9887L_info, 1);
	i2c_register_board_info(5, &tf9887R_info, 1);

    lephone_nv_init();
}

static const char *msm8226_dt_match[] __initconst = {
	"qcom,msm8226",
	"qcom,msm8926",
	"qcom,apq8026",
	NULL
};

DT_MACHINE_START(MSM8226_DT, "Qualcomm MSM 8226 (Flattened Device Tree)")
	.map_io = msm_map_msm8226_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8226_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8226_dt_match,
	.reserve = msm8226_reserve,
	.init_very_early = msm8226_early_memory,
	.restart = msm_restart,
	.smp = &arm_smp_ops,
MACHINE_END
