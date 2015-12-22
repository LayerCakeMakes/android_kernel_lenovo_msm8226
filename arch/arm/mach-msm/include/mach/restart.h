/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASM_ARCH_MSM_RESTART_H_
#define _ASM_ARCH_MSM_RESTART_H_

#define RESTART_NORMAL 0x0
#define RESTART_DLOAD  0x1

#if defined(CONFIG_MSM_NATIVE_RESTART)
void msm_set_restart_mode(int mode);
void msm_restart(char mode, const char *cmd);
#elif defined(CONFIG_ARCH_FSM9XXX)
void fsm_restart(char mode, const char *cmd);
#else
#define msm_set_restart_mode(mode)
#endif

extern int pmic_reset_irq;
#ifdef CONFIG_MSM_SECURE_MODE
void set_efuse_mode(void);
int get_efuse_state(void);

enum boot_efuse_status
{
    BOOT_EFUSE_NOBLOW,        //the phone don't enable secure boot
    BOOT_EFUSE_BLOWED,        //the phone secure boot enabled
    BOOT_EFUSE_NOT_MATCH_SIGN,
    BOOT_EFUSE_READYTOBLOW,  //set the flag,the phone will reboot and blow efuse
    BOOT_EFUSE_UNKNOW_ERR
};
#endif

#endif

