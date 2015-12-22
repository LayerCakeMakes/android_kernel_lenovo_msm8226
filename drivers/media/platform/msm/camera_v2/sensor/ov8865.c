/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include "msm_sensor.h"
#define OV8865_SENSOR_NAME "ov8865"
DEFINE_MSM_MUTEX(ov8865_mut);

struct otp_struct {
        uint16_t module_integrator_id;
        uint16_t lens_id;
        uint16_t production_year;
        uint16_t production_month;
        uint16_t production_day;
        uint16_t rg_ratio;
        uint16_t bg_ratio;
        uint16_t typical_rg;
        uint16_t typical_bg;
        uint16_t lenc[62];
	//below is added for lenovo
	uint16_t otp_version;
	uint16_t af_flag;
	uint16_t ir_flag;
	uint16_t vcm_id;
	uint16_t drv_ic_id;
	uint16_t ct_id;
	uint16_t rsv;
	uint16_t info_checksum;
	uint16_t awb_checksum;
	uint16_t lenc_checksum;
	uint16_t info_valid;
	uint16_t awb_valid;
	uint16_t lenc_valid;
};


static struct msm_sensor_ctrl_t ov8865_s_ctrl;

static struct msm_sensor_power_setting ov8865_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov8865_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8865_i2c_id[] = {
	{OV8865_SENSOR_NAME, (kernel_ulong_t)&ov8865_s_ctrl},
	{ }
};

static int32_t msm_ov8865_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8865_s_ctrl);
}

static struct i2c_driver ov8865_i2c_driver = {
	.id_table = ov8865_i2c_id,
	.probe  = msm_ov8865_i2c_probe,
	.driver = {
		.name = OV8865_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8865_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8865_dt_match[] = {
	{.compatible = "qcom,ov8865", .data = &ov8865_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8865_dt_match);

static struct platform_driver ov8865_platform_driver = {
	.driver = {
		.name = "qcom,ov8865",
		.owner = THIS_MODULE,
		.of_match_table = ov8865_dt_match,
	},
};

static int32_t ov8865_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	
	printk("enter %s\n",__func__);

	match = of_match_device(ov8865_dt_match, &pdev->dev);
	
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static void ov8865_init_debugfs(void);
static void ov8865_clean_debugfs(void);

static int __init ov8865_init_module(void)
{
	int32_t rc = 0;

	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8865_platform_driver,
		ov8865_platform_probe);
	if (!rc){
		ov8865_init_debugfs();
		return rc;
	}

	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8865_i2c_driver);
}

static void __exit ov8865_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8865_s_ctrl.pdev) {
		
		ov8865_clean_debugfs();
	
		msm_sensor_free_sensor_data(&ov8865_s_ctrl);
		platform_driver_unregister(&ov8865_platform_driver);
	} else
		i2c_del_driver(&ov8865_i2c_driver);
	return;
}


//otp function
int32_t ov8865_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t address,uint16_t *data )
{
        int32_t rc=0;

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                address, data,MSM_CAMERA_I2C_BYTE_DATA);

        printk("[ov8865_i2c]:%s:addr=%x,data=%x\n", __func__, address,*data);

        if(rc<0)
                printk(KERN_EMERG"[ov8865_i2c]:read otp register addrees[0x%x] fail, rc=%d\n", address, rc);
        return rc;
}

int32_t ov8865_write_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t address,  uint16_t val)
{
        int ret=0;

        //printk("[ov8865_i2c]:%s:addr=%x,data=%x\n",__func__,address,val);

        ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client, address, val, MSM_CAMERA_I2C_BYTE_DATA);
        return ret;
}


static uint16_t OV8865_read_cmos_sensor(uint16_t addr)
{
	uint16_t data;

	/* if (!ov8865_s_ctrl){
		printk(KERN_EMERG"s_ctrl not init in %s\n",__func__);
		return 0;
	}*/
	ov8865_read_i2c(&ov8865_s_ctrl,addr,&data);
	return data;
}


static int32_t OV8865_write_cmos_sensor(uint16_t addr, uint16_t data)
{
	int32_t ret;

	/*if (!ov8865_s_ctrl){
		printk(KERN_EMERG"s_ctrl not init in %s\n",__func__);
		return -1;
	}*/
	
	ret = ov8865_write_i2c(&ov8865_s_ctrl,addr,data);
	return ret;
}


//#define BG_Ratio_Typical_Value 0x11c
//#define RG_Ratio_Typical_Value 0x110

static uint32_t BG_Ratio_Typical;
static uint32_t RG_Ratio_Typical;


// index: index of otp group. (1, 2, 3)
// return:             0, group index is empty
//                     1, group index has invalid data
//                     2, group index has valid data
static uint16_t check_otp_info(uint16_t index)
{
        uint16_t flag;
        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, 0x70);
        OV8865_write_cmos_sensor(0x3d89, 0x10);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, 0x70);
        OV8865_write_cmos_sensor(0x3d8B, 0x10);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(5);
        flag = OV8865_read_cmos_sensor(0x7010);

	printk(KERN_EMERG"[ov8865_otp]:%s, index=%d,0x7010=0x%x\n",__func__,index,flag);

        //select group
        if (index == 1)
        {
                flag = (flag>>6) & 0x03;
        }
        else if (index == 2)
        {
                flag = (flag>>4) & 0x03;
        }
        else if (index ==3)
        {
                flag = (flag>>2) & 0x03;
        }
        // clear otp buffer
        OV8865_write_cmos_sensor(0x7010, 0x00);

	if (flag == 0x00) {
                return 0;
        }
        else if (flag & 0x02) {
                return 1;
        }
        else {
                return 2;
        }
}

// index: index of otp group. (1, 2, 3)
// return:             0, group index is empty
//                     1, group index has invalid data
//                     2, group index has valid data
static uint16_t check_otp_wb(uint16_t index)
{
        uint16_t flag;
        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, 0x70);
        OV8865_write_cmos_sensor(0x3d89, 0x20);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, 0x70);
        OV8865_write_cmos_sensor(0x3d8B, 0x20);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(5);
        //select group
        flag = OV8865_read_cmos_sensor(0x7020);

	printk(KERN_EMERG"[ov8865_otp]:%s, index=%d,0x7020=0x%x\n",__func__,index,flag);

        if (index == 1)
        {
                flag = (flag>>6) & 0x03;
        }
        else if (index == 2)
        {
                flag = (flag>>4) & 0x03;
        }
        else if (index == 3)
        {
                flag = (flag>>2) & 0x03;
        }
        // clear otp buffer
	OV8865_write_cmos_sensor( 0x7020, 0x00);
        if (flag == 0x00) {
                return 0;
        }
        else if (flag & 0x02) {
                return 1;
        }
        else {
                return 2;
        }
}


// index: index of otp group. (1, 2, 3)
// return:             0, group index is empty
//                     1, group index has invalid data
//                     2, group index has valid data
static uint16_t check_otp_lenc(uint16_t index)
{
        uint16_t flag;
        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, 0x70);
        OV8865_write_cmos_sensor(0x3d89, 0x3A);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, 0x70);
        OV8865_write_cmos_sensor(0x3d8B, 0x3A);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(5);
        flag = OV8865_read_cmos_sensor(0x703a);

	printk(KERN_EMERG"[ov8865_otp]:%s, index=%d,0x703a=0x%x\n",__func__,index,flag);

        if (index == 1)
        {
                flag = (flag>>6) & 0x03;
        }
        else if (index == 2)
        {
                flag = (flag>>4) & 0x03;
        }
        else if (index == 3)
        {
                flag = (flag>> 2)& 0x03;
        }
        // clear otp buffer
	OV8865_write_cmos_sensor( 0x703a, 0x00);
        if (flag == 0x00) {
                return 0;
        }
        else if (flag & 0x02) {
                return 1;
        }
        else {
                return 2;
        }
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static uint16_t read_otp_info(uint16_t index, struct otp_struct *otp_ptr)
{
        uint16_t i;
        uint16_t start_addr, end_addr;
	uint16_t check_sum = 0;

        if (index == 1) {
                start_addr = 0x7011;
                end_addr = 0x7015;
        }
        else if (index == 2) {
                start_addr = 0x7016;
                end_addr = 0x701a;
        }
        else if (index == 3) {
                start_addr = 0x701b;
                end_addr = 0x701f;
        }

	OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(5);

        (*otp_ptr).module_integrator_id = OV8865_read_cmos_sensor(start_addr);
	check_sum += (*otp_ptr).module_integrator_id;

        (*otp_ptr).lens_id = OV8865_read_cmos_sensor(start_addr + 1);
	 check_sum += (*otp_ptr).lens_id;

        (*otp_ptr).production_year = OV8865_read_cmos_sensor(start_addr + 2);
	 check_sum += (*otp_ptr).production_year;

        (*otp_ptr).production_month = OV8865_read_cmos_sensor(start_addr + 3);
	 check_sum += (*otp_ptr).production_month;

        (*otp_ptr).production_day = OV8865_read_cmos_sensor(start_addr + 4);
	 check_sum += (*otp_ptr).production_day;

        // clear otp buffer
        for (i=start_addr; i<=end_addr; i++) {
                OV8865_write_cmos_sensor(i, 0x00);
        }

	//read lenovo special info
	 if (index == 1) {
                start_addr = 0x70f5;
                end_addr = 0x70fc;
        }
        else if (index == 2) {
                start_addr = 0x70fd;
                end_addr = 0x7104;
        }
        else if (index == 3) {
                start_addr = 0x7105;
                end_addr = 0x710c;
        }

        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(5);

		
	(*otp_ptr).otp_version = OV8865_read_cmos_sensor(start_addr);
	 check_sum += (*otp_ptr).otp_version;

	(*otp_ptr).af_flag = OV8865_read_cmos_sensor(start_addr + 1);
	 check_sum += (*otp_ptr).af_flag;

	(*otp_ptr).ir_flag = OV8865_read_cmos_sensor(start_addr + 2);
	 check_sum += (*otp_ptr).ir_flag;

	(*otp_ptr).vcm_id = OV8865_read_cmos_sensor(start_addr + 3);
	 check_sum += (*otp_ptr).vcm_id;

	(*otp_ptr).drv_ic_id = OV8865_read_cmos_sensor(start_addr + 4);
	 check_sum += (*otp_ptr).drv_ic_id;

	(*otp_ptr).ct_id = OV8865_read_cmos_sensor(start_addr + 5);
	 check_sum += (*otp_ptr).ct_id;

	(*otp_ptr).rsv = OV8865_read_cmos_sensor(start_addr + 6);
	 check_sum += (*otp_ptr).rsv;

	(*otp_ptr).info_checksum = OV8865_read_cmos_sensor(start_addr + 7);

        // clear otp buffer
        for (i=start_addr; i<=end_addr; i++) {
                OV8865_write_cmos_sensor(i, 0x00);
        }

	printk(KERN_EMERG"[ov8865_otp]: module_integrator_id=0x%x\n",(*otp_ptr).module_integrator_id);
	printk(KERN_EMERG"[ov8865_otp]: lens_id=0x%x\n",(*otp_ptr).lens_id);
	printk(KERN_EMERG"[ov8865_otp]: production_year=0x%x\n",(*otp_ptr).production_year);
	printk(KERN_EMERG"[ov8865_otp]: production_month=0x%x\n",(*otp_ptr).production_month);
	printk(KERN_EMERG"[ov8865_otp]: production_day=0x%x\n",(*otp_ptr).production_day);
	printk(KERN_EMERG"[ov8865_otp]: otp_version=0x%x\n",(*otp_ptr).otp_version);
	printk(KERN_EMERG"[ov8865_otp]: af_flag=0x%x\n",(*otp_ptr).af_flag);
	printk(KERN_EMERG"[ov8865_otp]: ir_flag=0x%x\n",(*otp_ptr).ir_flag);
	printk(KERN_EMERG"[ov8865_otp]: vcm_id=0x%x\n",(*otp_ptr).vcm_id);
	printk(KERN_EMERG"[ov8865_otp]: drv_ic_id=0x%x\n",(*otp_ptr).drv_ic_id);
	printk(KERN_EMERG"[ov8865_otp]: ct_id=0x%x\n",(*otp_ptr).ct_id);
	printk(KERN_EMERG"[ov8865_otp]: rsv=0x%x\n",(*otp_ptr).rsv);	

	check_sum = (check_sum % 0XFF) + 1;
	printk(KERN_EMERG"[ov8865_otp]: %s: read out checksum=0x%x, calculated checksum=0x%x\n",
		__func__, (*otp_ptr).info_checksum,check_sum);
	
	if ( check_sum == (*otp_ptr).info_checksum){
		(*otp_ptr).info_valid = 1;
	} else {
		printk(KERN_EMERG"[ov8865_otp], otp info checksum doesn't match !!!!\n");
		(*otp_ptr).info_valid = 0;
	}	
	
        return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return:             0,
static uint16_t read_otp_wb(uint16_t index, struct otp_struct *otp_ptr)
{
        uint16_t i;
        uint16_t temp,temp1;
        uint16_t start_addr, end_addr;
	uint16_t check_sum = 0;

        if (index == 1) {
                start_addr = 0x7021;
                end_addr = 0x7025;
        }
        else if (index == 2) {
                start_addr = 0x7026;
                end_addr = 0x702a;
        }
        else if (index == 3) {
                start_addr = 0x702b;
                end_addr = 0x702f;
        }
	
	OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);

        mdelay(5);

        temp = OV8865_read_cmos_sensor(start_addr + 4);
	check_sum += temp;

	temp1 = OV8865_read_cmos_sensor(start_addr);
        (*otp_ptr).rg_ratio = (temp1 <<2) + ((temp>>6) & 0x03);
	check_sum += temp1;

	temp1 = OV8865_read_cmos_sensor(start_addr + 1);
        (*otp_ptr).bg_ratio = (temp1 <<2) + ((temp>>4) & 0x03);
	check_sum += temp1;

	temp1 = OV8865_read_cmos_sensor(start_addr + 2);
        (*otp_ptr).typical_rg = (temp1 <<2) + ((temp>>2) & 0x03);
	check_sum += temp1;

	temp1 = OV8865_read_cmos_sensor(start_addr + 3);
        (*otp_ptr).typical_bg = (temp1 <<2) + (temp & 0x03);
	check_sum += temp1;

        // clear otp buffer
        for (i=start_addr; i<=end_addr; i++) {
                OV8865_write_cmos_sensor(i, 0x00);
        }

  	printk(KERN_EMERG"[ov8865_otp]:(*otp_ptr).rg_ratio =0x%x, (*otp_ptr).bg_ratio=0x%x\n",
		(*otp_ptr).rg_ratio, (*otp_ptr).bg_ratio);
	printk(KERN_EMERG"[ov8865_otp]:(*otp_ptr).typical_rg =0x%x, (*otp_ptr).typical_bg=0x%x\n",(*otp_ptr).typical_rg, (*otp_ptr).typical_bg);


	//do check sum
	if (index == 1) {
                start_addr = 0x710D;
                end_addr = 0x710D;
        }
        else if (index == 2) {
                start_addr = 0x710E;
                end_addr = 0x710E;
        }
        else if (index == 3) {
                start_addr = 0x710F;
                end_addr = 0x710F;
        }

        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);

        mdelay(5);

	(*otp_ptr).awb_checksum  = OV8865_read_cmos_sensor(start_addr);

	check_sum = (check_sum % 0XFF)+1;
	printk(KERN_EMERG"[ov8865_otp]:%s:otp checksum=0x%x, calculated checksum=0x%x\n",
			__func__, (*otp_ptr).awb_checksum, check_sum);

	if (check_sum == (*otp_ptr).awb_checksum){
		(*otp_ptr).awb_valid = 1;
	} else {
		printk(KERN_EMERG"[ov8865_otp],awb checksum not match!!!!\n");
		(*otp_ptr).awb_valid = 0;
	}		
	
	//clear otp buffer
	OV8865_write_cmos_sensor(start_addr, 0x00);

        return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return:             0,
static uint16_t read_otp_lenc(uint16_t index, struct otp_struct *otp_ptr)
{
        uint16_t i;
        uint16_t start_addr, end_addr;
	uint16_t check_sum = 0;

        if (index == 1) {
                start_addr = 0x703b;
                end_addr = 0x7078;
        }
        else if (index == 2) {
                start_addr = 0x7079;
                end_addr = 0x70b6;
        }
        else if (index == 3) {
                start_addr = 0x70b7;
                end_addr = 0x70f4;
        }
        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(10);

	for(i=0; i<62; i++) {
                (* otp_ptr).lenc[i]=OV8865_read_cmos_sensor(start_addr + i);
		check_sum += (* otp_ptr).lenc[i];
                printk(KERN_EMERG"[ov8865_otp]:(* otp_ptr).lenc[%d] =0x%x checksum=0x%x\n",
			i,(* otp_ptr).lenc[i],check_sum);
        }
	
        // clear otp buffer
        for (i=start_addr; i<=end_addr; i++) {
                OV8865_write_cmos_sensor(i, 0x00);
        }

	//checksum read
	if (index == 1) {
                start_addr = 0x7113;
                end_addr = 0x7113;
        }
        else if (index == 2) {
                start_addr = 0x7114;
                end_addr = 0x7114;
        }
        else if (index == 3) {
                start_addr = 0x7115;
                end_addr = 0x7115;
        }
        OV8865_write_cmos_sensor(0x3d84, 0xC0);
        //partial mode OTP write start address
        OV8865_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d89, start_addr & 0xff);
        // partial mode OTP write end address
        OV8865_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
        OV8865_write_cmos_sensor(0x3d8B, end_addr & 0xff);
        // read otp into buffer
        OV8865_write_cmos_sensor(0x3d81, 0x01);
        mdelay(10);

	(* otp_ptr).lenc_checksum = OV8865_read_cmos_sensor(start_addr);

	//clear otp buffer
	OV8865_write_cmos_sensor(start_addr, 0x00);

	check_sum = (check_sum % 0XFF) + 1;
	printk(KERN_EMERG"[ov8865_otp], %s, otp checksum=0x%x, calculated checksum=0x%x\n",
		__func__, (* otp_ptr).lenc_checksum,check_sum);

	if (check_sum == (* otp_ptr).lenc_checksum){
		(* otp_ptr).lenc_valid = 1;
	} else {
		(* otp_ptr).lenc_valid = 0;
		printk(KERN_EMERG"[ov8865_otp]:lenc otp checksum doesn't match!!!!\n");
	}

        return 0;
}

// R_gain, sensor red gain of AWB, 0x1000 =1
// G_gain, sensor green gain of AWB, 0x1000 =1
// B_gain, sensor blue gain of AWB, 0x1000 =1
// return 0;
static uint16_t update_awb_gain(uint16_t R_gain, uint16_t G_gain, uint16_t B_gain)
{
        printk(KERN_EMERG"[ov8865_otp]:%s, R_gain =0x%x, G_gain=0x%x, B_gain=0x%x\n",
		__func__,R_gain, G_gain, B_gain);

        if (R_gain>0x1000) {
                OV8865_write_cmos_sensor(0x5018, R_gain>>8);
                OV8865_write_cmos_sensor(0x5019, R_gain & 0x00ff);
        }
        if (G_gain>0x1000) {
                OV8865_write_cmos_sensor(0x501A, G_gain>>8);
                OV8865_write_cmos_sensor(0x501B, G_gain & 0x00ff);
        }
        if (B_gain>0x1000) {
                OV8865_write_cmos_sensor(0x501C, B_gain>>8);
                OV8865_write_cmos_sensor(0x501D, B_gain & 0x00ff);
        }
        return 0;
}

// otp_ptr: pointer of otp_struct
static uint32_t update_lenc(struct otp_struct * otp_ptr)
{
        uint32_t i, temp;
        temp = OV8865_read_cmos_sensor(0x5000);
        temp = 0x80 | temp;
        OV8865_write_cmos_sensor(0x5000, temp);

        for(i=0;i<62;i++) {
                OV8865_write_cmos_sensor(0x5800 + i, (*otp_ptr).lenc[i]);
        }
        return 0;
}


static struct otp_struct current_otp;
// call this function after OV8865 initialization
// return value: 0 update success
//                     1, no OTP
static uint16_t update_otp_wb(void)
{
        //struct otp_struct current_otp;
        uint16_t i;
        uint16_t otp_index;
        uint16_t temp;
        uint16_t rg,bg;
        uint16_t R_gain, G_gain, B_gain;
        uint16_t nR_G_gain, nB_G_gain, nG_G_gain;
        uint16_t nBase_gain;

        // R/G and B/G of current camera module is read out from sensor OTP
        // check first OTP with valid data
        if ( current_otp.awb_valid == 0){ 
		printk("[ov8865_otp],%s, there no valid otp otp data yet, to load\n",
			__func__);

		for(i=1;i<=3;i++) {
        	        temp = check_otp_wb(i);
                	if (temp == 2) {
                        	otp_index = i;
                        	break;
               		 }
       		 }
       		 if (i>3) {
                	// no valid wb OTP data
			printk(KERN_EMERG"[ov8865_otp],%s ,no vaild wb otp data\n",__func__);
                	return 1;
        	}
		printk("[ov8865_otp],valid awb OTP data index = %d\n", otp_index);

	        read_otp_wb(otp_index, &current_otp);
	} else {
		printk("[ov8865_otp],awb data already loaded, updated it directly\n");
	}

	 if ( current_otp.awb_valid != 1){
		printk("[ov8865_otp],Error,there no valid awb otp data to update!!\n");
		return 1;
	}

	//YAO, fix me here, for lenovo, light source is typical rg/bg now 
#if 0
	if(current_otp.typical_rg==0) {
                // no light source information in OTP, light factor = 1
                //RG_Ratio_Typical =
                rg = current_otp.rg_ratio;
        }
        else {
                rg = current_otp.rg_ratio * (current_otp.typical_rg +512) / 1024;
        }
        if(current_otp.typical_bg==0) {
                BG_Ratio_Typical =
                // not light source information in OTP, light factor = 1
                bg = current_otp.bg_ratio;
        }
        else {
                bg = current_otp.bg_ratio * (current_otp.typical_bg +512) / 1024;
        }
#endif
	RG_Ratio_Typical = current_otp.typical_rg;
	BG_Ratio_Typical = current_otp.typical_bg;
	rg = current_otp.rg_ratio;
	bg = current_otp.bg_ratio;	

        printk(KERN_EMERG"[ov8865_otp]:rg =0x%x, bg=0x%x\n",rg, bg);
        printk(KERN_EMERG"[ov8865_otp]:RG_Ratio_Typical =0x%x, BG_Ratio_Typical=0x%x\n",RG_Ratio_Typical,
	BG_Ratio_Typical);
        //calculate G gain
        nR_G_gain = (RG_Ratio_Typical*0x1000) / rg;
        nB_G_gain = (BG_Ratio_Typical*0x1000) / bg;
        nG_G_gain = 0x1000;


	if (nR_G_gain < 0x1000 || nB_G_gain < 0x1000)
        {
                if (nR_G_gain < nB_G_gain)
                        nBase_gain = nR_G_gain;
                else
                        nBase_gain = nB_G_gain;
        }
        else
        {
                nBase_gain = nG_G_gain;
        }

        R_gain = 0x1000 * nR_G_gain / (nBase_gain);
        B_gain = 0x1000 * nB_G_gain / (nBase_gain);
        G_gain = 0x1000 * nG_G_gain / (nBase_gain);
        update_awb_gain(R_gain, G_gain, B_gain);
        return 0;
}

// call this function after OV8865 initialization
// return value: 0 update success
//                     1, no OTP
static uint16_t update_otp_lenc(void)
{
        //struct otp_struct current_otp;
        uint16_t i;
        uint16_t otp_index;
        uint16_t temp;
        // check first lens correction OTP with valid data
	if ( current_otp.lenc_valid == 0){ //lenc data not loaded, need read
		printk("[ov8865_otp], there's no valid otp lenc data yet, need to load\n");
        	for(i=1;i<=3;i++) {
                	temp = check_otp_lenc(i);
                	if (temp == 2) {
                        	otp_index = i;
                        	break;
               		 }
        	}
        	if (i>3) {

                	printk("[ov8865_otp]:[OV8865OTP]No lenc OTP Data\n");
               	 	// no valid WB OTP data
                	return 1;
       		 }
		printk("[ov8865_otp],%s, otp_index=%d\n", __func__,otp_index);

        	read_otp_lenc(otp_index, &current_otp);
	} else {
		printk("[ov8865_otp],lenc data already loaded, update directly\n");
	}
	
	if ( current_otp.lenc_valid == 1)
        	update_lenc(&current_otp);
	else {
		printk("[ov8865_otp],Error, there's no valid otp data to update lenc!!\n");
		return 1;
	}
		
        // success
        return 0;
}

static uint16_t update_otp_info(void)
{
        //struct otp_struct current_otp;
        uint16_t i;
        uint16_t otp_index;
        uint16_t temp;
        // check first lens correction OTP with valid data
	if ( current_otp.info_valid == 0 ){ //otp info ls not loaded
        	for(i=1;i<=3;i++) {
                	temp = check_otp_info(i);
                	if (temp == 2) {
                        	otp_index = i;
                        	break;
                	}
        	}
        	if (i>3) {

                	printk("[ov8865_otp]:No OTP info Data\n");
                	// no valid WB OTP data
                	return 1;
        	}
        	printk("[ov8865_otp],%s, otp_index=%d\n", __func__,otp_index);

        	read_otp_info(otp_index, &current_otp);
	}
        // success
        return 0;
}

static int otp_disable = 0;

static int32_t ov8865_sensor_otp_proc(struct msm_sensor_ctrl_t *s_ctrl)
{
    int i =0;
		printk(KERN_EMERG"[ov8865_otp], %s\n",__func__);

	if (otp_disable){
	
		printk("[ov8865_otp],otp disabled for debug purpose\n");
		return 0;
	}
    for(i = 0;i < 3;i++)
    {
	    update_otp_info();	
	    update_otp_wb();
	    update_otp_lenc();
		
		if((current_otp.info_valid == 1)&&(current_otp.awb_valid == 1)&&(current_otp.lenc_valid == 1))
		{
			break;  
		}
	   
     }
    if(i == 3)
    {
        printk(KERN_EMERG"ov8865 OTP checksum doesn't match !!!!\n");
	     return  -EFAULT; 
    }
	else
	{ 
	   printk("[ov8865_otp],ov8865 OTP checksum match\n");
	   return 0;
	}
	
}

///////////////////debug fs function ///////////////

static  ssize_t exposure_line_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        uint16_t val;
        const int size = 32;
        uint32_t line_cnt = 0;
        char buffer[size];
        int n = 0;

        ov8865_read_i2c(&ov8865_s_ctrl,0x3500,&val);
        line_cnt += val << 12;
        ov8865_read_i2c(&ov8865_s_ctrl,0x3501,&val);
        line_cnt += val << 4;
        ov8865_read_i2c(&ov8865_s_ctrl,0x3502,&val);
        line_cnt += val >> 4;

        n = scnprintf(buffer,size,"exposure line = %d\n",line_cnt);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}

static ssize_t exposure_line_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 32;
        char lbuf[size];
        int line_cnt;
        uint16_t val;
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        line_cnt = simple_strtol(lbuf,NULL,0);

        printk(KERN_EMERG"[ov8865_debug],%s, read -%s, line_cnt=%d\n",
                __func__,lbuf,line_cnt);

        val = line_cnt & 0x0f;
        val = val << 4;
        ov8865_write_i2c(&ov8865_s_ctrl,0X3502,val);

        val = line_cnt & 0xff0;
        val = val >> 4;
        ov8865_write_i2c(&ov8865_s_ctrl,0X3501,val);

        val = (line_cnt & 0xf000) >> 12;
        ov8865_write_i2c(&ov8865_s_ctrl,0X3500,val);

        return cnt;
}

static const struct file_operations exp_line_debug_ops = {
        .read = exposure_line_debug_read,
        .write = exposure_line_debug_write,
};

static  ssize_t vts_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        uint16_t val;
        const int size = 32;
        uint32_t vts  = 0;
        char buffer[size];
        int n = 0;

        ov8865_read_i2c(&ov8865_s_ctrl,0x380e,&val);
        vts = val << 8;
        ov8865_read_i2c(&ov8865_s_ctrl,0x380f,&val);
        vts  += val ;

        n = scnprintf(buffer,size,"vts  = %d\n", vts);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}


static ssize_t vts_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 32;
        char lbuf[size];
        int vts;
        uint16_t val;
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

      	vts  = simple_strtol(lbuf,NULL,0);

        val = vts & 0xff;
        ov8865_write_i2c(&ov8865_s_ctrl,0X380f,val);

        val = vts >> 8;
        ov8865_write_i2c(&ov8865_s_ctrl,0X380e,val);

        return cnt;
}

static const struct file_operations vts_debug_ops = {
        .read = vts_debug_read,
        .write = vts_debug_write,
};

static  ssize_t gain_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        uint16_t val;
        const int size = 32;
        uint32_t gain = 0;
        char buffer[size];
        int n = 0;

        ov8865_read_i2c(&ov8865_s_ctrl,0x3508,&val);
        gain = val << 8;
        ov8865_read_i2c(&ov8865_s_ctrl,0x3509,&val);
        gain  += val;

        n = scnprintf(buffer,size,"gain_128 = %d\n",gain);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}


static ssize_t gain_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 32;
        char lbuf[size];
        int gain;
        uint16_t val;
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        gain = simple_strtol(lbuf,NULL,0);

        val = gain & 0xff;
        ov8865_write_i2c(&ov8865_s_ctrl,0X3509,val);

        val = (gain & 0xff00) >> 8;
        ov8865_write_i2c(&ov8865_s_ctrl,0X3508,val);

        return cnt;
}

static const struct file_operations gain_debug_ops = {
        .read = gain_debug_read,
        .write = gain_debug_write,
};


extern int32_t lenovo_camera_sensor_i2c_debug;
extern int32_t lenovo_camera_ov8865_ae_disable;

static  ssize_t debug_mode_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        const int size = 32;
        char buffer[size];
        int n = 0;

        n = scnprintf(buffer,size,"i2c debug mode = %d\n",lenovo_camera_sensor_i2c_debug);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}

static ssize_t debug_mode_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 32;
        char lbuf[size];
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        lenovo_camera_sensor_i2c_debug = simple_strtol(lbuf,NULL,0);

        return cnt;
}


static  ssize_t ae_disable_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        const int size = 32;
        char buffer[size];
        int n = 0;

        n = scnprintf(buffer,size,"ae disable  = %d\n", lenovo_camera_ov8865_ae_disable);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}

static ssize_t ae_disable_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 32;
        char lbuf[size];
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        lenovo_camera_ov8865_ae_disable = simple_strtol(lbuf,NULL,0);

        return cnt;
}



static const struct file_operations debug_mode_debug_ops = {
        .read = debug_mode_debug_read,
        .write = debug_mode_debug_write,
};

static const struct file_operations ae_disable_debug_ops = {
        .read = ae_disable_debug_read,
        .write = ae_disable_debug_write,
};


static uint16_t reg_offset = 0x3500;
static uint16_t reg_val;

static  ssize_t reg_offset_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        const int size = 16;
        char buffer[size];
        int n = 0;

        n = scnprintf(buffer,size,"0x%x\n",reg_offset);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}


static ssize_t reg_offset_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 16;
        char lbuf[size];
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        reg_offset  = simple_strtol(lbuf,NULL,0);

        return cnt;
}

static  ssize_t reg_val_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        const int size = 32;
        char buffer[size];
        int n = 0;

        ov8865_read_i2c(&ov8865_s_ctrl, reg_offset, &reg_val);

        n = scnprintf(buffer,size,"[0x%x]=0x%x\n",reg_offset,reg_val);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}


static ssize_t reg_val_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 16;
        char lbuf[size];
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        reg_val  = simple_strtol(lbuf,NULL,0);

        ov8865_write_i2c(&ov8865_s_ctrl,reg_offset,reg_val);

        return cnt;
}


static const struct file_operations reg_offset_debug_ops = {
        .read = reg_offset_debug_read,
        .write = reg_offset_debug_write,
};

static const struct file_operations reg_val_debug_ops = {
        .read = reg_val_debug_read,
        .write = reg_val_debug_write,
};


static  ssize_t otp_debug_read(struct file *file,
        char __user *buf,size_t count, loff_t *pos)
{
        const int size = 16;
        char buffer[size];
        int n = 0;

        n = scnprintf(buffer,size,"otp_disable=%d\n",otp_disable);
        buffer[n] = 0;

        return simple_read_from_buffer(buf,count,pos,buffer,n);
}


static ssize_t otp_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
        const int size = 16;
        char lbuf[size];
        int rc;

        if (cnt > size-1)
                return -EINVAL;

        rc = copy_from_user(lbuf,ubuf,cnt);

        otp_disable  = simple_strtol(lbuf,NULL,0);

        return cnt;
}

static const struct file_operations otp_debug_ops = {
        .read = otp_debug_read,
        .write = otp_debug_write,
};

static struct dentry* ov8865_dir=NULL;
static struct dentry* exp_line_dir=NULL;
static struct dentry* debug_mode_dir=NULL;
static struct dentry* reg_offset_dir=NULL;
static struct dentry* reg_val_dir=NULL;
static struct dentry* gain_dir=NULL;
static struct dentry* ae_disable_dir=NULL;
static struct dentry* vts_dir=NULL;
static struct dentry* otp_dir=NULL;

static void ov8865_init_debugfs(void)
{
        ov8865_dir = debugfs_create_dir("ov8865",NULL);

        exp_line_dir = debugfs_create_file("exposure",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&exp_line_debug_ops);

        debug_mode_dir =debugfs_create_file("debug_mode",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&debug_mode_debug_ops);

        reg_offset_dir =debugfs_create_file("reg_addr",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&reg_offset_debug_ops);

        reg_val_dir =debugfs_create_file("reg_val",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&reg_val_debug_ops);

        gain_dir =debugfs_create_file("gain",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&gain_debug_ops);

        ae_disable_dir =debugfs_create_file("ae_disable",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&ae_disable_debug_ops);

	vts_dir =debugfs_create_file("vts",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&vts_debug_ops);

	otp_dir = debugfs_create_file("otp_disable",S_IFREG | S_IRUGO| S_IWUSR,
                ov8865_dir,NULL,&otp_debug_ops);
}
	


static void ov8865_clean_debugfs(void)
{
        debugfs_remove(exp_line_dir);
        debugfs_remove(debug_mode_dir);
        debugfs_remove(reg_offset_dir);
        debugfs_remove(reg_val_dir);
        debugfs_remove(gain_dir);
        debugfs_remove(ae_disable_dir);
	debugfs_remove(vts_dir);
	debugfs_remove(otp_dir);
        debugfs_remove(ov8865_dir);
}

//end of debug fs

static struct msm_sensor_ctrl_t ov8865_s_ctrl = {
	.sensor_i2c_client = &ov8865_sensor_i2c_client,
	.power_setting_array.power_setting = ov8865_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8865_power_setting),
	.msm_sensor_mutex = &ov8865_mut,
	.sensor_v4l2_subdev_info = ov8865_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8865_subdev_info),
	//lenovo added
	.sensor_otp_proc = ov8865_sensor_otp_proc,
	//lenovo end
};

module_init(ov8865_init_module);
module_exit(ov8865_exit_module);
MODULE_DESCRIPTION("ov8865");
MODULE_LICENSE("GPL v2");
