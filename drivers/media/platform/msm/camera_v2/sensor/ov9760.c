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

#define ov9760_SENSOR_NAME "ov9760"
DEFINE_MSM_MUTEX(ov9760_mut);

static struct msm_sensor_ctrl_t ov9760_s_ctrl;

static struct msm_sensor_power_setting ov9760_power_setting[] = {
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
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},

	/*{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},*/

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},

	{
                .seq_type = SENSOR_GPIO,
                .seq_val = SENSOR_GPIO_RESET,
                .config_val = GPIO_OUT_HIGH,
                .delay = 10,
        },

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
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

static struct v4l2_subdev_info ov9760_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov9760_i2c_id[] = {
	{ov9760_SENSOR_NAME, (kernel_ulong_t)&ov9760_s_ctrl},
	{ }
};
static int32_t msm_ov9760_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov9760_s_ctrl);
}


static struct i2c_driver ov9760_i2c_driver = {
	.id_table = ov9760_i2c_id,
	.probe  = msm_ov9760_i2c_probe,
	.driver = {
		.name = ov9760_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov9760_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


static const struct of_device_id ov9760_dt_match[] = {
	{.compatible = "qcom,ov9760", .data = &ov9760_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov9760_dt_match);

static struct platform_driver ov9760_platform_driver = {
	.driver = {
		.name = "qcom,ov9760",
		.owner = THIS_MODULE,
		.of_match_table = ov9760_dt_match,
	},
};

static int32_t ov9760_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_info("%s:%d\n", __func__, __LINE__);
	match = of_match_device(ov9760_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);

	printk("ov9760_platform_probe, rc=%d\n",
			rc);

	return rc;
}

static void ov9760_init_debugfs(void);
static void ov9760_clean_debugfs(void);

static int __init ov9760_init_module(void)
{
	int32_t rc = 0;

	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov9760_platform_driver,
		ov9760_platform_probe);
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	if (!rc){
		ov9760_init_debugfs();
		return rc;
	}

	return i2c_add_driver(&ov9760_i2c_driver);
}

static void __exit ov9760_exit_module(void)
{
	if (ov9760_s_ctrl.pdev) {

		ov9760_clean_debugfs();

		msm_sensor_free_sensor_data(&ov9760_s_ctrl);
		platform_driver_unregister(&ov9760_platform_driver);
	} else
		i2c_del_driver(&ov9760_i2c_driver);
	return;
}

//otp function
int32_t ov9760_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t address,uint16_t *data )
{
	int32_t rc=0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		address, data,MSM_CAMERA_I2C_BYTE_DATA);
	pr_debug("[ov9760_i2c]:%s:add=0x%x, data=0x%x\n", __func__, address,*data);
	if(rc<0)
		printk(KERN_EMERG"[ov9760_i2c]:read otp register addrees[0x%x] fail, rc=%d\n", address, rc);
	return rc;
}

int32_t ov9760_write_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t address,  uint16_t val)
{
	int ret=0;

	//printk("[ov9760_i2c]:%s:addr=0x%x,data=0x%x\n",__func__,address,val);

	ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, address, val, MSM_CAMERA_I2C_BYTE_DATA);
	return ret;
}

/*
uint16_t ov9760_read_i2c_seq(struct msm_sensor_ctrl_t *s_ctrl,uint32_t address, uint8_t *data, uint16_t data_len )
{
	int rc=0;
	int i;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
			s_ctrl->sensor_i2c_client,
			address, data, data_len);
	if(rc<0)
		printk("read otp register addrees[0x%x] fail, rc=%d\n", address, rc);

	for(i=0;i<data_len;i++){
		printk(KERN_EMERG"%s:addr=%x,data=%x\n",
			__func__,address+i,data[i]);
	}

	return rc;
}

int ov9760_write_i2c_seq(struct msm_sensor_ctrl_t *s_ctrl,int address, uint8_t *data, int data_len)
{
	int ret=0;
	{
	int i=0;
	for(i=0;i<data_len;i++)
	printk("%s:[0x%x,0x%x] \n", __func__,address+i,*(data+i) );
	}
	ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_seq(
			s_ctrl->sensor_i2c_client, address, data, data_len);
	return ret;
}*/

struct otp_struct{
int module_integrator_id;
int lens_id;
int year;
int month;
int date;
int rg_ratio;
int bg_ratio;
int light_rg;
int light_bg;
int user_datta[5];
int lenc[24];
int awb_checksum;
int awb_valid;
int lenc_checksum;
int lenc_valid;
};

static struct otp_struct current_otp;

// R/G and B/G of typical camera module is defined here
uint16_t RG_Ratio_Typical1 = 0x140;
uint16_t BG_Ratio_Typical1 = 0x12e;
int32_t ov9760_check_otp_wb(struct msm_sensor_ctrl_t *ctrl,uint16_t index)
{
	uint16_t temp, i;
	uint16_t address;
	uint16_t bank;

	bank = index | 0xc0;
	
	pr_debug("[ov9760_otp],ov9760_check_otp_wb,index=%d,bank=%x\n",
		index,bank);
	
	ov9760_write_i2c(ctrl,0x3d84, bank);
	// read otp into buffer
	ov9760_write_i2c(ctrl,0x3d81, 0x01);
	address = 0x3d00;
	mdelay(10);
	ov9760_read_i2c(ctrl,address,&temp);
	pr_debug("[ov9760_otp]: the 0x3d00=%d\n",temp);
	temp = temp & 0xc0;
	
	// clear otp buffer
	for (i=0;i<16;i++) {
		ov9760_write_i2c(ctrl,0x3d00 + i, 0x00);
	}
	if (temp == 0) {
		return 0; //empty
	}
	else if (temp & 0x80) {
		return 1; //invalid data
	}
	else {
		return 2; //valid data
	}
}


int32_t ov9760_check_otp_lenc(struct msm_sensor_ctrl_t *ctrl,uint16_t index)
{
	uint16_t temp, i;
	uint16_t address;
	uint16_t bank;

	bank = (index+1) * 2 + 0xc0;

	pr_debug("[ov9760_otp],ov9760_check_otp_lenc,index=%d,bank=%x\n",
                index,bank);


	ov9760_write_i2c(ctrl,0x3d84, bank);
	// read otp into buffer
	ov9760_write_i2c(ctrl,0x3d81, 0x01);
	address = 0x3d00;
	mdelay(10);
	ov9760_read_i2c(ctrl,address,&temp);
	pr_debug("[ov9760_otp]: the 0x3d00=%d\n",temp);
	temp = temp & 0xc0;

	// clear otp buffer
	for (i=0;i<16;i++) {
		ov9760_write_i2c(ctrl,0x3d00 + i, 0x00);
	}
	if (temp == 0) {
		return 0; //empty
	}
	else if (temp & 0x80) {
		return 1; //invalid
	}
	else {
		return 2; //valid
	}
}



// index: index of otp group. (0, 1, 2)
// return: 0,
int32_t ov9760_read_otp_wb(struct msm_sensor_ctrl_t *ctrl,
	uint16_t index, struct otp_struct *otp_ptr)
{
	uint16_t i, temp,temp1;
	uint16_t bank;
	uint16_t address;
	int check_sum = 0;

	pr_debug("[ov9760_otp]:%s\n,index=%d",__func__,index);

	bank = index | 0xc0;
	ov9760_write_i2c(ctrl,0x3d84, bank);
	// read otp into buffer
	ov9760_write_i2c(ctrl,0x3d81, 0x01);
	mdelay(10);
	address = 0x3d01;

	ov9760_read_i2c(ctrl, address, &temp);
	otp_ptr->module_integrator_id = temp;
	check_sum += temp;
	pr_debug("[ov9760_otp]:module_id=%x\n",otp_ptr->module_integrator_id);

	ov9760_read_i2c(ctrl, address+1, &temp);
	otp_ptr->lens_id = temp;
	check_sum += temp;
	pr_debug("[ov9760_otp]:lens_id=%x\n",otp_ptr->lens_id);

	ov9760_read_i2c(ctrl, address+2, &temp);
        otp_ptr->year = temp;
	check_sum += temp;
	pr_debug("[ov9760_otp]:year=%x\n",otp_ptr->year);

	ov9760_read_i2c(ctrl, address+3, &temp);
        otp_ptr->month = temp;
	check_sum += temp;
	pr_debug("[ov9760_otp]:month=%x\n",otp_ptr->month);
	

	ov9760_read_i2c(ctrl, address+4,&temp);
        otp_ptr->date = temp;
	check_sum += temp;
	pr_debug("[ov9760_otp]:date=%x\n",otp_ptr->date);
	
	ov9760_read_i2c(ctrl,address + 9,&temp);
	check_sum += temp;
	printk(KERN_EMERG"[ov9760_otp]:awb 0x3d0a=%x\n",temp);

  	ov9760_read_i2c(ctrl,address+5,&temp1);
	check_sum += temp1;
  	pr_debug("[ov9760_otp]:awb 0x3d06:%2x\n",temp1);

	
	otp_ptr->rg_ratio = temp1*4 +((temp&0xc0)>>6);

	ov9760_read_i2c(ctrl,address+6,&temp1);
	check_sum += temp1;
	pr_debug("[ov9760_otp]:awb 0x3d07:%2x\n",temp1);
	otp_ptr->bg_ratio = temp1*4 + ((temp & 0x30)>>4);

	ov9760_read_i2c(ctrl,address+7,&temp1);
	check_sum += temp1;
	pr_debug("[ov9760_otp]:awb 0x3d08:%2x\n",temp1);
	otp_ptr->light_rg = temp1*4+ ((temp & 0x0c)>>2);

	ov9760_read_i2c(ctrl,address+8,&temp1);
	check_sum += temp1;
	pr_debug("[ov9760_otp]:awb 0x3d09:%2x\n",temp1);
	//printk("awb 0x3d0a:%2x\n",temp);
	otp_ptr->light_bg = temp1*4 + (temp & 0x03);


	pr_debug("[ov9760_otp]: awb the rg_ratio=%4x bg_ratio=%4x light_rg=%4x light_bg=%4x \n",
	otp_ptr->rg_ratio,otp_ptr->bg_ratio,otp_ptr->light_rg,otp_ptr->light_bg);

	ov9760_read_i2c(ctrl,address+0xa,&temp);
	check_sum += temp;
	pr_debug("[ov9760_otp]:otp_ver=%d,cal temp=%d ir/bg=%d\n",
		(temp & 0xe0) >> 5,
		(temp & 0x18) >> 3,
		(temp & 0x07));

	ov9760_read_i2c(ctrl,address+0xb,&temp);
	otp_ptr->awb_checksum = temp;
	check_sum = (check_sum % 0xff) + 1;
	pr_debug("[ov9760_otp]: readout check_sum=0x%x, calculated checksum=0x%x\n",
		otp_ptr->awb_checksum, check_sum);

	if (check_sum == otp_ptr->awb_checksum){
		otp_ptr->awb_valid = 1;
	} else {
		otp_ptr->awb_valid = 0;
		printk("[ov9760_otp]: awb checksum doesn't match\n");
	}
	
	// clear otp buffer
	for (i=0;i<16;i++) {
		ov9760_write_i2c(ctrl,0x3d00 + i, 0x00);
	}
	return 0;
}


// index: index of otp group. (0, 1, 2)
// return: 0,
int32_t ov9760_read_otp_lenc(struct msm_sensor_ctrl_t *ctrl,
	uint16_t index, struct otp_struct * otp_ptr)
{
	uint16_t i, temp,temp1;
	uint16_t bank;
	uint16_t address;
	int checksum = 0;

	bank = (index+1) * 2 + 0xc0;
	pr_debug("[ov9760_otp],ov9760_read_otp_lenc,bank=%x,index=%d\n",bank,index);

	ov9760_write_i2c(ctrl,0x3d84, bank);
	// read otp into buffer
	ov9760_write_i2c(ctrl,0x3d81, 0x01);
	mdelay(10);

	address = 0x3d01;
	ov9760_read_i2c(ctrl,address + 2,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);

	(*otp_ptr).lenc[0] = (temp & 0x39)>>3; // Red_X0[10:8]
	
	ov9760_read_i2c(ctrl,address,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[1] = temp1; // Red_X0[7:0]
	
	(*otp_ptr).lenc[2] = temp & 0x07; // Red_Y0[10;8}

	ov9760_read_i2c(ctrl,address + 1,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[3] = temp; // Red_Y0[7:0]


	ov9760_read_i2c(ctrl,address + 3,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
        (*otp_ptr).lenc[4] = temp; // Red_A1

	ov9760_read_i2c(ctrl,address + 5,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[5]=(temp>>4)&0x0f;

	ov9760_read_i2c(ctrl,address + 4,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[6] = temp1; // Red_B1

	(*otp_ptr).lenc[7] = temp & 0x0f; // Red_B2

	ov9760_read_i2c(ctrl,address + 8,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[8] = (temp & 0x39)>>3; // Green_X0[10:8]

	ov9760_read_i2c(ctrl,address + 6,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[9] = temp1; // Green_X0[7:0]

	(*otp_ptr).lenc[10] = temp & 0x07; // Green_Y0[10;8}

	ov9760_read_i2c(ctrl,address + 7,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[11] = temp; // Green_Y0[7:0]

	ov9760_read_i2c(ctrl,address + 9,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[12] = temp; // Green_A1

	ov9760_read_i2c(ctrl,address + 11,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[13] = (temp>>4)&0x0f; // Green_A2

	ov9760_read_i2c(ctrl,address + 10,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[14] = temp1; // Green_B1
                                                                  
	(*otp_ptr).lenc[15] = temp & 0x0f; // Green_B2

	ov9760_read_i2c(ctrl,address + 14,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[16] = (temp & 0x39)>>3; // Blue_X0[10:8]

	ov9760_read_i2c(ctrl,address + 12,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[17] = temp1; // Blue_X0[7:0]

	(*otp_ptr).lenc[18] = temp & 0x07; // Blue_Y0[10;8}

	ov9760_read_i2c(ctrl,address + 13,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[19] = temp; // Blue_Y0[7:0]

	bank++;
	ov9760_write_i2c(ctrl,0x3d84,bank);
	ov9760_write_i2c(ctrl,0x3d81,0x01);
	address = 0x3d00;

	ov9760_read_i2c(ctrl,address,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[20] = temp;// Blue_A1

	ov9760_read_i2c(ctrl,address+2,&temp);
	checksum += temp;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[21] = (temp>>4) & 0x0f; // Blue_A2

	ov9760_read_i2c(ctrl,address+1,&temp1);
	checksum += temp1;
	pr_debug("[ov9760_otp]:checksum=0x%x(%d)\n",checksum,checksum);
	(*otp_ptr).lenc[22] = temp1;// Blue_B1
	(*otp_ptr).lenc[23] = temp & 0x0f; // Blue_B2 */

	//read checksum
	ov9760_read_i2c(ctrl,address+3,&temp);
	(*otp_ptr).lenc_checksum = temp;
	
	checksum = (checksum % 0xff) + 1;
	pr_debug("[ov9760_otp]:readout checksum=0x%x,calculated checksum=0x%x\n",
		(*otp_ptr).lenc_checksum, checksum);

	if ((*otp_ptr).lenc_checksum == checksum){
		(*otp_ptr).lenc_valid = 1;
	} else {
		(*otp_ptr).lenc_valid = 0;
		printk("[ov9760_otp]: Error!!lenc checksum doesn't match\n");
	}

	// clear otp buffer
	for (i=0;i<16;i++) {
		ov9760_write_i2c(ctrl,0x3d00 + i, 0x00);
	}

	for (i=0; i< 24; i++){
		pr_debug("[ov9760_otp]:lenc[%d]=%x\n",i,otp_ptr->lenc[i]);
	}	
	return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int32_t ov9760_update_awb_gain(struct msm_sensor_ctrl_t *ctrl,
	uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
	uint16_t temp,i;
	
	//Enable AWB manual control
	ov9760_read_i2c(ctrl,0x5186,&temp);
	temp |=0x1;
	ov9760_write_i2c(ctrl,0x5186,temp);

	//AWB gain enable, BLC enable
	ov9760_read_i2c(ctrl,0x5002,&temp);
	temp |=0x41;
	ov9760_write_i2c(ctrl,0x5002, temp);

	for(i=0;i<7;i++){
		ov9760_read_i2c(ctrl, 0x5180 + i, &temp);
		pr_debug("[ov9760_otp]:before update: awb gain Register:0x%x=0x%x\n",
			0x5180+i,temp);
	}

	if (R_gain>=0x400) {
		ov9760_write_i2c(ctrl,0x5180, R_gain>>8);
		ov9760_write_i2c(ctrl,0x5181, R_gain & 0x00ff);
	}

	if (G_gain>=0x400) {
		ov9760_write_i2c(ctrl,0x5182, G_gain>>8);
		ov9760_write_i2c(ctrl,0x5183, G_gain & 0x00ff);
	}

	if (B_gain>=0x400) {
		ov9760_write_i2c(ctrl,0x5184, B_gain>>8);
		ov9760_write_i2c(ctrl,0x5185, B_gain & 0x00ff);
	}

	for(i=0;i<7;i++){
		ov9760_read_i2c(ctrl,0x5180+i,&temp);
		pr_debug("[ov9760_otp]:after update:awb gain Register:0x%x=0x%x\n",
			0x5180+i,temp);
	}

	return 0;
}

// call this function after ov9760 initialization
// return value: 0 update success
// 1, no OTP
int32_t ov9760_update_otp_wb(struct msm_sensor_ctrl_t *ctrl)
{
	//struct otp_struct current_otp;
	uint16_t i;
	uint16_t otp_index;
	uint16_t temp;
	uint16_t R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;

	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	if (current_otp.awb_valid == 0){
		printk("[ov9760_otp]:no valid awb data yet, need to load\n");

		for(i=1;i<4;i++) {
			temp = ov9760_check_otp_wb(ctrl,i);
			if (temp == 2) {
				otp_index = i;
				printk("[ov9760_otp]:the valid otp index is %d\n",i);
				break;
			}
		}
		if (i==4) {
			// no valid wb OTP data
			printk("[ov9760_otp]: verify there is no valid wb otp data\n");
			return 1;
		}

		ov9760_read_otp_wb(ctrl,otp_index, &current_otp);
	} else{
		pr_debug("[ov9760_otp]:awb data already loaded, updated directly\n");
	}
	
	if (current_otp.awb_valid == 0){
		printk("[ov9760_otp]:Error!! no valid awb otp data\n");
		return 1;	
	}

	rg=current_otp.rg_ratio;
	bg=current_otp.bg_ratio;
	BG_Ratio_Typical1=current_otp.light_bg;
	RG_Ratio_Typical1=current_otp.light_rg;
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical1) {
		if (rg< RG_Ratio_Typical1) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical1 / bg;
			R_gain = 0x400 * RG_Ratio_Typical1 / rg;
		}
		else {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical1;
			B_gain = G_gain * BG_Ratio_Typical1 /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical1) {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical1;
			R_gain = G_gain * RG_Ratio_Typical1 / rg;
		}
		else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical1;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical1;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical1 /rg;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical1 / bg;
			}
		}
	}
	
	pr_debug("[ov9760_otp]: the R_gain=0x%4x,G_gain=0x%4x,B_gain=0x%4X",
		R_gain,G_gain,B_gain);
	ov9760_update_awb_gain(ctrl,R_gain, G_gain, B_gain);

	return 0;
}


int32_t ov9760_update_lenc(struct msm_sensor_ctrl_t *ctrl, struct otp_struct* otp_ptr)
{
	uint16_t i,temp; 

	pr_debug("[ov9760_otp]:ov9760_update_lenc\n");

	//LENC correction enable
	ov9760_read_i2c(ctrl,0x5000,&temp);
	temp = 0x80 | temp;
	ov9760_write_i2c(ctrl,0x5000,temp);

	for(i=0;i<24;i++)
	{
		pr_debug("[ov9760_otp]:register:0x%0x=0x%0x\n",i+0x5800,(*otp_ptr).lenc[i]);
		ov9760_write_i2c(ctrl,0x5800+i,(*otp_ptr).lenc[i]);
	}
	return 0;
}

int32_t ov9760_update_otp_lenc(struct msm_sensor_ctrl_t *ctrl)
{
	//struct otp_struct current_otp;
	uint16_t i, otp_index,temp;

	if (current_otp.lenc_valid == 0){
		pr_debug("[ov9760_otp]:no valid lenc data yet, need to load\n");

		for(i=1;i<=3;i++)
		{
			temp = ov9760_check_otp_lenc(ctrl,i);
			if(temp==2)
			{
				otp_index=i;
				break;
			}
		}

		if(i>3){
			printk("[ov9760_otp]:can't find lenc data\n"); 
			return 1;
		}

		ov9760_read_otp_lenc(ctrl,otp_index,&current_otp);
	} else {
		pr_debug("[ov9760_otp]:lenc data already loaded, update directly\n");
	}
	
	if (current_otp.lenc_valid == 0){
		printk("[ov9760_otp]:Error!!there's no valid lenc otp data\n");
		return 1;
	}

	ov9760_update_lenc(ctrl,&current_otp);

	return 0;
}


int32_t ov9760_update_blc_ratio(struct msm_sensor_ctrl_t *ctrl)
{
	uint16_t K,temp;
	uint16_t data;
	
	ov9760_write_i2c(ctrl,0x3d84,0xdf); //bank 31
	ov9760_write_i2c(ctrl,0x3d81,0x01);
	mdelay(5);
	
	ov9760_read_i2c(ctrl,0x3d0b,&data);
	K = data;
	pr_debug("[ov9760_otp]: ov9760_update_blc_ratio:K=%x\n",K); 

	if(K!=0)
	{
		if(K>=0x15 && K<=0x40){
		    	ov9760_read_i2c(ctrl,0x4000,&data);
			temp = data;
			pr_debug("[ov9760_otp]: ov9760_update_blc_ratio:temp=%x\n",temp);
			temp &=0x9f; //Auto load mode, 0x4000[5]=0,0x4000[6]=0
			ov9760_write_i2c(ctrl,0x4000,temp);
			return 2;
		}
	}

	ov9760_read_i2c(ctrl,0x3d0a,&K);
	pr_debug("[ov9760_otp]: ov9760_update_blc_ratio:K=%x\n",K);
	
	if(K >0x10 && K<=0x40){
		ov9760_write_i2c(ctrl,0x4006,K);
  		ov9760_read_i2c(ctrl,0x4000,&temp);
		pr_debug("[ov9760_otp]: ov9760_update_blc_ratio:temp=%x\n",temp);
		temp |=0x40;
		temp &=0xdf; //Manual load mode, 0x4000[5]=0,0x4000[6]=1
		ov9760_write_i2c(ctrl,0x4000,temp);
		return 1;
	}
	else {
		//set to default
		ov9760_write_i2c(ctrl,0x4006,0x20);
		ov9760_read_i2c(ctrl,0x4000,&temp);
		pr_debug("[ov9760_otp]: ov9760_update_blc_ratio:temp=%x\n",temp);
		temp |=0x40;
 		temp &=0xdf;
   		ov9760_write_i2c(ctrl,0x4000,temp);
		return 0;
	}
}

static int otp_disable = 0;

static int32_t ov9760_sensor_otp_proc(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i =0;	
	printk(KERN_EMERG"[ov9760_otp], %s\n",__func__);

	if (otp_disable){
                printk("[ov9760_otp],otp disabled for debug purpose\n");
                return 0;
        }
    for(i = 0;i < 3;i++)
    {
       ov9760_update_blc_ratio(s_ctrl);
	   ov9760_update_otp_wb(s_ctrl);
	   ov9760_update_otp_lenc(s_ctrl);
       if((current_otp.awb_valid == 1)&&(current_otp.lenc_valid == 1))
       	{
			 break; 	
		}
    }
	if(i == 3)
	{
	    printk(KERN_EMERG"ov9760 OTP checksum doesn't match !!!!\n");
		return -EFAULT; 
	}
	else
	{
	   printk("[ov9760_otp],ov9760 OTP checksum match\n");	
	   return 0;
	}
	
}

//////////////////// debug fs function /////////////////////

static  ssize_t exposure_line_debug_read(struct file *file,
	char __user *buf,size_t count, loff_t *pos)
{
	uint16_t val;
	const int size = 32;
	uint32_t line_cnt = 0;
	char buffer[size];
	int n = 0;
	
	ov9760_read_i2c(&ov9760_s_ctrl,0x3500,&val);
	line_cnt += val << 12;
	ov9760_read_i2c(&ov9760_s_ctrl,0x3501,&val);
	line_cnt += val << 4;
	ov9760_read_i2c(&ov9760_s_ctrl,0x3502,&val);
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

	val = line_cnt & 0x0f;
	val = val << 4;
	ov9760_write_i2c(&ov9760_s_ctrl,0X3502,val);

	val = line_cnt & 0xff0;
	val = val >> 4;
	ov9760_write_i2c(&ov9760_s_ctrl,0X3501,val);	

	val = (line_cnt & 0xf000) >> 12;
	ov9760_write_i2c(&ov9760_s_ctrl,0X3500,val);
	
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

        ov9760_read_i2c(&ov9760_s_ctrl,0x0340,&val);
        vts = val << 8;
        ov9760_read_i2c(&ov9760_s_ctrl,0x0341,&val);
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
        ov9760_write_i2c(&ov9760_s_ctrl,0X0341,val);
        
        val = vts >> 8;
        ov9760_write_i2c(&ov9760_s_ctrl,0X0340,val);

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

        ov9760_read_i2c(&ov9760_s_ctrl,0x350a,&val);
        gain = val << 8;
        ov9760_read_i2c(&ov9760_s_ctrl,0x350b,&val);
        gain  += val; 

        n = scnprintf(buffer,size,"gain_16 = %d\n",gain);
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
        ov9760_write_i2c(&ov9760_s_ctrl,0X350b,val);

        val = (gain & 0xff00) >> 8;
        ov9760_write_i2c(&ov9760_s_ctrl,0X350a,val);

        return cnt;
}

static const struct file_operations gain_debug_ops = {
        .read = gain_debug_read,
        .write = gain_debug_write,
};

extern int32_t lenovo_camera_sensor_i2c_debug;
extern int32_t lenovo_camera_ov9760_ae_disable;

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

        n = scnprintf(buffer,size,"ae disable  = %d\n", lenovo_camera_ov9760_ae_disable);
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

        lenovo_camera_ov9760_ae_disable = simple_strtol(lbuf,NULL,0);

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

	ov9760_read_i2c(&ov9760_s_ctrl, reg_offset, &reg_val);

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

	ov9760_write_i2c(&ov9760_s_ctrl,reg_offset,reg_val);

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

static struct dentry* ov9760_dir=NULL;
static struct dentry* exp_line_dir=NULL;
static struct dentry* debug_mode_dir=NULL;
static struct dentry* reg_offset_dir=NULL;
static struct dentry* reg_val_dir=NULL;
static struct dentry* gain_dir=NULL;
static struct dentry* ae_disable_dir=NULL;
static struct dentry* vts_dir=NULL;
static struct dentry* otp_dir=NULL;

static void ov9760_init_debugfs(void)
{
	ov9760_dir = debugfs_create_dir("ov9760",NULL);

	exp_line_dir = debugfs_create_file("exposure",S_IFREG | S_IRUGO| S_IWUSR,
		ov9760_dir,NULL,&exp_line_debug_ops);

	debug_mode_dir =debugfs_create_file("debug_mode",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&debug_mode_debug_ops);

	reg_offset_dir =debugfs_create_file("reg_addr",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&reg_offset_debug_ops);

	reg_val_dir =debugfs_create_file("reg_val",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&reg_val_debug_ops);

	gain_dir =debugfs_create_file("gain",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&gain_debug_ops);

	ae_disable_dir =debugfs_create_file("ae_disable",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&ae_disable_debug_ops);
	
	vts_dir =debugfs_create_file("vts",S_IFREG | S_IRUGO| S_IWUSR,
               	ov9760_dir,NULL,&vts_debug_ops);

	otp_dir =debugfs_create_file("otp_disable",S_IFREG | S_IRUGO| S_IWUSR,
                ov9760_dir,NULL,&otp_debug_ops);
}
	

static void ov9760_clean_debugfs(void)
{
	debugfs_remove(exp_line_dir);
	debugfs_remove(debug_mode_dir);
	debugfs_remove(reg_offset_dir);
	debugfs_remove(reg_val_dir);
	debugfs_remove(gain_dir);
	debugfs_remove(ae_disable_dir);
	debugfs_remove(vts_dir);
	debugfs_remove(otp_dir);
	debugfs_remove(ov9760_dir);
}
//////// end of debug fs

static struct msm_sensor_ctrl_t ov9760_s_ctrl = {
	.sensor_i2c_client = &ov9760_sensor_i2c_client,
	.power_setting_array.power_setting = ov9760_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov9760_power_setting),
	.msm_sensor_mutex = &ov9760_mut,
	.sensor_v4l2_subdev_info = ov9760_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov9760_subdev_info),
	.sensor_otp_proc = ov9760_sensor_otp_proc,
};

//module_init(ov9760_init_module);
late_initcall(ov9760_init_module);
module_exit(ov9760_exit_module);
MODULE_DESCRIPTION("ov9760");
MODULE_LICENSE("GPL v2");
