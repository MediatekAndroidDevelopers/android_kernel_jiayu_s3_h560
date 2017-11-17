/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 GC2235mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc2235mipi_Sensor.h"

#define PFX "GC2235_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = GC2235_SENSOR_ID,
	
	.checksum_value = 0xf7375923,
	
	.pre = {
		.pclk = 30000000,				//record different mode's pclk
		.linelength = 1107,				//record different mode's linelength
		.framelength = 1387,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,		//record different mode's width of grabwindow
		.grabwindow_height = 1200,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 200,	
	},
	.cap = {
		.pclk = 30000000,
		.linelength = 1107,
		.framelength = 1387,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 200,
	},
	.cap1 = {
		.pclk = 42000000,
		.linelength = 1120,
		.framelength = 1546,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,	
	},
	.normal_video = {
		.pclk = 42000000,
		.linelength = 1120,
		.framelength = 1246,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 176000000,
		.linelength = 2880,
		.framelength = 1984,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 176000000,
		.linelength = 2688,
		.framelength = 1984,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.margin = 0,
	.min_shutter = 12,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 3,	  //support sensor mode num
	
	.cap_delay_frame = 2, 
	.pre_delay_frame = 2, 
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x78, 0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x78,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{
 { 1600, 1200,	  0,	0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200,	  0,	0, 1600,  1200}, // Preview 
 { 1600, 1200,	  0,	0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200,	  0,	0, 1600,  1200}, // capture 
 { 1600, 1200,	  0,	0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200,	  0,	0, 1600,  1200}, // video 
 { 1600, 1200,	  0,	0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200,	  0,	0, 1600,  1200}, //hight speed video 
 { 1600, 1200,	  0,	0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200,	  0,	0, 1600,  1200}};// slim video 


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[1] = {(char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
#if 1
	char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
#else
	iWriteReg((u16)addr, (u32)para, 2, imgsensor.i2c_write_id);
#endif
}

static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

	write_cmos_sensor(0x0006, imgsensor.frame_length);  
	write_cmos_sensor(0x0008, imgsensor.line_length);
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	unsigned long flags;

	LOG_INF("framerate = %d ", framerate);
   
	frame_length = (10 * imgsensor.pclk) / framerate / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.dummy_line = frame_length - imgsensor.frame_length;
	if (imgsensor.dummy_line < 0) imgsensor.dummy_line = 0;
	imgsensor.frame_length += imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	
	// Update Shutter
	
	if(shutter < 1) shutter = 1;
	if(shutter > 8191) shutter = 8191;//2^13
	//Update Shutter
	write_cmos_sensor(0x04, (shutter) & 0xFF);
	write_cmos_sensor(0x03, (shutter >> 8) & 0x1F);	

	LOG_INF("shutter =%d", shutter);

	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
	reg_gain = gain / 4 - 16;
	
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 iReg,temp;

	iReg = gain;
	if(256> iReg)
	{
	//analogic gain
	write_cmos_sensor(0xb0, 0x40); // global gain
	write_cmos_sensor(0xb1, iReg);//only digital gain 12.13
	}
	else
	{
	//analogic gain
	temp = 64*iReg/256;		
	write_cmos_sensor(0xb0, temp); // global gain
	write_cmos_sensor(0xb1, 0xff);//only digital gain 12.13
	
	}
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
	
	switch (image_mirror)
	{
		case IMAGE_NORMAL://IMAGE_NORMAL:
			write_cmos_sensor(0x17,0x14);//bit[1][0]
	//		write_cmos_sensor(0x92,0x03);
	//		write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
			write_cmos_sensor(0x17,0x15);
	//		GC2235_write_cmos_sensor(0x92,0x03);
	//		GC2235_write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
			write_cmos_sensor(0x17,0x16);
	//		GC2235_write_cmos_sensor(0x92,0x02);
	//		GC2235_write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
			write_cmos_sensor(0x17,0x17);
	//		GC2235_write_cmos_sensor(0x92,0x02);
	//		GC2235_write_cmos_sensor(0x94,0x0b);
			break;
	}


}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E");
	/* SYS */
/////////////////////////////////////////////////////
	//////////////////////	 SYS   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xf2, 0x00);
	write_cmos_sensor(0xf6, 0x00);
	write_cmos_sensor(0xfc, 0x06);

	write_cmos_sensor(0xf7, 0x15); //pll enable
	write_cmos_sensor(0xf8, 0x84); //Pll mode 2
	write_cmos_sensor(0xfa, 0x00); //div

	write_cmos_sensor(0xf9, 0xfe); //[0] pll enable
	write_cmos_sensor(0xfe, 0x00);
	
	/////////////////////////////////////////////////////
	////////////////   ANALOG & CISCTL	 ////////////////
	/////////////////////////////////////////////////////

	write_cmos_sensor(0x03, 0x05);
	write_cmos_sensor(0x04, 0x4b);
	write_cmos_sensor(0x05, 0x01);
	write_cmos_sensor(0x06, 0x1d);
	write_cmos_sensor(0x07, 0x00);
	write_cmos_sensor(0x08, 0x9b);

	write_cmos_sensor(0x0a, 0x02);
	write_cmos_sensor(0x0c, 0x00);
	write_cmos_sensor(0x0d, 0x04);
	write_cmos_sensor(0x0e, 0xd0);
	write_cmos_sensor(0x0f, 0x06); 
	write_cmos_sensor(0x10, 0x50);
	
	write_cmos_sensor(0x17, 0x15);//14 //[0]mirror [1]flip
	write_cmos_sensor(0x18, 0x12); //  0x1e
	write_cmos_sensor(0x19, 0x06);
	write_cmos_sensor(0x1a, 0x01);
	write_cmos_sensor(0x1b, 0x48);

	write_cmos_sensor(0x1e, 0x88); 
	write_cmos_sensor(0x1f, 0x48); 
	write_cmos_sensor(0x20, 0x03);
	write_cmos_sensor(0x21, 0x6f);
	write_cmos_sensor(0x22, 0x80); 
	write_cmos_sensor(0x23, 0xc1);
	write_cmos_sensor(0x24, 0x2f);//PAD_drv
	write_cmos_sensor(0x26, 0x01);
	write_cmos_sensor(0x27, 0x30);
	write_cmos_sensor(0x3f, 0x00);
	
	/////////////////////////////////////////////////////
	//////////////////////	 ISP   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0x8b, 0xa0);
	write_cmos_sensor(0x8c, 0x02);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, 0x03);
	write_cmos_sensor(0x94, 0x06);
	write_cmos_sensor(0x95, 0x04);
	write_cmos_sensor(0x96, 0xb0);
	write_cmos_sensor(0x97, 0x06);
	write_cmos_sensor(0x98, 0x40);
	
	/////////////////////////////////////////////////////
	//////////////////////	 BLK   //////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0x40, 0x72); //smooth speed 
	write_cmos_sensor(0x41, 0x04);
	write_cmos_sensor(0x43, 0x18); //global_offset 20140124 lanking
	write_cmos_sensor(0x5e, 0x00);
	write_cmos_sensor(0x5f, 0x00);
	write_cmos_sensor(0x60, 0x00);
	write_cmos_sensor(0x61, 0x00); 
	write_cmos_sensor(0x62, 0x00);
	write_cmos_sensor(0x63, 0x00); 
	write_cmos_sensor(0x64, 0x00);
	write_cmos_sensor(0x65, 0x00);
	write_cmos_sensor(0x66, 0x20);
	write_cmos_sensor(0x67, 0x20); 
	write_cmos_sensor(0x68, 0x20);
	write_cmos_sensor(0x69, 0x20);
	//Sleep(100);

	
	/////////////////////////////////////////////////////
	//////////////////////	 GAIN	/////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb3, 0x40);
	write_cmos_sensor(0xb4, 0x40);
	write_cmos_sensor(0xb5, 0x40);
	
	/////////////////////////////////////////////////////
	////////////////////   DARK SUN   ///////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0xb8, 0x0f);
	write_cmos_sensor(0xb9, 0x23);
	write_cmos_sensor(0xba, 0xff);
	write_cmos_sensor(0xbc, 0x00);
	write_cmos_sensor(0xbd, 0x00);
	write_cmos_sensor(0xbe, 0xff);
	write_cmos_sensor(0xbf, 0x09);

	/////////////////////////////////////////////////////
	//////////////////////	 OUTPUT	/////////////////////
	/////////////////////////////////////////////////////
	write_cmos_sensor(0xfe, 0x03);

	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0x02, 0x11);//mipi drv
	write_cmos_sensor(0x03, 0x11);//mipi drv
	write_cmos_sensor(0x06, 0x80);
	write_cmos_sensor(0x11, 0x2b);
	write_cmos_sensor(0x12, 0xd0);
	write_cmos_sensor(0x13, 0x07);
	write_cmos_sensor(0x15, 0x10);
	write_cmos_sensor(0x04, 0x20);
	write_cmos_sensor(0x05, 0x00);
	write_cmos_sensor(0x17, 0x01);


	write_cmos_sensor(0x21, 0x01);
	write_cmos_sensor(0x22, 0x02);
	write_cmos_sensor(0x23, 0x01);
	write_cmos_sensor(0x29, 0x02);
	write_cmos_sensor(0x2a, 0x01);

	write_cmos_sensor(0x10, 0x81);  // 93 line_sync_mode 

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf2, 0x00);

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E!\n");

  //MIPI//
  write_cmos_sensor(0xfe,0x03);
  write_cmos_sensor(0x12,0xd0);
  write_cmos_sensor(0x13,0x07);
  write_cmos_sensor(0x10,0x91);
  write_cmos_sensor(0xfe,0x00);	
	
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
		//max 24fps //
		  // AEC&frame length//
		  //MIPI//

  write_cmos_sensor(0xfe,0x03);
  write_cmos_sensor(0x12,0xd0);
  write_cmos_sensor(0x13,0x07);
  write_cmos_sensor(0x10,0x91);
  write_cmos_sensor(0xfe,0x00);	
	} else {   //30fps			//30fps for Normal capture & ZSD
		// max 30fps//
	
		   //MI
  write_cmos_sensor(0xfe,0x03);
  write_cmos_sensor(0x12,0xd0);
  write_cmos_sensor(0x13,0x07);
  write_cmos_sensor(0x10,0x91);
  write_cmos_sensor(0xfe,0x00);	
	}
		
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	

	 //MIPI//
	 write_cmos_sensor(0xfe,0x03);

	 write_cmos_sensor(0x10,0x91);
	 write_cmos_sensor(0xfe,0x00);
		
}
static void hs_video_setting()
{
	LOG_INF("E\n");
	//////////////////////////////////////////////////////////////////////////
	//			Sensor			 : Hi-544
	//		Mode			 :	
	//		Size			 : 1920 * 1080
	//			set file				 : v0.26
	//			Date						 : 20140510
	//////////////////////////////////////////////////////////////////////////
	
	write_cmos_sensor(0x0A00, 0x0000); //sleep On
	
	//--- Initial Set file ---//
	
	write_cmos_sensor(0x0B16, 0x4A0B);
	write_cmos_sensor(0x004C, 0x0100); 
	write_cmos_sensor(0x0032, 0x0101); 
	write_cmos_sensor(0x001E, 0x0101); 
	write_cmos_sensor(0x000C, 0x0000);
	
	write_cmos_sensor(0x0902, 0x4101); 
	write_cmos_sensor(0x090A, 0x03E4); 
	write_cmos_sensor(0x090C, 0x0020); 
	write_cmos_sensor(0x090E, 0x0020); 
	write_cmos_sensor(0x0910, 0x5D07); 
	write_cmos_sensor(0x0912, 0x061e); 
	write_cmos_sensor(0x0914, 0x0407); 
	write_cmos_sensor(0x0916, 0x0b0a); 
	write_cmos_sensor(0x0918, 0x0e09); 
	
	write_cmos_sensor(0x0012, 0x00BC); 
	write_cmos_sensor(0x0018, 0x0ABB); 
	
	write_cmos_sensor(0x0026, 0x0114); 
	write_cmos_sensor(0x002C, 0x06b3); 
	
	
	//--Crop size 2560x1440 ----///
	write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l
	write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l
	write_cmos_sensor(0x012C, 0x0A00); // digital_crop_image_width
	write_cmos_sensor(0x012E, 0x05A0); // digital_crop_image_height
	
	//----< Image FMT Size >--------------------//
	//Image size 1920x1080
	write_cmos_sensor(0x0110, 0x0780); //X_output_size_h	 
	write_cmos_sensor(0x0112, 0x0438); //Y_output_size_h  
	
	//----< Frame / Line Length >--------------//
	write_cmos_sensor(0x0006, 0x07C0); //frame_length_h 1984 @2560x1440@30.7fps
	write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	write_cmos_sensor(0x000A, 0x0DB0); //line_length for binning 3504
	//---------------------------------------//
	
	//--- ETC set ----//
	write_cmos_sensor(0x0000, 0x0000); 
	write_cmos_sensor(0x0700, 0x5090); 
	
	write_cmos_sensor(0x0032, 0x0101);	
	write_cmos_sensor(0x001E, 0x0101);	
	//----------------//
	
	//--- ISP enable Selection ---------------//
	write_cmos_sensor(0x0A04, 0x0133); //isp_en. [9]s-gamma,[8]MIPI_en,[6]compresion10to8,[5]Scaler,[4]window,[3]DG,[2]LSC,[1]adpc,[0]tpg //TEST PATTERN
	//----------------------------------------//
	
	write_cmos_sensor(0x0A00, 0x0100); //sleep Off 
	
	mDELAY(30);
}

static void slim_video_setting()
{
	LOG_INF("E\n");
	//////////////////////////////////////////////////////////////////////////
	//			Sensor			 : Hi-544
	//		Mode			 :	
	//		Size			 : 1280 * 720
	//			set file				 : v0.26
	//			Date						 : 20140510
	//////////////////////////////////////////////////////////////////////////
	
	write_cmos_sensor(0x0A00, 0x0000); //sleep On
	
	//--- Initial Set file ---//
	
	write_cmos_sensor(0x0B16, 0x4A0B);
	write_cmos_sensor(0x004C, 0x0100); 
	write_cmos_sensor(0x0032, 0x0101); 
	write_cmos_sensor(0x001E, 0x0101); 
	write_cmos_sensor(0x000C, 0x0000);
	
	write_cmos_sensor(0x0902, 0x4101); 
	write_cmos_sensor(0x090A, 0x03E4); 
	write_cmos_sensor(0x090C, 0x0020); 
	write_cmos_sensor(0x090E, 0x0020); 
	write_cmos_sensor(0x0910, 0x5D07); 
	write_cmos_sensor(0x0912, 0x061e); 
	write_cmos_sensor(0x0914, 0x0407); 
	write_cmos_sensor(0x0916, 0x0b0a); 
	write_cmos_sensor(0x0918, 0x0e09); 
	 
	write_cmos_sensor(0x0012, 0x00BC); 
	write_cmos_sensor(0x0018, 0x0ABB); 
	 
	write_cmos_sensor(0x0026, 0x0114); 
	write_cmos_sensor(0x002C, 0x06b3); 
	
	
	//--Crop size 2560x1440 ----///
	write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l
	write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l
	write_cmos_sensor(0x012C, 0x0A00); // digital_crop_image_width
	write_cmos_sensor(0x012E, 0x05A0); // digital_crop_image_height
	
	//----< Image FMT Size >--------------------//
	//Image size 1280x720
	write_cmos_sensor(0x0110, 0x0500); //X_output_size_h	 
	write_cmos_sensor(0x0112, 0x02d0); //Y_output_size_h  
	
	//----< Frame / Line Length >--------------//
	write_cmos_sensor(0x0006, 0x07C0); //frame_length_h 1984 @2560x1440@30.7fps
	write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	write_cmos_sensor(0x000A, 0x0DB0); //line_length for binning 3504
	
	//--- ETC set ----//
	write_cmos_sensor(0x0000, 0x0000); 
	write_cmos_sensor(0x0700, 0xA1A8); 
	
	write_cmos_sensor(0x0032, 0x0101);	
	write_cmos_sensor(0x001E, 0x0101);	
	
	//--- ISP enable Selection ---------------//
	write_cmos_sensor(0x0A04, 0x0133); //TEST PATTERN
	//----------------------------------------//
	
	write_cmos_sensor(0x0A00, 0x0100); //sleep Off

}

#ifdef VIDEO_720P

static void video_720p_setting(void)
{
	LOG_INF("E\n");

	//5.1.4 Video BQ720p Full FOV 30fps 24M MCLK 2lane 864Mbps/lane
	write_cmos_sensor(0x0100,0x00);  //software sleep
	
	write_cmos_sensor(0x3708,0xe6);  //sensor control
	write_cmos_sensor(0x3709,0xc3);  //sensor control
	write_cmos_sensor(0x3803,0xf4);  //timing Y start L
	write_cmos_sensor(0x3806,0x06);  //timing Y end H
	write_cmos_sensor(0x3807,0xab);  //timing Y end L
	
	write_cmos_sensor(0x3808,0x05);  //X output size H	1280
	write_cmos_sensor(0x3809,0x00);  //X output size L
	write_cmos_sensor(0x380a,0x02);  //Y output size H	720
	write_cmos_sensor(0x380b,0xd0);  //Y output size L
	
	//write_cmos_sensor(0x380c,0x0d);  //HTS H	2688
	//write_cmos_sensor(0x380d,0xb0);  //HTS L
	//write_cmos_sensor(0x380e,0x05);  //VTS H	1984
	//write_cmos_sensor(0x380f,0xf0);  //VTS L
	write_cmos_sensor(0x380c, ((imgsensor_info.video2.linelength >> 8) & 0xFF)); // hts = 2688
	write_cmos_sensor(0x380d, (imgsensor_info.video2.linelength & 0xFF));		 // hts
	write_cmos_sensor(0x380e, ((imgsensor_info.video2.framelength >> 8) & 0xFF));  // vts = 1984
	write_cmos_sensor(0x380f, (imgsensor_info.video2.framelength & 0xFF));		   // vts	  
	
	write_cmos_sensor(0x3810,0x00);  //timing ISP x win H
	write_cmos_sensor(0x3811,0x08);  //timing ISP x win L
	write_cmos_sensor(0x3812,0x00);  //timing ISP y win H
	write_cmos_sensor(0x3813,0x02);  //timing ISP y win L
	write_cmos_sensor(0x3814,0x31);  //timing X inc
	write_cmos_sensor(0x3815,0x31);  //timing Y inc

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	write_cmos_sensor(0x3820,0x01);  //v fast vbin of, flip off, v bin on
	write_cmos_sensor(0x3821,0x1f);  //hsync on, h mirror on, h bin on
	
	write_cmos_sensor(0x3a04,0x06);
	write_cmos_sensor(0x3a05,0x14);
	write_cmos_sensor(0x5002,0x00);  //scale off
	
	write_cmos_sensor(0x0100,0x01);  //wake up

	LOG_INF("Exit!\n");
}

#elif defined VIDEO_1080P

static void video_1080p_setting(void)
{
	LOG_INF("E\n");
	
	//5.1.5 Video 1080p 30fps 24M MCLK 2lane 864Mbps/lane
	write_cmos_sensor(0x0100,0x00);  //software sleep

	write_cmos_sensor(0x3708,0xe2);  //sensor control
	write_cmos_sensor(0x3709,0xc3);  //sensor control
	write_cmos_sensor(0x3803,0xf8);  //timing Y start L
	write_cmos_sensor(0x3806,0x06);  //timing Y end H
	write_cmos_sensor(0x3807,0xab);  //timing Y end L

	write_cmos_sensor(0x3808,0x07);  //X output size H	1920
	write_cmos_sensor(0x3809,0x80);  //X output size L
	write_cmos_sensor(0x380a,0x04);  //Y output size H	1080
	write_cmos_sensor(0x380b,0x38);  //Y output size L

	//write_cmos_sensor(0x380c,0x0a);  //HTS H	2688
	//write_cmos_sensor(0x380d,0x80);  //HTS L
	//write_cmos_sensor(0x380e,0x07);  //VTS H	1984
	//write_cmos_sensor(0x380f,0xc0);  //VYS L
	write_cmos_sensor(0x380c, ((imgsensor_info.video1_linelength >> 8) & 0xFF)); // hts = 2688
	write_cmos_sensor(0x380d, (imgsensor_info.video1_linelength & 0xFF));		 // hts
	write_cmos_sensor(0x380e, ((imgsensor_info.video1_framelength >> 8) & 0xFF));  // vts = 1984
	write_cmos_sensor(0x380f, (imgsensor_info.video1_framelength & 0xFF));		   // vts		

	write_cmos_sensor(0x3810,0x00);  //timing ISP x win H
	write_cmos_sensor(0x3811,0x02);  //timing ISP x win L
	write_cmos_sensor(0x3812,0x00);  //timing ISP y win H
	write_cmos_sensor(0x3813,0x02);  //timing ISP y win L
	write_cmos_sensor(0x3814,0x11);  //timing X inc
	write_cmos_sensor(0x3815,0x11);  //timing Y inc

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	write_cmos_sensor(0x3820,0x00);  //v bin off
	write_cmos_sensor(0x3821,0x1e);  //hsync on, h mirror on, h bin off
	
	write_cmos_sensor(0x3a04,0x06);
	write_cmos_sensor(0x3a05,0x14);
	write_cmos_sensor(0x5002,0x80);  //scale on

	write_cmos_sensor(0x0100,0x01);  //wake up

	LOG_INF("Exit!\n");
}

#else

static void video_setting(void)
{
	LOG_INF("ihdr_en:%d\n",imgsensor.ihdr_en);
	

	 //MIPI//
	write_cmos_sensor(0xfe,0x03);
	write_cmos_sensor(0x10,0x91); 
	write_cmos_sensor(0xfe,0x00);

}

#endif

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, write_id: 0x%x, sensor_id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_INF("PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, lanking___id: 0x%x\n", sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.current_fps = 300;
	if (imgsensor.current_fps == 240) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.current_fps = imgsensor_info.cap.max_framerate;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame; 
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_int16 dummyLine;
	kal_uint32 lineLength,frameHeight;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frameHeight = (10 * imgsensor_info.pre.pclk) / framerate / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.pre.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length += imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			frameHeight = (10 * imgsensor_info.normal_video.pclk) / framerate / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.normal_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;			
			imgsensor.frame_length += imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			frameHeight = (10 * imgsensor_info.cap.pclk) / framerate / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.cap.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length += imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frameHeight = (10 * imgsensor_info.hs_video.pclk) / framerate / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.hs_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frameHeight = (10 * imgsensor_info.slim_video.pclk) / framerate / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.slim_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;			
			imgsensor.frame_length += imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
		default:			
			LOG_INF("error scenario_id = %d\n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5E00, 0x80);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5E00, 0x00);
	}	 
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data_16);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data_16);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data_16);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data_16);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, *(feature_data_32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data_16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", *feature_data_16);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data_16;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
			LOG_INF("ihdr enable :%d\n", *feature_data_16);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = *feature_data_16;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
			wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(*(feature_data_32+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			break;
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",*feature_data_32,*(feature_data_32+1),*(feature_data_32+2)); 
			ihdr_write_shutter_gain(*feature_data_32,*(feature_data_32+1),*(feature_data_32+2));	
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 GC2235_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	GC2235_MIPI_RAW_SensorInit	*/
