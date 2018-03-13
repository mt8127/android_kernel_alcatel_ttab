/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 OV56702NDmipi_Sensor.c
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
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov56702ndmipi_Sensor.h"

#include "ov56702ndmipiraw_Camera_Sensor_para.h"
#include "ov56702ndmipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(ov56702ndmipiraw_drv_lock);
static int module_id = 0;

#define OV56702ND_DEBUG
#ifdef OV56702ND_DEBUG
	#define OV56702NDDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV56702NDRaw] ",  fmt, ##arg)
#else
	#define OV56702NDDB(fmt, arg...)
#endif


kal_uint32 OV56702ND_FeatureControl_PERIOD_PixelNum=OV56702ND_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV56702ND_FeatureControl_PERIOD_LineNum=OV56702ND_PV_PERIOD_LINE_NUMS;

UINT16 OV56702ND_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV56702NDCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV56702NDSensorConfigData;
static OV56702ND_PARA_STRUCT OV56702ND;
kal_uint32 OV56702ND_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV56702NDSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV56702NDSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;


#define OV56702ND_TEST_PATTERN_CHECKSUM 0xca3667da //0x5d8082f0 //0x75bef806 //0xa2230d9f    //0xf5e2f1ce
kal_bool OV56702ND_During_testpattern = KAL_FALSE;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV56702ND_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV56702NDMIPI_WRITE_ID)

kal_uint16 OV56702ND_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV56702NDMIPI_WRITE_ID);
    return get_byte;
}


void OV56702ND_Init_Para(void)
{

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND.sensorMode = SENSOR_MODE_INIT;
	OV56702ND.OV56702NDAutoFlickerMode = KAL_FALSE;
	OV56702ND.OV56702NDVideoMode = KAL_FALSE;
	OV56702ND.DummyLines= 0;
	OV56702ND.DummyPixels= 0;
	OV56702ND.pvPclk =  (10285); 
	OV56702ND.videoPclk = (10285);
	OV56702ND.capPclk = (10285);

	OV56702ND.shutter = 0x4C00;
	OV56702ND.ispBaseGain = BASEGAIN;
	OV56702ND.sensorGlobalGain = 0x0200;
	spin_unlock(&ov56702ndmipiraw_drv_lock);
}

#define RG_Ratio_Typical 264
#define BG_Ratio_Typical 236

struct otp_struct {
   int flag; // bit[7]: info, bit[6]:wb
   int module_integrator_id;
   int lens_id;
   int production_year;
   int production_month;
   int production_day;
   int rg_ratio;
   int bg_ratio;
};


static void otp_i2c_write( uint32_t addr, kal_uint16 data)
{
  OV56702ND_write_cmos_sensor(addr, data);
}

static kal_uint16 otp_i2c_read(uint32_t addr)

{
	return OV56702ND_read_cmos_sensor(addr);
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
static int read_otp(struct otp_struct *otp_ptr)
{
   int otp_flag, addr, temp, i;
   //set 0x5002[3] to “0”
   int temp1;
   
   temp1 = otp_i2c_read(0x5002);
   otp_i2c_write(0x5002, (0x00 & 0x08) | (temp1 & (~0x08)));
   
   // read OTP into buffer
   otp_i2c_write(0x3d84, 0xC0);
   otp_i2c_write(0x3d88, 0x70); // OTP start address
   otp_i2c_write(0x3d89, 0x10);
   otp_i2c_write(0x3d8A, 0x70); // OTP end address
   otp_i2c_write(0x3d8B, 0x29);
   otp_i2c_write(0x3d81, 0x01); // load otp into buffer
   mdelay(5);
   
   // OTP into
   otp_flag = otp_i2c_read(0x7010);
   addr = 0;
   
   if((otp_flag & 0xc0) == 0x40) {
      addr = 0x7011; // base address of info group 1
   } else if((otp_flag & 0x30) == 0x10) {
      addr = 0x7016; // base address of info group 2
   }else if((otp_flag & 0x0c) == 0x04) {
      addr = 0x701b; // base address of info group 3
   }
   
   if(addr != 0) {
        (*otp_ptr).flag = 0x80; // valid base info in OTP
        (*otp_ptr).module_integrator_id = otp_i2c_read( addr );
        (*otp_ptr).lens_id = otp_i2c_read( addr + 1);
        (*otp_ptr).production_year = otp_i2c_read( addr + 2);
        (*otp_ptr).production_month = otp_i2c_read( addr + 3);
        (*otp_ptr).production_day = otp_i2c_read( addr + 4);
   } else {
        (*otp_ptr).flag = 0x00; // not info in OTP
        (*otp_ptr).module_integrator_id = 0;
        (*otp_ptr).lens_id = 0;
        (*otp_ptr).production_year = 0;
        (*otp_ptr).production_month = 0;
        (*otp_ptr).production_day = 0;
   }

   module_id = (*otp_ptr).module_integrator_id;
   // OTP WB Calibration
   otp_flag = otp_i2c_read(0x7020);
   addr = 0;
   
   if((otp_flag & 0xc0) == 0x40) {
     addr = 0x7021; // base address of WB Calibration group 1
   }else if((otp_flag & 0x30) == 0x10) {
     addr = 0x7024; // base address of WB Calibration group 2
   }else if((otp_flag & 0x0c) == 0x04) {
     addr = 0x7027; // base address of WB Calibration group 3
   }
   
   if(addr != 0) {
     (*otp_ptr).flag |= 0x40;
     temp = otp_i2c_read( addr + 2);
     (*otp_ptr).rg_ratio = (otp_i2c_read(addr)<<2) + ((temp>>6) & 0x03);
     (*otp_ptr).bg_ratio = (otp_i2c_read( addr + 1)<<2) + ((temp>>4) & 0x03);
   }else {
     (*otp_ptr).rg_ratio = 0;
     (*otp_ptr).bg_ratio = 0;
   }
   
   for(i=0x7010;i<=0x7029;i++) {
     otp_i2c_write(i,0); // clear OTP buffer, recommended use continuous write to accelarate
   }
   
   //set 0x5002[3] to “1”
   temp1 = otp_i2c_read(0x5002);
   otp_i2c_write(0x5002, (0x02 & 0x08) | (temp1 & (~0x08)));
   return (*otp_ptr).flag ;
}

static int apply_otp(struct otp_struct *otp_ptr)
{
	 int rg, bg, R_gain, G_gain, B_gain, Base_gain;

   // apply OTP WB Calibration
   if ((*otp_ptr).flag & 0x40) {
        rg = (*otp_ptr). rg_ratio;
        bg = (*otp_ptr).bg_ratio;
        //calculate G gain
        R_gain = (RG_Ratio_Typical*1000) / rg;
        B_gain = (BG_Ratio_Typical*1000) / bg;
        G_gain = 1000;    
        
        if (R_gain < 1000 || B_gain < 1000)
        {
             if (R_gain < B_gain)
               Base_gain = R_gain;
             else
               Base_gain = B_gain;
        }
        else
        {
             Base_gain = G_gain;
        }
        
        R_gain = 0x400 * R_gain / (Base_gain);
        B_gain = 0x400 * B_gain / (Base_gain);
        G_gain = 0x400 * G_gain / (Base_gain);
        // update sensor WB gain
        if (R_gain>0x400) {
        otp_i2c_write(0x5032, R_gain>>8);
        otp_i2c_write(0x5033, R_gain & 0x00ff);
        }
        if (G_gain>0x400) {
        otp_i2c_write(0x5034, G_gain>>8);
        otp_i2c_write(0x5035, G_gain & 0x00ff);
        }
        if (B_gain>0x400) {
        otp_i2c_write(0x5036, B_gain>>8);
        otp_i2c_write(0x5037, B_gain & 0x00ff);
        }
   }
   return (*otp_ptr).flag ;	
}

void ov56702nd_otp_config()
{   
	struct otp_struct otp_info ;  
	
	read_otp(&otp_info);   
	apply_otp(&otp_info) ;
}
kal_uint32 GetOV56702NDLineLength(void)
{
	kal_uint32 OV56702ND_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == OV56702ND.sensorMode )  
	{
		OV56702ND_line_length = OV56702ND_PV_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == OV56702ND.sensorMode ) 
	{
		OV56702ND_line_length = OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels;
	}
	else
	{
		OV56702ND_line_length = OV56702ND_FULL_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels;
	}

    return OV56702ND_line_length;

}


kal_uint32 GetOV56702NDFrameLength(void)
{
	kal_uint32 OV56702ND_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV56702ND.sensorMode )  
	{
		OV56702ND_frame_length = OV56702ND_PV_PERIOD_LINE_NUMS + OV56702ND.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == OV56702ND.sensorMode ) 
	{
		OV56702ND_frame_length = OV56702ND_VIDEO_PERIOD_LINE_NUMS + OV56702ND.DummyLines ;
	}
	else
	{
		OV56702ND_frame_length = OV56702ND_FULL_PERIOD_LINE_NUMS + OV56702ND.DummyLines ;
	}

	return OV56702ND_frame_length;
}


kal_uint32 OV56702ND_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

    return extra_lines;

}


kal_uint32 OV56702ND_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV56702NDCurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (OV56702ND.capPclk*10000) /(OV56702ND_FULL_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/OV56702ND_AUTOFLICKER_OFFSET_25*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV56702ND_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (OV56702ND.videoPclk*10000) /(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/OV56702ND_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV56702ND_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (OV56702ND.videoPclk*10000) /(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/OV56702ND_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV56702ND_VIDEO_PERIOD_LINE_NUMS + OV56702ND.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (OV56702ND.pvPclk*10000) /(OV56702ND_PV_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/OV56702ND_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	OV56702NDDB("AutoFlicker_min_framelength =%d,OV56702NDCurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV56702NDCurrentScenarioId);

	return AutoFlicker_min_framelength;

}


void OV56702ND_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = OV56702ND_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	//for test
	//shutter = 0x7fc;  //issue 
	
	
    line_length  = GetOV56702NDLineLength();
	frame_length = GetOV56702NDFrameLength();
	
	max_shutter  = frame_length-OV56702ND_SHUTTER_MARGIN;

    frame_length = frame_length + OV56702ND_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(OV56702ND.OV56702NDAutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV56702ND_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}
	

	spin_lock_irqsave(&ov56702ndmipiraw_drv_lock,flags);
	OV56702ND_FeatureControl_PERIOD_PixelNum = line_length;
	OV56702ND_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&ov56702ndmipiraw_drv_lock,flags);

	//Set total frame length  //VTS
	OV56702ND_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV56702ND_write_cmos_sensor(0x380f, frame_length & 0xFF);

	//Set shutter 
	OV56702ND_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV56702ND_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV56702ND_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	  /* Don't use the fraction part. */

	OV56702NDDB("ov56702nd write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);

}


static kal_uint16 OV56702NDReg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV56702ND_GAIN_BASE;
	return iGain;
	
}

static kal_uint16 OV56702NDGain2Reg(const kal_uint16 Gain)
{
    kal_uint32 iReg = 0x0000;
	kal_uint32 TempGain = BASEGAIN;


	TempGain = Gain;
	if(TempGain < BASEGAIN){
		TempGain = BASEGAIN;
		//OV56702NDDB("###ov56702nd write gain underflow### Gain =%x\n", Gain);
	}
	if(TempGain > 16*BASEGAIN){
		TempGain = 16*BASEGAIN;
		//OV56702NDDB("###ov56702nd write gain overflow### Gain =%x\n", Gain);
	}

	iReg = (TempGain*OV56702ND_GAIN_BASE)/BASEGAIN;

	//iReg = ((TempGain /BASEGAIN)<<7)+((TempGain % BASEGAIN)<<7/BASEGAIN);
	iReg = iReg & 0xFFFF;

	//OV56702NDDB("###ov56702nd write Reg ### iReg =%x\n", iReg);

    return iReg;

}

void write_OV56702ND_gain(kal_uint16 gain)
{
	kal_uint16 iGain =1;
	kal_uint8 ChangeFlag=0x01;

	kal_uint16 read_gain;
	
	iGain=(gain / OV56702ND_GAIN_BASE);

	if(iGain<2){
		ChangeFlag= 0x00;
	}
	else if(iGain<4){
		ChangeFlag= 0x01;
	}
	else if(iGain<8){
		ChangeFlag= 0x03;
	}
	else{
		ChangeFlag= 0x07;
	}

	//ChangeFlag= 0x07;
	
	OV56702ND_write_cmos_sensor(0x301d, 0xf0);
	OV56702ND_write_cmos_sensor(0x3209, 0x00);
	OV56702ND_write_cmos_sensor(0x320a, 0x01);
	
	//group write  hold
	//group 0:delay 0x366a for one frame,then active with gain
	OV56702ND_write_cmos_sensor(0x3208, 0x00);
	OV56702ND_write_cmos_sensor(0x366a, ChangeFlag);
	OV56702ND_write_cmos_sensor(0x3208, 0x10);

	//group 1:all other registers( gain)
	OV56702ND_write_cmos_sensor(0x3208, 0x01);
	OV56702ND_write_cmos_sensor(0x3508,(gain>>8));
    OV56702ND_write_cmos_sensor(0x3509,(gain&0xff));
	
	OV56702ND_write_cmos_sensor(0x3208, 0x11);

	//group lanch
	OV56702ND_write_cmos_sensor(0x320B, 0x15);
	OV56702ND_write_cmos_sensor(0x3208, 0xA1);

	//read_gain=(((OV56702ND_read_cmos_sensor(0x3508)&0x1F) << 8) | OV56702ND_read_cmos_sensor(0x3509));
	//OV56702NDDB("[OV56702ND_SetGain]0x3508|0x3509=0x%x \n",read_gain);
	//OV56702NDDB("[OV56702ND_SetGain]0x366a=%d \n",(OV56702ND_read_cmos_sensor(0x366a)));

	return;

}

void OV56702ND_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&ov56702ndmipiraw_drv_lock,flags);

	OV56702NDDB("OV56702ND_SetGain iGain = %d :\n ",iGain);
	
	OV56702ND.realGain = iGain;
	OV56702ND.sensorGlobalGain = OV56702NDGain2Reg(iGain);
	spin_unlock_irqrestore(&ov56702ndmipiraw_drv_lock,flags);

	write_OV56702ND_gain(OV56702ND.sensorGlobalGain);
	OV56702NDDB(" [OV56702ND_SetGain]OV56702ND.sensorGlobalGain=0x%x,OV56702ND.realGain =%d",OV56702ND.sensorGlobalGain,
		OV56702ND.realGain); 

	//temperature test
	//OV56702ND_write_cmos_sensor(0x4d12,0x01);
	//OV56702NDDB("Temperature read_reg  0x4d13  =%x \n",OV56702ND_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV56702ND_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV56702ND_read_cmos_sensor(0x3508)&0x1F) << 8) | OV56702ND_read_cmos_sensor(0x3509));

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND.sensorGlobalGain = read_gain;
	OV56702ND.realGain = OV56702NDReg2Gain(OV56702ND.sensorGlobalGain);
	spin_unlock(&ov56702ndmipiraw_drv_lock);

	OV56702NDDB("OV56702ND.sensorGlobalGain=0x%x,OV56702ND.realGain=%d\n",OV56702ND.sensorGlobalGain,OV56702ND.realGain);

	return OV56702ND.sensorGlobalGain;
}  


#if 1
void OV56702ND_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV56702NDSensorReg[i].Addr; i++)
    {
        OV56702ND_write_cmos_sensor(OV56702NDSensorReg[i].Addr, OV56702NDSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV56702NDSensorReg[i].Addr; i++)
    {
        OV56702ND_write_cmos_sensor(OV56702NDSensorReg[i].Addr, OV56702NDSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV56702ND_write_cmos_sensor(OV56702NDSensorCCT[i].Addr, OV56702NDSensorCCT[i].Para);
    }
}

void OV56702ND_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV56702NDSensorReg[i].Addr; i++)
    {
         temp_data = OV56702ND_read_cmos_sensor(OV56702NDSensorReg[i].Addr);
		 spin_lock(&ov56702ndmipiraw_drv_lock);
		 OV56702NDSensorReg[i].Para =temp_data;
		 spin_unlock(&ov56702ndmipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV56702NDSensorReg[i].Addr; i++)
    {
        temp_data = OV56702ND_read_cmos_sensor(OV56702NDSensorReg[i].Addr);
		spin_lock(&ov56702ndmipiraw_drv_lock);
		OV56702NDSensorReg[i].Para = temp_data;
		spin_unlock(&ov56702ndmipiraw_drv_lock);
    }
}

kal_int32  OV56702ND_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV56702ND_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void OV56702ND_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= OV56702NDSensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/OV56702ND.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV56702ND_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV56702ND_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool OV56702ND_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * OV56702ND.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&ov56702ndmipiraw_drv_lock);
          OV56702NDSensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&ov56702ndmipiraw_drv_lock);
          OV56702ND_write_cmos_sensor(OV56702NDSensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&ov56702ndmipiraw_drv_lock);
                    OV56702ND_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&ov56702ndmipiraw_drv_lock);
                    break;
                case 1:
                    OV56702ND_write_cmos_sensor(OV56702ND_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}
#endif


static void OV56702ND_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV56702ND.sensorMode )
	{
		line_length = OV56702ND_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV56702ND_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== OV56702ND.sensorMode )
	{
		line_length = OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV56702ND_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV56702ND_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV56702ND_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND_FeatureControl_PERIOD_PixelNum = line_length;
	OV56702ND_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov56702ndmipiraw_drv_lock);

	//Set total frame length
	OV56702ND_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV56702ND_write_cmos_sensor(0x380f, frame_length & 0xFF);
	//Set total line length
	OV56702ND_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV56702ND_write_cmos_sensor(0x380d, line_length & 0xFF);

}   


void OV56702NDPreviewSetting(void)
{
	OV56702NDDB(" OV56702NDPreviewSetting_2lane enter\n");

	/*  //
	
	//@@PV_Quarter_size_30fps_800Mbps/lane				  
	//99 1296 960 										  
	//;;102 3601	157c									  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M

		OV56702ND_write_cmos_sensor(0x0100, 0x00);  //
	
	OV56702ND_write_cmos_sensor(0x3501, 0x3d);  //
	OV56702ND_write_cmos_sensor(0x366e, 0x08);  //
	OV56702ND_write_cmos_sensor(0x370b, 0x1b);  //
	OV56702ND_write_cmos_sensor(0x3808, 0x05);  //
	OV56702ND_write_cmos_sensor(0x3809, 0x10);  //
	OV56702ND_write_cmos_sensor(0x380a, 0x03);  //
	OV56702ND_write_cmos_sensor(0x380b, 0xc0);  //
	OV56702ND_write_cmos_sensor(0x380c, 0x06);  //
	OV56702ND_write_cmos_sensor(0x380d, 0x8c);  //
	OV56702ND_write_cmos_sensor(0x380e, 0x07);  //;03
	OV56702ND_write_cmos_sensor(0x380f, 0xfd);  //;e0
	OV56702ND_write_cmos_sensor(0x3814, 0x03);  //
	OV56702ND_write_cmos_sensor(0x3820, 0x90);  //
	OV56702ND_write_cmos_sensor(0x3821, 0x47);  //
	OV56702ND_write_cmos_sensor(0x382a, 0x03);  //
	OV56702ND_write_cmos_sensor(0x4009, 0x05);  //
	OV56702ND_write_cmos_sensor(0x4502, 0x48);  //
	OV56702ND_write_cmos_sensor(0x4508, 0x55);  //
	OV56702ND_write_cmos_sensor(0x4509, 0x55);  //
	OV56702ND_write_cmos_sensor(0x4600, 0x00);  //
	OV56702ND_write_cmos_sensor(0x4601, 0x81);  //
	OV56702ND_write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning sum format.
	OV56702ND_write_cmos_sensor(0x400a, 0x02);  //;
	OV56702ND_write_cmos_sensor(0x400b, 0x00);  //; 
	
	OV56702ND_write_cmos_sensor(0x0100, 0x01);  //
*/

	//@@PV_Quarter_size_30fps_800Mbps/lane_1296x972							  
	//99 1296 972 															  
	//;;102 3601	157c														  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M					  
																			  
	OV56702ND_write_cmos_sensor(0x0100, 0x00);  // 	
	
	OV56702ND_write_cmos_sensor(0x3501, 0x73);  // 							  
	OV56702ND_write_cmos_sensor(0x3502, 0x00);  // 							  
	OV56702ND_write_cmos_sensor(0x3508, 0x01);  // 							  
	OV56702ND_write_cmos_sensor(0x3509, 0x80);  // 							  
	OV56702ND_write_cmos_sensor(0x366e, 0x08);  // 							  
	OV56702ND_write_cmos_sensor(0x370b, 0x1b);  // 							  
	OV56702ND_write_cmos_sensor(0x3808, 0x05);  // 							  
	OV56702ND_write_cmos_sensor(0x3809, 0x10);  // 							  
	OV56702ND_write_cmos_sensor(0x380a, 0x03);  // 							  
	OV56702ND_write_cmos_sensor(0x380b, 0xcc);  //;c0							  
	OV56702ND_write_cmos_sensor(0x380c, 0x06);  // 							  
	OV56702ND_write_cmos_sensor(0x380d, 0x8c);  // 							  
	OV56702ND_write_cmos_sensor(0x380e, 0x07);  //;03							  
	OV56702ND_write_cmos_sensor(0x380f, 0xfd);  //;e0							  
	OV56702ND_write_cmos_sensor(0x3814, 0x03);  // 							  
	OV56702ND_write_cmos_sensor(0x3820, 0x90);  // 							  
	OV56702ND_write_cmos_sensor(0x3821, 0x47);  // 							  
	OV56702ND_write_cmos_sensor(0x382a, 0x03);  // 							  
	OV56702ND_write_cmos_sensor(0x3845, 0x02);  // 							  
	OV56702ND_write_cmos_sensor(0x4009, 0x05);  // 							  
	OV56702ND_write_cmos_sensor(0x4502, 0x48);  // 							  
	OV56702ND_write_cmos_sensor(0x4508, 0x55);  // 							  
	OV56702ND_write_cmos_sensor(0x4509, 0x55);  // 							  
	OV56702ND_write_cmos_sensor(0x4600, 0x00);  // 							  
	OV56702ND_write_cmos_sensor(0x4601, 0x81);  // 							  
	OV56702ND_write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning 
	OV56702ND_write_cmos_sensor(0x400a, 0x02);  //;							  
	OV56702ND_write_cmos_sensor(0x400b, 0x00);  //;	
	
	OV56702ND_write_cmos_sensor(0x0100, 0x01);  // 	
}


void OV56702NDVideoSetting(void)
{
	OV56702NDDB(" OV56702NDvideoSetting_2lane enter:video/preview sync\n");

	OV56702NDPreviewSetting();
}



void OV56702NDCaptureSetting(void)
{
	OV56702NDDB("OV56702NDCaptureSetting_2lane enter\n");

	OV56702ND_write_cmos_sensor(0x0100, 0x00); 
	
	OV56702ND_write_cmos_sensor(0x3501, 0x5f); //long exposure
	OV56702ND_write_cmos_sensor(0x3502, 0xd0);  //long exposure
	
	OV56702ND_write_cmos_sensor(0x3508, 0x03);  //gain
	OV56702ND_write_cmos_sensor(0x3509, 0x00);  //gain
	
	OV56702ND_write_cmos_sensor(0x366e, 0x10); 
	OV56702ND_write_cmos_sensor(0x370b, 0x1b); 
	OV56702ND_write_cmos_sensor(0x3808, 0x0a); 
	OV56702ND_write_cmos_sensor(0x3809, 0x20); 
	OV56702ND_write_cmos_sensor(0x380a, 0x07); 
	OV56702ND_write_cmos_sensor(0x380b, 0x98); 
	OV56702ND_write_cmos_sensor(0x380c, 0x07); //;06
	OV56702ND_write_cmos_sensor(0x380d, 0xdc); //;8c
	OV56702ND_write_cmos_sensor(0x380e, 0x07); 
	OV56702ND_write_cmos_sensor(0x380f, 0xfd); 
	OV56702ND_write_cmos_sensor(0x3814, 0x01); 
	OV56702ND_write_cmos_sensor(0x3820, 0x80); 
	OV56702ND_write_cmos_sensor(0x3821, 0x46); 
	OV56702ND_write_cmos_sensor(0x382a, 0x01);
	
	OV56702ND_write_cmos_sensor(0x3845, 0x00);  //v_offset for auto size mode
	
	OV56702ND_write_cmos_sensor(0x4009, 0x0d); 
	OV56702ND_write_cmos_sensor(0x4502, 0x40); 
	OV56702ND_write_cmos_sensor(0x4508, 0xaa); 
	OV56702ND_write_cmos_sensor(0x4509, 0xaa); 
	OV56702ND_write_cmos_sensor(0x4600, 0x01); 
	OV56702ND_write_cmos_sensor(0x4601, 0x03); 
	OV56702ND_write_cmos_sensor(0x4017, 0x08); //threshold= 2LSB for full size
	OV56702ND_write_cmos_sensor(0x400a, 0x02); //
	OV56702ND_write_cmos_sensor(0x400b, 0x00); //
	
	OV56702ND_write_cmos_sensor(0x0100, 0x01); 
	
}


static void OV56702ND_Sensor_Init(void)
{
	OV56702NDDB("OV56702ND_Sensor_Init_2lane enter\n");
	
	OV56702ND_write_cmos_sensor(0x0103,0x01);// ; software reset
	mdelay(10);
	OV56702ND_write_cmos_sensor(0x0100, 0x00);// ; software standby
	OV56702ND_write_cmos_sensor(0x0100, 0x00); 
	OV56702ND_write_cmos_sensor(0x0300, 0x04); 
	OV56702ND_write_cmos_sensor(0x0301, 0x00); 
	OV56702ND_write_cmos_sensor(0x0302, 0x64); //;78
	OV56702ND_write_cmos_sensor(0x0303, 0x00); 
	OV56702ND_write_cmos_sensor(0x0304, 0x03); 
	OV56702ND_write_cmos_sensor(0x0305, 0x01); 
	OV56702ND_write_cmos_sensor(0x0306, 0x01); 
	OV56702ND_write_cmos_sensor(0x030a, 0x00); 
	OV56702ND_write_cmos_sensor(0x030b, 0x00); 
	OV56702ND_write_cmos_sensor(0x030c, 0x00); 
	OV56702ND_write_cmos_sensor(0x030d, 0x1e); 
	OV56702ND_write_cmos_sensor(0x030e, 0x00); 
	OV56702ND_write_cmos_sensor(0x030f, 0x06); 
	OV56702ND_write_cmos_sensor(0x0312, 0x01); 
	OV56702ND_write_cmos_sensor(0x3000, 0x00); 
	OV56702ND_write_cmos_sensor(0x3002, 0x21); 
	OV56702ND_write_cmos_sensor(0x3005, 0xf0); 
	OV56702ND_write_cmos_sensor(0x3007, 0x00); 
	OV56702ND_write_cmos_sensor(0x3015, 0x0f); 
	OV56702ND_write_cmos_sensor(0x3018, 0x32); 
	OV56702ND_write_cmos_sensor(0x301a, 0xf0); 
	OV56702ND_write_cmos_sensor(0x301b, 0xf0); 
	OV56702ND_write_cmos_sensor(0x301c, 0xf0); 
	OV56702ND_write_cmos_sensor(0x301d, 0xf0); 
	OV56702ND_write_cmos_sensor(0x301e, 0xf0); 
	OV56702ND_write_cmos_sensor(0x3030, 0x00); 
	OV56702ND_write_cmos_sensor(0x3031, 0x0a); 
	OV56702ND_write_cmos_sensor(0x303c, 0xff); 
	OV56702ND_write_cmos_sensor(0x303e, 0xff); 
	OV56702ND_write_cmos_sensor(0x3040, 0xf0); 
	OV56702ND_write_cmos_sensor(0x3041, 0x00); 
	OV56702ND_write_cmos_sensor(0x3042, 0xf0); 
	OV56702ND_write_cmos_sensor(0x3106, 0x11); 
	OV56702ND_write_cmos_sensor(0x3500, 0x00); 
	OV56702ND_write_cmos_sensor(0x3501, 0x7b); 
	OV56702ND_write_cmos_sensor(0x3502, 0x00); 
	OV56702ND_write_cmos_sensor(0x3503, 0x04); 
	OV56702ND_write_cmos_sensor(0x3504, 0x03); 
	OV56702ND_write_cmos_sensor(0x3505, 0x83); 
	OV56702ND_write_cmos_sensor(0x3508, 0x07); 
	OV56702ND_write_cmos_sensor(0x3509, 0x80); 
	OV56702ND_write_cmos_sensor(0x350e, 0x04); 
	OV56702ND_write_cmos_sensor(0x350f, 0x00); 
	OV56702ND_write_cmos_sensor(0x3510, 0x00); 
	OV56702ND_write_cmos_sensor(0x3511, 0x02); 
	OV56702ND_write_cmos_sensor(0x3512, 0x00); 
	OV56702ND_write_cmos_sensor(0x3601, 0xc8); 
	OV56702ND_write_cmos_sensor(0x3610, 0x88); 
	OV56702ND_write_cmos_sensor(0x3612, 0x48); 
	OV56702ND_write_cmos_sensor(0x3614, 0x5b); 
	OV56702ND_write_cmos_sensor(0x3615, 0x96); 
	OV56702ND_write_cmos_sensor(0x3621, 0xd0); 
	OV56702ND_write_cmos_sensor(0x3622, 0x00); 
	OV56702ND_write_cmos_sensor(0x3623, 0x00); 
	OV56702ND_write_cmos_sensor(0x3633, 0x13); 
	OV56702ND_write_cmos_sensor(0x3634, 0x13); 
	OV56702ND_write_cmos_sensor(0x3635, 0x13); 
	OV56702ND_write_cmos_sensor(0x3636, 0x13); 
	OV56702ND_write_cmos_sensor(0x3645, 0x13); 
	OV56702ND_write_cmos_sensor(0x3646, 0x82); 
	OV56702ND_write_cmos_sensor(0x3650, 0x00); 
	OV56702ND_write_cmos_sensor(0x3652, 0xff); 
	OV56702ND_write_cmos_sensor(0x3655, 0x20); 
	OV56702ND_write_cmos_sensor(0x3656, 0xff); 
	OV56702ND_write_cmos_sensor(0x365a, 0xff); 
	OV56702ND_write_cmos_sensor(0x365e, 0xff); 
	OV56702ND_write_cmos_sensor(0x3668, 0x00); 
	OV56702ND_write_cmos_sensor(0x366a, 0x07); 
	OV56702ND_write_cmos_sensor(0x366e, 0x10); 
	OV56702ND_write_cmos_sensor(0x366d, 0x00); 
	OV56702ND_write_cmos_sensor(0x366f, 0x80); 
	OV56702ND_write_cmos_sensor(0x3700, 0x28); 
	OV56702ND_write_cmos_sensor(0x3701, 0x10); 
	OV56702ND_write_cmos_sensor(0x3702, 0x3a); 
	OV56702ND_write_cmos_sensor(0x3703, 0x19); 
	OV56702ND_write_cmos_sensor(0x3704, 0x10);
	OV56702ND_write_cmos_sensor(0x3705, 0x00); 
	OV56702ND_write_cmos_sensor(0x3706, 0x66); 
	OV56702ND_write_cmos_sensor(0x3707, 0x08); 
	OV56702ND_write_cmos_sensor(0x3708, 0x34); 
	OV56702ND_write_cmos_sensor(0x3709, 0x40); 
	OV56702ND_write_cmos_sensor(0x370a, 0x01); 
	OV56702ND_write_cmos_sensor(0x370b, 0x1b); 
	OV56702ND_write_cmos_sensor(0x3714, 0x24); 
	OV56702ND_write_cmos_sensor(0x371a, 0x3e); 
	OV56702ND_write_cmos_sensor(0x3733, 0x00); 
	OV56702ND_write_cmos_sensor(0x3734, 0x00); 
	OV56702ND_write_cmos_sensor(0x373a, 0x05); 
	OV56702ND_write_cmos_sensor(0x373b, 0x06); 
	OV56702ND_write_cmos_sensor(0x373c, 0x0a); 
	OV56702ND_write_cmos_sensor(0x373f, 0xa0); 
	OV56702ND_write_cmos_sensor(0x3755, 0x00); 
	OV56702ND_write_cmos_sensor(0x3758, 0x00); 
	OV56702ND_write_cmos_sensor(0x375b, 0x0e);
	OV56702ND_write_cmos_sensor(0x3766, 0x5f); 
	OV56702ND_write_cmos_sensor(0x3768, 0x00); 
	OV56702ND_write_cmos_sensor(0x3769, 0x22); 
	OV56702ND_write_cmos_sensor(0x3773, 0x08); 
	OV56702ND_write_cmos_sensor(0x3774, 0x1f); 
	OV56702ND_write_cmos_sensor(0x3776, 0x06); 
	OV56702ND_write_cmos_sensor(0x37a0, 0x88); 
	OV56702ND_write_cmos_sensor(0x37a1, 0x5c); 
	OV56702ND_write_cmos_sensor(0x37a7, 0x88); 
	OV56702ND_write_cmos_sensor(0x37a8, 0x70); 
	OV56702ND_write_cmos_sensor(0x37aa, 0x88); 
	OV56702ND_write_cmos_sensor(0x37ab, 0x48); 
	OV56702ND_write_cmos_sensor(0x37b3, 0x66); 
	OV56702ND_write_cmos_sensor(0x37c2, 0x04); 
	OV56702ND_write_cmos_sensor(0x37c5, 0x00); 
	OV56702ND_write_cmos_sensor(0x37c8, 0x00); 
	OV56702ND_write_cmos_sensor(0x3800, 0x00); 
	OV56702ND_write_cmos_sensor(0x3801, 0x0c); 
	OV56702ND_write_cmos_sensor(0x3802, 0x00); 
	OV56702ND_write_cmos_sensor(0x3803, 0x04); 
	OV56702ND_write_cmos_sensor(0x3804, 0x0a); 
	OV56702ND_write_cmos_sensor(0x3805, 0x33); 
	OV56702ND_write_cmos_sensor(0x3806, 0x07); 
	OV56702ND_write_cmos_sensor(0x3807, 0xa3); 
	OV56702ND_write_cmos_sensor(0x3808, 0x0a); 
	OV56702ND_write_cmos_sensor(0x3809, 0x20); 
	OV56702ND_write_cmos_sensor(0x380a, 0x07); 
	OV56702ND_write_cmos_sensor(0x380b, 0x98); 
	OV56702ND_write_cmos_sensor(0x380c, 0x07); // ;06
	OV56702ND_write_cmos_sensor(0x380d, 0xdc); // ;8c
	OV56702ND_write_cmos_sensor(0x380e, 0x07); 
	OV56702ND_write_cmos_sensor(0x380f, 0xb8); 
	OV56702ND_write_cmos_sensor(0x3811, 0x04); 
	OV56702ND_write_cmos_sensor(0x3813, 0x02); 
	OV56702ND_write_cmos_sensor(0x3814, 0x01); 
	OV56702ND_write_cmos_sensor(0x3815, 0x01); 
	OV56702ND_write_cmos_sensor(0x3816, 0x00); 
	OV56702ND_write_cmos_sensor(0x3817, 0x00); 
	OV56702ND_write_cmos_sensor(0x3818, 0x00); 
	OV56702ND_write_cmos_sensor(0x3819, 0x00); 
	OV56702ND_write_cmos_sensor(0x3820, 0x80); 
	OV56702ND_write_cmos_sensor(0x3821, 0x46); 
	OV56702ND_write_cmos_sensor(0x3822, 0x48); 
	OV56702ND_write_cmos_sensor(0x3826, 0x00); 
	OV56702ND_write_cmos_sensor(0x3827, 0x08); 
	OV56702ND_write_cmos_sensor(0x382a, 0x01); 
	OV56702ND_write_cmos_sensor(0x382b, 0x01); 
	OV56702ND_write_cmos_sensor(0x3830, 0x08); 
	OV56702ND_write_cmos_sensor(0x3836, 0x02); 
	OV56702ND_write_cmos_sensor(0x3837, 0x00); 
	OV56702ND_write_cmos_sensor(0x3838, 0x10); 
	OV56702ND_write_cmos_sensor(0x3841, 0xff); 
	OV56702ND_write_cmos_sensor(0x3846, 0x48); 
	OV56702ND_write_cmos_sensor(0x3861, 0x00); 
	OV56702ND_write_cmos_sensor(0x3862, 0x04);//x00); 
	OV56702ND_write_cmos_sensor(0x3863, 0x06);//0x18); 
	OV56702ND_write_cmos_sensor(0x3a11, 0x01); 
	OV56702ND_write_cmos_sensor(0x3a12, 0x78); 
	OV56702ND_write_cmos_sensor(0x3b00, 0x00); 
	OV56702ND_write_cmos_sensor(0x3b02, 0x00); 
	OV56702ND_write_cmos_sensor(0x3b03, 0x00); 
	OV56702ND_write_cmos_sensor(0x3b04, 0x00); 
	OV56702ND_write_cmos_sensor(0x3b05, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c00, 0x89); 
	OV56702ND_write_cmos_sensor(0x3c01, 0xab); 
	OV56702ND_write_cmos_sensor(0x3c02, 0x01); 
	OV56702ND_write_cmos_sensor(0x3c03, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c04, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c05, 0x03); 
	OV56702ND_write_cmos_sensor(0x3c06, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c07, 0x05); 
	OV56702ND_write_cmos_sensor(0x3c0c, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c0d, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c0e, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c0f, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c40, 0x00); 
	OV56702ND_write_cmos_sensor(0x3c41, 0xa3); 
	OV56702ND_write_cmos_sensor(0x3c43, 0x7d); 
	OV56702ND_write_cmos_sensor(0x3c45, 0xd7); 
	OV56702ND_write_cmos_sensor(0x3c47, 0xfc); 
	OV56702ND_write_cmos_sensor(0x3c50, 0x05); 
	OV56702ND_write_cmos_sensor(0x3c52, 0xaa); 
	OV56702ND_write_cmos_sensor(0x3c54, 0x71); 
	OV56702ND_write_cmos_sensor(0x3c56, 0x80); 
	OV56702ND_write_cmos_sensor(0x3d85, 0x17); 
	OV56702ND_write_cmos_sensor(0x3f03, 0x00); 
	OV56702ND_write_cmos_sensor(0x3f0a, 0x00); 
	OV56702ND_write_cmos_sensor(0x3f0b, 0x00); 
	OV56702ND_write_cmos_sensor(0x4001, 0x60); 
	OV56702ND_write_cmos_sensor(0x4009, 0x0d); 
	OV56702ND_write_cmos_sensor(0x4020, 0x00); 
	OV56702ND_write_cmos_sensor(0x4021, 0x00); 
	OV56702ND_write_cmos_sensor(0x4022, 0x00); 
	OV56702ND_write_cmos_sensor(0x4023, 0x00); 
	OV56702ND_write_cmos_sensor(0x4024, 0x00); 
	OV56702ND_write_cmos_sensor(0x4025, 0x00); 
	OV56702ND_write_cmos_sensor(0x4026, 0x00); 
	OV56702ND_write_cmos_sensor(0x4027, 0x00); 
	OV56702ND_write_cmos_sensor(0x4028, 0x00); 
	OV56702ND_write_cmos_sensor(0x4029, 0x00); 
	OV56702ND_write_cmos_sensor(0x402a, 0x00); 
	OV56702ND_write_cmos_sensor(0x402b, 0x00); 
	OV56702ND_write_cmos_sensor(0x402c, 0x00); 
	OV56702ND_write_cmos_sensor(0x402d, 0x00); 
	OV56702ND_write_cmos_sensor(0x402e, 0x00); 
	OV56702ND_write_cmos_sensor(0x402f, 0x00); 
	OV56702ND_write_cmos_sensor(0x4040, 0x00); 
	OV56702ND_write_cmos_sensor(0x4041, 0x03);//0x00); 
	OV56702ND_write_cmos_sensor(0x4042, 0x00); 
	OV56702ND_write_cmos_sensor(0x4043, 0x7a);//0x80); 
	OV56702ND_write_cmos_sensor(0x4044, 0x00); 
	OV56702ND_write_cmos_sensor(0x4045, 0x7a);//0x80); 
	OV56702ND_write_cmos_sensor(0x4046, 0x00); 
	OV56702ND_write_cmos_sensor(0x4047, 0x7a);//0x80); 
	OV56702ND_write_cmos_sensor(0x4048, 0x00); 
	OV56702ND_write_cmos_sensor(0x4049, 0x7a);//0x80); 
	OV56702ND_write_cmos_sensor(0x4303, 0x00); 
	OV56702ND_write_cmos_sensor(0x4307, 0x30); 
	OV56702ND_write_cmos_sensor(0x4500, 0x58); 
	OV56702ND_write_cmos_sensor(0x4501, 0x04); 
	OV56702ND_write_cmos_sensor(0x4502, 0x40); 
	OV56702ND_write_cmos_sensor(0x4503, 0x10); 
	OV56702ND_write_cmos_sensor(0x4508, 0xaa); 
	OV56702ND_write_cmos_sensor(0x4509, 0xaa); 
	OV56702ND_write_cmos_sensor(0x450a, 0x00); 
	OV56702ND_write_cmos_sensor(0x450b, 0x00); 
	OV56702ND_write_cmos_sensor(0x4600, 0x01); 
	OV56702ND_write_cmos_sensor(0x4601, 0x03); 
	OV56702ND_write_cmos_sensor(0x4700, 0xa4); 
	OV56702ND_write_cmos_sensor(0x4800, 0x4c); 
	OV56702ND_write_cmos_sensor(0x4816, 0x53); 
	OV56702ND_write_cmos_sensor(0x481f, 0x40); 
	OV56702ND_write_cmos_sensor(0x4837, 0x14); // ;11
	OV56702ND_write_cmos_sensor(0x5000, 0x56);//0x16); 
	OV56702ND_write_cmos_sensor(0x5001, 0x01); 
	OV56702ND_write_cmos_sensor(0x5002, 0x28);//0xa8); 
	OV56702ND_write_cmos_sensor(0x5004, 0x0c); 
	OV56702ND_write_cmos_sensor(0x5006, 0x0c); 
	OV56702ND_write_cmos_sensor(0x5007, 0xe0); 
	OV56702ND_write_cmos_sensor(0x5008, 0x01); 
	OV56702ND_write_cmos_sensor(0x5009, 0xb0); 
	OV56702ND_write_cmos_sensor(0x5901, 0x00); 
	OV56702ND_write_cmos_sensor(0x5a01, 0x00); 
	OV56702ND_write_cmos_sensor(0x5a03, 0x00); 
	OV56702ND_write_cmos_sensor(0x5a04, 0x0c); 
	OV56702ND_write_cmos_sensor(0x5a05, 0xe0); 
	OV56702ND_write_cmos_sensor(0x5a06, 0x09); 
	OV56702ND_write_cmos_sensor(0x5a07, 0xb0); 
	OV56702ND_write_cmos_sensor(0x5a08, 0x06); 
	OV56702ND_write_cmos_sensor(0x5e00, 0x00); 
	//for BLC
	OV56702ND_write_cmos_sensor(0x3734, 0x40); 
	OV56702ND_write_cmos_sensor(0x5b00, 0x01); 
	OV56702ND_write_cmos_sensor(0x5b01, 0x10); 
	OV56702ND_write_cmos_sensor(0x5b02, 0x01); 
	OV56702ND_write_cmos_sensor(0x5b03, 0xdb); 
	OV56702ND_write_cmos_sensor(0x3d8c, 0x71);
	OV56702ND_write_cmos_sensor(0x3d8d, 0xea);
	OV56702ND_write_cmos_sensor(0x4017, 0x08);
	
	OV56702ND_write_cmos_sensor(0x3618, 0x2a); 
								   
	//;Ally031414					  
	OV56702ND_write_cmos_sensor(0x3734, 0x40); //	;; Improve HFPN
	OV56702ND_write_cmos_sensor(0x5b00, 0x01);  // ;; [2:0] otp start addr[10:8]
	OV56702ND_write_cmos_sensor(0x5b01, 0x10);  // ;; [7:0] otp start addr[7:0]
	OV56702ND_write_cmos_sensor(0x5b02, 0x01);  // ;; [2:0] otp end addr[10:8]
	OV56702ND_write_cmos_sensor(0x5b03, 0xDB);  // ;; [7:0] otp end addr[7:0]
	OV56702ND_write_cmos_sensor(0x3d8c, 0x71); //; Header address high byte
	OV56702ND_write_cmos_sensor(0x3d8d, 0xEA); //; Header address low byte
	OV56702ND_write_cmos_sensor(0x4017, 0x08); // ; threshold= 2LSB for full size
								  
	//;Strong DPC1.53				 
	OV56702ND_write_cmos_sensor(0x5780, 0x3e); 
	OV56702ND_write_cmos_sensor(0x5781, 0x0f); 
	OV56702ND_write_cmos_sensor(0x5782, 0x44); 
	OV56702ND_write_cmos_sensor(0x5783, 0x02); 
	OV56702ND_write_cmos_sensor(0x5784, 0x01); 
	OV56702ND_write_cmos_sensor(0x5785, 0x00); 
	OV56702ND_write_cmos_sensor(0x5786, 0x00); 
	OV56702ND_write_cmos_sensor(0x5787, 0x04); 
	OV56702ND_write_cmos_sensor(0x5788, 0x02); 
	OV56702ND_write_cmos_sensor(0x5789, 0x0f); 
	OV56702ND_write_cmos_sensor(0x578a, 0xfd); 
	OV56702ND_write_cmos_sensor(0x578b, 0xf5); 
	OV56702ND_write_cmos_sensor(0x578c, 0xf5); 
	OV56702ND_write_cmos_sensor(0x578d, 0x03); 
	OV56702ND_write_cmos_sensor(0x578e, 0x08); 
	OV56702ND_write_cmos_sensor(0x578f, 0x0c); 
	OV56702ND_write_cmos_sensor(0x5790, 0x08); 
	OV56702ND_write_cmos_sensor(0x5791, 0x04); 
	OV56702ND_write_cmos_sensor(0x5792, 0x00); 
	OV56702ND_write_cmos_sensor(0x5793, 0x52); 
	OV56702ND_write_cmos_sensor(0x5794, 0xa3); 
	//;Ping 					  
	OV56702ND_write_cmos_sensor(0x380e, 0x07); //; fps fine adjustment
	OV56702ND_write_cmos_sensor(0x380f, 0xfd); //; fps fine adjustment
	OV56702ND_write_cmos_sensor(0x3503, 0x00); //; real gain [2]   gain no delay, shutter no delay
	//;added					 
	OV56702ND_write_cmos_sensor(0x3d85, 0x17); 
	OV56702ND_write_cmos_sensor(0x3655, 0x20); 
								   
	OV56702ND_write_cmos_sensor(0x0100, 0x01); //;01

  ov56702nd_otp_config();
}


UINT32 OV56702NDOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	OV56702NDDB("OV56702ND Open enter :\n ");
	OV56702ND_write_cmos_sensor(0x0103,0x01);// Reset sensor
  mdelay(2);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV56702ND_read_cmos_sensor(0x300B)<<8)|OV56702ND_read_cmos_sensor(0x300C);
		OV56702NDDB("OV56702ND READ ID :%x",sensor_id);
		if(sensor_id != OV5670MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV56702ND_Sensor_Init();
  OV56702ND_Init_Para();
	OV56702NDDB("OV56702NDOpen exit :\n ");

 return ERROR_NONE;
}

void OV56702NDGetModuleID(void)
{
	struct otp_struct otp_info ; 
	
	OV56702ND_write_cmos_sensor(0x0100, 0x01); //stream on
	read_otp(&otp_info);
	OV56702ND_write_cmos_sensor(0x0100, 0x00); //stream off
}

UINT32 OV56702NDGetSensorID(UINT32 *sensorID)
{
    int  retry = 2;

	OV56702NDDB("OV56702NDGetSensorID enter :\n ");
    mdelay(5);

    do {
        *sensorID = (OV56702ND_read_cmos_sensor(0x300B)<<8)|OV56702ND_read_cmos_sensor(0x300C);
        if (*sensorID == OV56702NDMIPI_SENSOR_ID)
	{
		OV56702NDDB("Sensor ID = 0x%04x\n", *sensorID);
		OV56702NDGetModuleID();
		break;
	}
	OV56702NDDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV56702NDMIPI_SENSOR_ID) {
		OV56702NDDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);

		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
    } else {
    		OV56702NDDB("alexadd %s module_id=%d", __func__, module_id);
		if (module_id != 0x44) {
			*sensorID = 0xFFFFFFFF;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
    }
    return ERROR_NONE;
}


void OV56702ND_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&ov56702ndmipiraw_drv_lock);
   OV56702ND.shutter= iShutter;
   spin_unlock(&ov56702ndmipiraw_drv_lock);

   OV56702ND_write_shutter(iShutter);
   return;
}



UINT32 OV56702ND_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV56702ND_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV56702ND_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV56702ND_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

void OV56702ND_NightMode(kal_bool bEnable)
{

}

UINT32 OV56702NDClose(void)
{

    return ERROR_NONE;
}

#if 0
void OV56702NDSetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	mirror= OV56702ND_read_cmos_sensor(0x3820);
	flip  = OV56702ND_read_cmos_sensor(0x3821);

    switch (imgMirror)
    {
        case IMAGE_H_MIRROR://IMAGE_NORMAL:
            OV56702ND_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set normal
            OV56702ND_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set normal
            break;
        case IMAGE_NORMAL://IMAGE_V_MIRROR:
            OV56702ND_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set flip
            OV56702ND_write_cmos_sensor(0x3821, (flip | (0x06)));	//Set flip
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            OV56702ND_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror
            OV56702ND_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
            OV56702ND_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror & flip
            OV56702ND_write_cmos_sensor(0x3821, (flip |(0x06)));	//Set mirror & flip
            break;
    }
}
#endif


UINT32 OV56702NDPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV56702NDDB("OV56702NDPreview enter:");

	//OV56702NDPreviewSetting();
	OV56702NDCaptureSetting();
	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND.sensorMode = SENSOR_MODE_PREVIEW; 
	OV56702ND.DummyPixels = 0;
	OV56702ND.DummyLines = 0 ;
	OV56702ND_FeatureControl_PERIOD_PixelNum=OV56702ND_PV_PERIOD_PIXEL_NUMS+ OV56702ND.DummyPixels;
	OV56702ND_FeatureControl_PERIOD_LineNum=OV56702ND_PV_PERIOD_LINE_NUMS+OV56702ND.DummyLines;
	OV56702ND.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov56702ndmipiraw_drv_lock);
	
	//OV56702NDSetFlipMirror(sensor_config_data->SensorImageMirror);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV56702NDDB("OV56702NDPreview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV56702NDVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV56702NDDB("OV56702NDVideo enter:");

	OV56702NDVideoSetting();

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND.sensorMode = SENSOR_MODE_VIDEO;
	OV56702ND_FeatureControl_PERIOD_PixelNum=OV56702ND_VIDEO_PERIOD_PIXEL_NUMS+ OV56702ND.DummyPixels;
	OV56702ND_FeatureControl_PERIOD_LineNum=OV56702ND_VIDEO_PERIOD_LINE_NUMS+OV56702ND.DummyLines;
	OV56702ND.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov56702ndmipiraw_drv_lock);
	
	//OV56702NDSetFlipMirror(sensor_config_data->SensorImageMirror);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV56702NDDB("OV56702NDVideo exit:\n");
    return ERROR_NONE;
}


UINT32 OV56702NDCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	//kal_uint32 shutter = OV56702ND.shutter;

	if( SENSOR_MODE_CAPTURE== OV56702ND.sensorMode)
	{
		OV56702NDDB("OV56702NDCapture BusrtShot / ZSD!!!\n");
	}
	else
	{
		OV56702NDDB("OV56702NDCapture enter:\n");

		OV56702NDCaptureSetting();
	    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY

		spin_lock(&ov56702ndmipiraw_drv_lock);
		OV56702ND.sensorMode = SENSOR_MODE_CAPTURE;
		OV56702ND.imgMirror = sensor_config_data->SensorImageMirror;
		OV56702ND.DummyPixels = 0;
		OV56702ND.DummyLines = 0 ;
		OV56702ND_FeatureControl_PERIOD_PixelNum = OV56702ND_FULL_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels;
		OV56702ND_FeatureControl_PERIOD_LineNum = OV56702ND_FULL_PERIOD_LINE_NUMS + OV56702ND.DummyLines;
		spin_unlock(&ov56702ndmipiraw_drv_lock);

		//OV56702NDSetFlipMirror(sensor_config_data->SensorImageMirror);

		OV56702NDDB("OV56702NDCapture exit:\n");
	}

	if(OV56702ND_During_testpattern == KAL_TRUE)
	{
		OV56702ND_write_cmos_sensor(0x4303,0x80);
	}

    return ERROR_NONE;
}	



UINT32 OV56702NDGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV56702NDDB("OV56702NDGetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV56702ND_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV56702ND_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV56702ND_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV56702ND_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV56702ND_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV56702ND_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV56702NDGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov56702ndmipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV56702ND_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV56702ND_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV56702ND_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV56702ND_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV56702ND_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV56702ND_FULL_Y_START;	

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV56702ND_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV56702ND_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV56702NDSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* OV56702NDGetInfo() */



UINT32 OV56702NDControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov56702ndmipiraw_drv_lock);
		OV56702NDCurrentScenarioId = ScenarioId;
		spin_unlock(&ov56702ndmipiraw_drv_lock);
		
		OV56702NDDB("OV56702NDCurrentScenarioId=%d\n",OV56702NDCurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV56702NDPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV56702NDDB("OV56702ND video_preiew sync\n");
			OV56702NDVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV56702NDCapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV56702NDControl() */



kal_uint32 OV56702ND_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{
    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(OV56702ND.OV56702NDAutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV56702ND_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV56702ND_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (OV56702ND.videoPclk*10000)/(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (OV56702ND.videoPclk*10000) /(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;

}



UINT32 OV56702NDSetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV56702NDDB("[OV56702NDSetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov56702ndmipiraw_drv_lock);
	OV56702ND_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov56702ndmipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV56702NDDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV56702NDDB("abmornal frame rate seting,pay attention~\n");

    if(OV56702ND.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV56702ND_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV56702ND_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV56702ND_VIDEO_PERIOD_LINE_NUMS;
			OV56702NDDB("[OV56702NDSetVideoMode]current fps = %d\n", (OV56702ND.videoPclk*10000)  /(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS)/OV56702ND_VIDEO_PERIOD_LINE_NUMS);
		}
		OV56702NDDB("[OV56702NDSetVideoMode]current fps (10 base)= %d\n", (OV56702ND.videoPclk*10000)*10/(OV56702ND_VIDEO_PERIOD_PIXEL_NUMS + OV56702ND.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV56702ND_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&ov56702ndmipiraw_drv_lock);
		OV56702ND.DummyPixels = 0;//define dummy pixels and lines
		OV56702ND.DummyLines = extralines ;
		spin_unlock(&ov56702ndmipiraw_drv_lock);
		
		OV56702ND_SetDummy(OV56702ND.DummyPixels,extralines);
    }
	
	OV56702NDDB("[OV56702NDSetVideoMode]MIN_Frame_length=%d,OV56702ND.DummyLines=%d\n",MIN_Frame_length,OV56702ND.DummyLines);

    return KAL_TRUE;
}


UINT32 OV56702NDSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

	if(bEnable) {   
		spin_lock(&ov56702ndmipiraw_drv_lock);
		OV56702ND.OV56702NDAutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov56702ndmipiraw_drv_lock);
        OV56702NDDB("OV56702ND Enable Auto flicker\n");
    } else {
    	spin_lock(&ov56702ndmipiraw_drv_lock);
        OV56702ND.OV56702NDAutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov56702ndmipiraw_drv_lock);
        OV56702NDDB("OV56702ND Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV56702NDSetTestPatternMode(kal_bool bEnable)
{
    OV56702NDDB("[OV56702NDSetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV56702ND_During_testpattern = KAL_TRUE;

		//OV56702ND_write_cmos_sensor(0x5000,0x16);// ; LENC off, MWB on, BPC on, WPC on
		
		OV56702ND_write_cmos_sensor(0x4303,0x08);
    }
	else
	{
        OV56702ND_During_testpattern = KAL_FALSE;
		//OV56702ND_write_cmos_sensor(0x5000,0x96);// ; LENC on, MWB on, BPC on, WPC on
		OV56702ND_write_cmos_sensor(0x4303,0x00);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV56702NDMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV56702NDDB("OV56702NDMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV56702ND_PREVIEW_PCLK;
			lineLength = OV56702ND_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV56702ND_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov56702ndmipiraw_drv_lock);
			OV56702ND.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&ov56702ndmipiraw_drv_lock);
			OV56702ND_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV56702ND_VIDEO_PCLK;
			lineLength = OV56702ND_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV56702ND_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov56702ndmipiraw_drv_lock);
			OV56702ND.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&ov56702ndmipiraw_drv_lock);
			OV56702ND_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV56702ND_CAPTURE_PCLK;
			lineLength = OV56702ND_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV56702ND_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov56702ndmipiraw_drv_lock);
			OV56702ND.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&ov56702ndmipiraw_drv_lock);
			OV56702ND_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


UINT32 OV56702NDMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV56702ND_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV56702ND_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV56702ND_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

/* Sensor output window information */

/* SZ TCT xuejian.zhong add for CTS test*/

static void OV56702NDGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
   // SENSORDB("OV56702NDGetAFMaxNumFocusAreas *pFeatureReturnPara32 = %d¥n",  *pFeatureReturnPara32);
}

static void OV56702NDGetAEMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{     
    *pFeatureReturnPara32 = 0;    
  //  SENSORDB("OV56702NDGetAEMaxNumMeteringAreas *pFeatureReturnPara32 = %d¥n",  *pFeatureReturnPara32);	
}

static void OV56702NDGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = AWB_MODE_AUTO;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}

/* SZ TCT xuejian.zhong  end */


UINT32 OV56702NDFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV56702ND_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV56702ND_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV56702ND_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV56702ND_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV56702NDCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV56702ND_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV56702ND_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV56702ND_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV56702ND_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV56702ND_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV56702ND_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           OV56702ND_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV56702ND_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV56702ND_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV56702ND_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov56702ndmipiraw_drv_lock);
                OV56702NDSensorCCT[i].Addr=*pFeatureData32++;
                OV56702NDSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov56702ndmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV56702NDSensorCCT[i].Addr;
                *pFeatureData32++=OV56702NDSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov56702ndmipiraw_drv_lock);
                OV56702NDSensorReg[i].Addr=*pFeatureData32++;
                OV56702NDSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov56702ndmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV56702NDSensorReg[i].Addr;
                *pFeatureData32++=OV56702NDSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV56702NDMIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV56702NDSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV56702NDSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV56702NDSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV56702ND_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV56702ND_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV56702ND_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV56702ND_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV56702ND_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV56702ND_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            OV56702NDSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV56702NDGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            OV56702NDSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV56702NDMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV56702NDMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			OV56702NDSetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV56702ND_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
			/*SZ TCT xuejian.zhong add for CTS test */
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			OV56702NDGetAFMaxNumFocusAreas(pFeatureData32);
			*pFeatureParaLen=4;							 
			break;

		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			OV56702NDGetAEMaxNumMeteringAreas(pFeatureData32);
			*pFeatureParaLen=4;							 
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
		//	SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO¥n");
		//	SENSORDB("EXIF addr = 0x%x¥n",*pFeatureData32);

			OV56702NDGetExifInfo(*pFeatureData32);
			break;
			/* xuejian.zhong add end */
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV56702ND=
{
    OV56702NDOpen,
    OV56702NDGetInfo,
    OV56702NDGetResolution,
    OV56702NDFeatureControl,
    OV56702NDControl,
    OV56702NDClose
};

UINT32 OV56702ND_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV56702ND;

    return ERROR_NONE;
}  
