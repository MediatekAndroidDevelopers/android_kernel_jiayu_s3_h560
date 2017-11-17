#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include <mach/upmu_common.h>

#include <mach/mt_gpio.h>		// For gpio control

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/*
0  1  2  3   4   5   6   7   8   9   10  11  12  13
25 50 75 100 125 150 300 400 500 600 700 800 900 1000
*/

/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
	#define logI PK_DBG_FUNC
	#define logE(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);
static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define LM3646_REG_ENABLE       	0x01
#define LM3646_REG_IVFM             0x02
#define LM3646_REG_CURRENT_MAX  	0x05
#define LM3646_REG_CURRENT_LED 		0x06
#define LM3646_REG_CURRENT_TORCH 	0x07
#define LM3646_REG_TIMING         	0x04
#define LM3646_REG_FLAG         	0x09

#define LM3646_DUTY_MAX        15 
#define LM3646_MIN_FLASH_DUTY   2

#define LM3646_GPIO_CTRL        0

#ifndef GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_CAMERA_FLASH_EN_PIN		 (GPIO127 | 0x80000000)
#endif
#define GPIO_TORCH_EN        (GPIO125 | 0x80000000)
#define GPIO_FLASH_EN        (GPIO124 | 0x80000000)

static int flashCur[] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
static int torchCur[] = { 0x07, 0x18, 0x29, 0x38, 0x48 };

static int g_bLtVersion = 0;
static int g_bOpen=1;

static struct work_struct workTimeOut;
static struct work_struct workWDReset;
/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C_fl(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C_fl(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

static void work_timeOutFunc(struct work_struct *data);
#define STROBE_DEVICE_ID 0xCE
//static DEFINE_MUTEX(lm3646_lock);


int FL_ReadReg(int reg)
{
  char buf[2];
  char bufR[2];
  buf[0]=reg;
//  mutex_lock(&lm3646_lock);
  iReadRegI2C_fl(buf , 1, bufR,1, STROBE_DEVICE_ID);
//  mutex_unlock(&lm3646_lock);
  PK_DBG("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
  return (int)bufR[0];
}

int FL_WriteReg(int reg, int data)
{
  char buf[2];
  buf[0]=reg;
  buf[1]=data;
 // mutex_lock(&lm3646_lock);
  iWriteRegI2C_fl(buf, 2, STROBE_DEVICE_ID);
 // mutex_unlock(&lm3646_lock);
  return 0;
}
int FL_Enable()
{
	PK_DBG("[LM3646]FL_Enable g_duty=%d,",
           g_duty);
    int n1 = 0;
    int nMax = 0;
    if( -1 == g_duty)
    {
        PK_DBG("[LM3646]FL_Enable error!!! both duty&dutyLt is -1\n");
        return -1;
    }

	//FL_WriteReg(LM3646_REG_TIMING, 0x47);	
    PK_DBG("[LM3646]FL_Enable single flash, \n");
    nMax = g_duty;
    if( nMax < LM3646_MIN_FLASH_DUTY )//torch
    {
      nMax = (nMax+1)*4-1;
      n1 = 63;
      FL_WriteReg(LM3646_REG_CURRENT_MAX,nMax<<4);
      FL_WriteReg(LM3646_REG_CURRENT_TORCH,n1);
      FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//10 set touch mode
      PK_DBG("[LM3646]FL_Enable torch Max=%d, led=%d\n", nMax, n1);
    }
    else
    {
      //nMax = 6;
      n1 = 63;
      FL_WriteReg(LM3646_REG_CURRENT_MAX,nMax);
      FL_WriteReg(LM3646_REG_CURRENT_LED,n1);	
      FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//100 0011 enable tx pin, set flash mode
      PK_DBG("[LM3646]FL_Enable flash Max=%d, led1=%d\n", nMax, n1);
    }

    return 0;
}

int FL_Disable(void)
{
	PK_DBG("[LM3646]FL_Disable \n");

    FL_WriteReg(LM3646_REG_ENABLE, 0x00);
    FL_WriteReg(LM3646_REG_CURRENT_TORCH,0x00);
    FL_WriteReg(LM3646_REG_CURRENT_LED,0x00);	
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("[LM3646]FL_dim_duty, duty=%d, \n", duty);

    duty = (duty>LM3646_DUTY_MAX)?LM3646_DUTY_MAX:duty;
  //if(duty > 8)
    //FL_WriteReg(LM3646_REG_TIMING, 0x7f);	
        g_duty=duty;
    return 0;
}

static int FL_getPreOnTime(int duty)
{
  if(duty>5)
    return 50;
  else
    return -1;

}

int FL_preOn()
{
	PK_DBG("[LM3646]FL_PreOn\n");
    return FL_Enable();
}

int FL_Init(void)
{
	PK_DBG("[LM3646]FL_Init\n");

	if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_MODE_00)){PK_DBG("[LM3646][constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ONE)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
/*
	if(mt_set_gpio_mode(GPIO_FLASH_EN,GPIO_MODE_00)){PK_DBG("[LM3646][constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ONE)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}

	if(mt_set_gpio_mode(GPIO_TORCH_EN,GPIO_MODE_00)){PK_DBG("[LM3646][constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_TORCH_EN, GPIO_OUT_ONE)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
	PK_DBG("[LM3646]Current state1:%d, state2:%d\n", FL_ReadReg(LM3646_REG_FLAG1), FL_ReadReg(LM3646_REG_FLAG2));
*/
    FL_WriteReg(LM3646_REG_ENABLE, 0x00);
    FL_WriteReg(LM3646_REG_IVFM, 0x80);
	FL_WriteReg(LM3646_REG_TIMING, 0x47);	
	FL_WriteReg(LM3646_REG_FLAG, 0x00);	
//    FL_WriteReg(LM3646_REG_TOUCHTIMER, 0x00);



    return 0;
}

int FL_Uninit(void)
{
	PK_DBG("[LM3646]FL_Uninit");

	FL_Disable();
    g_duty=0;

	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ZERO)){PK_DBG("[LM3646][constant_flashlight] set gpio dir failed!! \n");}
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;
static struct hrtimer g_WDResetTimer;
static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
    PK_DBG("[LM3646]ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	PK_DBG("[LM3646]ledTimeOut_callback\n");
	schedule_work(&workTimeOut);

    return HRTIMER_NORESTART;
}


static enum hrtimer_restart ledWDResetCallback(struct hrtimer *timer)
{
    schedule_work(&workWDReset);
    return HRTIMER_NORESTART;
}


static void timerInit(void)
{
    INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;
}
static int gGetPreOnDuty=0;
static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
    int temp;
	int i4RetValue = 0;
	int iFlashType = (int)FLASHLIGHT_NONE;
	int ior;
	int iow;
	int iowr;

	ior = _IOR(FLASHLIGHT_MAGIC,0, int);
	iow = _IOW(FLASHLIGHT_MAGIC,0, int);
	iowr = _IOWR(FLASHLIGHT_MAGIC,0, int);
	PK_DBG("[LM3646]constant_flashlight_ioctl() line=%d cmd=%d, ior=%d, iow=%d iowr=%d arg=%d\n",__LINE__, cmd, ior, iow, iowr, arg);
	PK_DBG("[LM3646]constant_flashlight_ioctl() line=%d cmd-ior=%d, cmd-iow=%d cmd-iowr=%d arg=%d\n",__LINE__, cmd-ior, cmd-iow, cmd-iowr, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("[LM3646]FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
			break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("[LM3646]FLASHLIGHT_DUTY: %d\n",arg);
            //g_duty=arg;
    		FL_dim_duty(arg);
    		break;
        case FLASH_IOC_SET_STEP:
            PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

            break;

#if 0
        case FLASH_IOC_PRE_ON:
            PK_DBG("FLASH_IOC_PRE_ON\n");
            FL_preOn(0);
            break;
        case FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY:
            PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY: %d\n",arg);
            gGetPreOnDuty = arg;
            break;
        case FLASH_IOC_GET_PRE_ON_TIME_MS:
            PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",arg);
            temp = FL_getPreOnTime(gGetPreOnDuty);
            if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
              PK_DBG(" ioctl copy to user failed\n");
              return -1;
            }
            break;

        case FLASH_IOC_GET_FLASH_DRIVER_NAME_ID:
            PK_DBG("FLASH_IOC_GET_FLASH_DRIVER_NAME_ID: %d\n",arg);
            temp = e_FLASH_DRIVER_6332;
            if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
              PK_DBG(" ioctl copy to user failed\n");
              return -1;
            }
            break;
#endif

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("[LM3646]FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
              if(g_timeOutTimeMs!=0)
              {
                ktime_t ktime;
                ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
              }
    			FL_Enable();
    		}
    		else
    		{
              FL_Disable();
              hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
#if 1
        case FLASHLIGHTIOC_G_FLASHTYPE:
            iFlashType = FLASHLIGHT_LED_CONSTANT;
            if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
            {
              PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
              return -EFAULT;
            }
           break;
#endif
		default :
    		PK_DBG("[LM3646] No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("[LM3646]constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("[LM3646]constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

    PK_ERR("[LM3646]constant_flashlight_open strobe_Res=%d\n", strobe_Res);
    strobe_Res++;

    g_bOpen=1;
    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("[LM3646]constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}

static int constant_flashlight_release(void *pArg)
{
    PK_DBG("[LM3646] constant_flashlight_release\n");

    if (1==strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);
        g_bOpen=0;

        strobe_Res = 0;

        /* LED On Status */

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }
    else
    {
        strobe_Res--;
    }

    PK_DBG("[LM3646] Done, strobe_Res=%d\n", strobe_Res);

    return 0;

}


static FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);
