#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <mach/mt_spi.h>

#include <linux/sched.h>
//#include "mt_spi_hal.h"
#include <linux/kthread.h>

#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/spi/spidev.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>//copy_from_user
#include <mach/mt_gpio.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_pwm.h>

#include "ice40.h"


//#include "../leds/leds_drv.h"
//#include <mach/battery_common.h>


#define LATTICE_SPIDEV_LOG(fmt, args...) printk("[lattice]:" fmt"\n", ##args) 
#define LATTICE_SPIDEV_ERR(fmt, args...) printk(KERN_ERR"[lattice]" fmt"\n", ##args )


#define DMA_MODE 1
#define FIF0_MODE 0

#define GPIO_ICE40_RST_PIN (GPIO61 | 0x80000000)
#define GPIO_ICE40_CDONE_PIN (GPIO62 | 0x80000000)


#define GPIO_SPI_CS_PIN         (GPIO144 | 0x80000000)
#define GPIO_SPI_CS_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_SPI_CS_PIN_M_SPI_CS   GPIO_MODE_01

#define GPIO_SPI_SCK_PIN         (GPIO145 | 0x80000000)
#define GPIO_SPI_SCK_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_SPI_SCK_PIN_M_SPI_CK   GPIO_MODE_01

#define GPIO_SPI_MISO_PIN         (GPIO146 | 0x80000000)
#define GPIO_SPI_MISO_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_SPI_MISO_PIN_M_SPI_MI   GPIO_MODE_01

#define GPIO_SPI_MOSI_PIN         (GPIO147 | 0x80000000)
#define GPIO_SPI_MOSI_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_SPI_MOSI_PIN_M_SPI_MO   GPIO_MODE_01

#define GPIO_IR_LDO1V2         (GPIO60 | 0x80000000)
#define GPIO_IR_LDO1V2_M_GPIO   GPIO_MODE_00

#define GPIO_IR_LDO1V8         (GPIO92 | 0x80000000)
#define GPIO_IR_LDO1V8_M_GPIO   GPIO_MODE_00

#define GPIO_IR_LDO2V8         (GPIO91 | 0x80000000)
#define GPIO_IR_LDO2V8_M_GPIO   GPIO_MODE_00

#define GPIO_IR_CLKM2_26M         (GPIO32 | 0x80000000)
#define GPIO_IR_CLKM2_26M_M_GPIO   GPIO_MODE_00
#define GPIO_IR_CLKM2_26M_M_PWM   GPIO_MODE_01
#define GPIO_IR_CLKM2_26M_CLK     CLK_OUT4
#define GPIO_IR_CLKM2_26M_FREQ    GPIO_CLKSRC_NONE


#define ICE40_BIN_PATH  "/system/usr/ice40.bin"
#define ICE40_IMAGE_BUFSIZE  (72 * 1024)

#define ICE40_IOCTL_CMD_META_CHIP_CHECK  0X11111111
#define ICE40_IOCTL_CMD_DOWNLOAD  0X22222222
#define ICE40_IOCTL_CMD_POWER_CTRL  _IOW(SPI_IOC_MAGIC, 10, __u32)

struct bin_info{
    int len;
    char *buf;
};
static struct bin_info ice40_bin_info;
static unsigned char *ice40_tx_buf = NULL;
static unsigned char *ice40_rx_buf = NULL;
static unsigned char *ice40_image_buf = NULL;
static int ice40_image_size = 0;
static int ice40_image_download_success = 0;
static int ice40_power_state = 0;

static struct spi_transfer tr[2];
static struct spi_device *ice40_spi_client = NULL;
static struct mt_chip_conf spi_conf;
static struct mt_chip_conf *spi_par = NULL;

static struct timer_list   timer_led;
static int led_state = 0;
static struct workqueue_struct *led_wqueue;
static struct delayed_work led_work;

static int ice40_spi_dma_xfer(unsigned char *txbuf,unsigned char *rxbuf, int len);
static int ice40_set_gpio_spi_mode(int enable);
static int ice40_set_com_mode(int mode);
static int ice40_set_spi_mode(int spi_mode);
static int ice40_set_clock_freq(int freq);
static int ice40_set_gpio_lowpower_mode(void);
static int ice40_set_gpio_run_mode(void);

static void ice40_power_control(int power)
{

  struct pwm_spec_config pwm_setting;
  pwm_setting.pwm_no = 2;
  pwm_setting.mode =PWM_MODE_OLD; 
  //pwm_setting.mode = PWM_MODE_FIFO;
  pwm_setting.pmic_pad = 0;
  pwm_setting.clk_div =0; 
  pwm_setting.clk_src =PWM_CLK_OLD_MODE_BLOCK; 
  //pwm_setting.clk_src =PWM_CLK_NEW_MODE_BLOCK; 
  pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
  pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
  pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
  pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
  pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 1; // 256 level
  pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;



    if(power)
    {
        mt_set_gpio_mode(GPIO_IR_LDO1V2, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO1V2, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO1V2, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO1V2, 1);

        mdelay(1);
        mt_set_gpio_mode(GPIO_IR_LDO1V8, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO1V8, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO1V8, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO1V8, 1);
        mdelay(1);

        mt_set_gpio_mode(GPIO_IR_LDO2V8, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO2V8, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO2V8, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO2V8, 1);

       mt_set_gpio_mode(GPIO_IR_CLKM2_26M, GPIO_IR_CLKM2_26M_M_PWM);
       //clk_monitor(4,1,0);
       pwm_set_spec_config(&pwm_setting);



    }
    else
    {
          mt_pwm_disable(2, 0);
          mt_set_gpio_mode(GPIO_IR_CLKM2_26M, GPIO_MODE_GPIO);
          mt_set_gpio_dir(GPIO_IR_CLKM2_26M, GPIO_DIR_IN);
          mt_set_gpio_pull_enable(GPIO_IR_CLKM2_26M, GPIO_PULL_ENABLE);
          mt_set_gpio_pull_select(GPIO_IR_CLKM2_26M, GPIO_PULL_DOWN);


        mt_set_gpio_mode(GPIO_IR_LDO1V2, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO1V2, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO1V2, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO1V2, 0);

        //mdelay(1);
        mt_set_gpio_mode(GPIO_IR_LDO1V8, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO1V8, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO1V8, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO1V8, 0);
        //mdelay(1);

        mt_set_gpio_mode(GPIO_IR_LDO2V8, GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_IR_LDO2V8, GPIO_DIR_OUT);
        //mt_set_gpio_pull_enable(GPIO_IR_LDO2V8, GPIO_PULL_ENABLE);
        mt_set_gpio_out(GPIO_IR_LDO2V8, 0);
    }
}


static int ice40_handshake(void)
{
    u8 send_buf[4] = {2, 0, 0, 0xaa};
    u8 read_cmd[4] = {3, 0, 0, 0};
    u8 recv_buf[4] = {0};

    //发
    ice40_spi_dma_xfer(send_buf, NULL, 4);

    //收
    ice40_spi_dma_xfer(read_cmd, recv_buf, 4);
    LATTICE_SPIDEV_LOG("recv_buf[3]=0X%X", recv_buf[3]);

    if(send_buf[3] == recv_buf[3])
    {
        LATTICE_SPIDEV_LOG("%s, ok", __func__);
        return 0;
    }
    else
    {
        LATTICE_SPIDEV_LOG("%s, fail", __func__);
        return -1;
    }
}
//read bin in kernel
static int ice40_read_image_in_kernel(char *path, char * buf, int *bin_size)
{
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;
    int image_size;

    LATTICE_SPIDEV_LOG("%s in", __func__);
    fp = filp_open(path, O_RDONLY, 0666);
    //fp = filp_open(ICE40_BIN_PATH, O_RDONLY, 0666);
    if(IS_ERR(fp))
    {
        LATTICE_SPIDEV_LOG("filp_open fail");
        return -1;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);

    pos = 0;
    image_size = vfs_read(fp, buf, ICE40_IMAGE_BUFSIZE, &pos);
    LATTICE_SPIDEV_LOG("%s, image_size=%d", __func__, image_size);
    *bin_size = image_size;

    filp_close(fp, NULL);
    set_fs(fs);

    return 0;
}

static int ice40_download_bin(unsigned char *image, int image_size)
{
    int dma_tx_len;//1024 MUL
    
    //set spi_cs_pin to GPIO_mode
    mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_MODE_00);
    //mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
    mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SPI_CS_PIN, 0);

    //reset
    mt_set_gpio_out(GPIO_ICE40_RST_PIN, 0);
    ndelay(300);//300 ns
    mt_set_gpio_out(GPIO_ICE40_RST_PIN, 1);
 
    udelay(800);//800 us
    
    //SET pin to spi_mode
    ice40_set_gpio_spi_mode(1);
    //use DMA to download bin
    //ice40_set_com_mode(DMA_MODE);
    ice40_set_spi_mode(3);

    //at least +100 clk wait time
    dma_tx_len = image_size + 32;
    LATTICE_SPIDEV_LOG("image_size=%d, dma_tx_len=%d", image_size, dma_tx_len);
    ice40_spi_dma_xfer(image, NULL, dma_tx_len);
    
        
    if(mt_get_gpio_in(GPIO_ICE40_CDONE_PIN))
    {
        mdelay(10);
        if(0 == ice40_handshake())
        {
            LATTICE_SPIDEV_LOG("image download ok");
            return 0;
        }
        else
        {
            LATTICE_SPIDEV_LOG("image download fail");
            return -1;
        }
    }
    else
    {
        LATTICE_SPIDEV_LOG("image download fail,CDONE LEVEL 0");
        return -1;
    }

    return 0;
}
static ssize_t download_test_store(struct device *dev, 
	struct device_attribute *attr, 
	const char *buf, size_t count)
{
    unsigned char enable;
    sscanf(buf, "%d", &enable);
    if(enable)
    {
        
    }
    else
    {}
    return count;
}

static ssize_t download_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char enable, count = 0;

    LATTICE_SPIDEV_LOG("%s in", __func__);
    count = snprintf(buf, PAGE_SIZE, "ice40_image_download_success=%d\n", ice40_image_download_success);
    return count;
}
static DEVICE_ATTR(download, 0660, download_test_show, download_test_store);

static ssize_t power_state_store(struct device *dev, 
	struct device_attribute *attr, 
	const char *buf, size_t count)
{
    unsigned char enable = 0;
    sscanf(buf, "%d", &enable);

    LATTICE_SPIDEV_LOG("%s in, enable=%d", __func__, enable);
    if(enable)
    {
        ice40_power_control(1);
        
        mdelay(1);

        ice40_power_state = 1;
        ice40_set_gpio_run_mode();
        
        if(0 == ice40_read_image_in_kernel(ICE40_BIN_PATH, ice40_image_buf, &ice40_image_size))
        {
            ice40_download_bin(ice40_image_buf, ice40_image_size);
        }
    }
    else//power off
    {
        ice40_power_state = 0;

        ice40_power_control(0);

        ice40_set_gpio_lowpower_mode();
    }
    return count;
}

static ssize_t power_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char enable, count = 0;

    LATTICE_SPIDEV_LOG("%s in", __func__);
    count = snprintf(buf, PAGE_SIZE, "ice40_power_state=%d\n", ice40_power_state);

    return count;
}
static DEVICE_ATTR(chip_power, 0660, power_state_show, power_state_store);

static struct device_attribute *spi_attribute[]={
	&dev_attr_download,
	&dev_attr_chip_power,
};

static int ice40_create_attribute(struct device *dev)
{
	int num,idx;
	int err =0;
	num = (int)(sizeof(spi_attribute)/sizeof(spi_attribute[0]));

	for (idx = 0; idx < num; idx ++) {
		if ((err = device_create_file(dev, spi_attribute[idx])))
			break;
	}
	return err;
	
}

static void ice40_remove_attribute(struct device *dev)
{
	int num, idx;
	num = (int)(sizeof(spi_attribute)/sizeof(spi_attribute[0]));

	for (idx = 0; idx < num; idx ++) {
		device_remove_file(dev, spi_attribute[idx]);
	}

	return;
}
static int ice40_set_gpio_spi_mode(int enable)
{
    
    if(enable)
    {
        mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CS);
        mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
        
        mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_SPI_CK);
        mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN, GPIO_PULL_DOWN);
        
        mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_SPI_MI);           
        mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_DOWN);
        
        mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_SPI_MO);           
        mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN, GPIO_PULL_DOWN);
                    
    }
    else
    {
        mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_DISABLE);
        
        mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_DISABLE);
        
        mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_DISABLE);
        
        mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_DISABLE);
    }
    
    return 0;
}

//config gpio low power mode
static int ice40_set_gpio_lowpower_mode(void)
{
    mt_set_gpio_dir(GPIO_ICE40_RST_PIN, GPIO_DIR_IN);
    
    mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_DOWN);

    
    mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN, GPIO_PULL_DOWN);
    
    mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_DOWN);
    
    mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN, GPIO_PULL_DOWN);
    return 0;
}
static int ice40_set_gpio_run_mode(void)
{
    mt_set_gpio_mode(GPIO_ICE40_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_ICE40_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_ICE40_RST_PIN, 1);

    ice40_set_gpio_spi_mode(1);
    return 0;
}


static int ice40_set_spi_mode(int spi_mode)
{
    int err; 
    if(spi_mode == 0)
    {
        ice40_spi_client->mode = SPI_MODE_0;
    }
    else if(spi_mode == 3)
    {
        ice40_spi_client->mode = SPI_MODE_3;
    }
    
    if(err = spi_setup(ice40_spi_client))
    {
        LATTICE_SPIDEV_ERR("spi_setup fail");
        return -1;
    }
    return 0;
}
static int ice40_set_clock_freq(int freq)
{
    int err;
    ice40_spi_client->controller_data = (void*)&spi_conf;
    spi_par =&spi_conf;
    
    if(!spi_par)
    {
        LATTICE_SPIDEV_ERR("spi par is NULL");
        return -1;
    }
    if(5000000 == freq)//5M HZ
    {
        spi_par->setuptime = 15;
        spi_par->holdtime = 15;
        spi_par->high_time = 10;  //   与low_time  共同决定着SPI CLK 的周期
        spi_par->low_time = 10;
        spi_par->cs_idletime = 20;
    }
    else if(1000000 == freq)// 1M HZ
    {
        spi_par->setuptime = 100;//15;
        spi_par->holdtime = 100;//15;
        spi_par->high_time = 50;//10;  //   与low_time  共同决定着SPI CLK 的周期
        spi_par->low_time = 50;//10;
        spi_par->cs_idletime = 100;//20;
    }
    
    if(err = spi_setup(ice40_spi_client))
    {
        LATTICE_SPIDEV_ERR("spi_setup fail");
        return -1;
    }

    return 0;
       
}       
//DMA OR FIFO com mode,1-DMA; 0-FIFO
static int ice40_set_com_mode(int mode)
{
    int err;
    ice40_spi_client->controller_data = (void*)&spi_conf;
    spi_par =&spi_conf;
    
    if(!spi_par)
    {
        LATTICE_SPIDEV_ERR("spi par is NULL");
        return -1;
    }
    if(DMA_MODE == mode)
    {
        spi_par->com_mod = DMA_TRANSFER;     // DMA  or FIFO
    }
    else
    {
        spi_par->com_mod = FIFO_TRANSFER;     // DMA  or FIFO
    }
    
    if(err = spi_setup(ice40_spi_client))
    {
        LATTICE_SPIDEV_ERR("spi_setup fail");
        return -1;
    }

    return 0;
}
//DMA mode
static int ice40_spi_dma_xfer(unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	struct spi_message msg;
	spi_message_init(&msg);

	int const pkt_count = len / 1024;
	int const remainder = len % 1024;

	LATTICE_SPIDEV_LOG("len=%d, txbuf=0x%p,rxbuf=0x%p",len,txbuf,rxbuf);

	if(len > 1024){	
		tr[0].tx_buf =(txbuf==NULL)?NULL: txbuf;
		tr[0].rx_buf =(rxbuf==NULL)?NULL: rxbuf;
		tr[0].len = 1024 * pkt_count;
		spi_message_add_tail(&tr[0], &msg);

		if(0 != remainder)	 { 
			tr[1].tx_buf =(txbuf==NULL)?NULL:txbuf+ (1024 * pkt_count);
			tr[1].rx_buf =(rxbuf==NULL)?NULL:rxbuf+ (1024 * pkt_count);
			tr[1].len = remainder;
			spi_message_add_tail(&tr[1], &msg);
		}
	}
	else{
		tr[0].tx_buf =(txbuf==NULL)?NULL: txbuf;
		tr[0].rx_buf =(rxbuf==NULL)?NULL: rxbuf;
		tr[0].len = len;
		spi_message_add_tail(&tr[0], &msg);
	}
	
	if(spi_sync(ice40_spi_client,&msg))
		return -1;	
	else
		return 0;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ice40_open(struct inode *inode, struct file *file)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static int ice40_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long ice40_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    struct spi_ioc_transfer transfer[2];
    int err, i;
    void __user *ptr = (void __user*) arg;
    char *log_buf;
    int power_state;
    
    LATTICE_SPIDEV_LOG("%s, cmd=0X%X", __func__, cmd);
    switch(cmd)
    {
        //apk write cmd
        case SPI_IOC_MESSAGE(1):
        LATTICE_SPIDEV_LOG("ioctl write");
        if(err = copy_from_user(&transfer, ptr, sizeof(transfer[0])))
        {
            LATTICE_SPIDEV_ERR("copy_from_user fail");
            return -1;
        }
        LATTICE_SPIDEV_LOG("write cmd len=%d", transfer[0].len);
        log_buf = (char *)transfer[0].tx_buf;
        /*
        printk("write:");
        for(i = 0; i < 20 && i < transfer[0].len; i ++)
        { 
            printk("[%d]=%X,", i, log_buf[i]);
        }
        printk("\n");
        */
        
        if(err = copy_from_user(ice40_tx_buf, transfer[0].tx_buf, transfer[0].len))
        {
            LATTICE_SPIDEV_ERR("copy_from_user fail");
            return -1;
        }
        LATTICE_SPIDEV_LOG("ice40_tx_buf=%X, %X, %X, %X", ice40_tx_buf[0], ice40_tx_buf[1], ice40_tx_buf[2], ice40_tx_buf[3]);

        //ice40_set_spi_mode(0);
        if(err = ice40_spi_dma_xfer(ice40_tx_buf, NULL, transfer[0].len))
        {
            LATTICE_SPIDEV_ERR("%s, spi transfer err", __func__);
            return -1;
        }

        break;


        //read ice40 data
        case SPI_IOC_MESSAGE(2):
        LATTICE_SPIDEV_LOG("ioctl read");
        if(err = copy_from_user(&transfer, ptr, 2 * sizeof(transfer[0])))
        {
            LATTICE_SPIDEV_ERR("copy_from_user fail");
            return -1;
        }
        LATTICE_SPIDEV_LOG("write cmd len=%d, read len=%d", transfer[0].len, transfer[1].len);
        log_buf = (char *)transfer[0].tx_buf;
        LATTICE_SPIDEV_LOG("write cmd=%X, %X, %X", log_buf[0], log_buf[1], log_buf[2]);
        if(err = copy_from_user(ice40_tx_buf, transfer[0].tx_buf, transfer[0].len))
        {
            LATTICE_SPIDEV_ERR("copy_from_user fail");
            return -1;
        }

        //write and read in one cycle
        if(err = ice40_spi_dma_xfer(ice40_tx_buf, ice40_rx_buf, transfer[0].len + transfer[1].len))
        {
            LATTICE_SPIDEV_ERR("%s, spi transfer err", __func__);
            return -1;
        }

        /*
        log_buf = ice40_rx_buf;
        printk("read:");
        for(i = 0; i < 20 && i < transfer[1].len; i ++)
        { 
            printk("[%d]=%X,", i, log_buf[i]);
        }
        printk("\n");
        */
        //don't care dummy bytes in ice40_rx_buf
        if(err = copy_to_user(transfer[1].rx_buf, &ice40_rx_buf[transfer[0].len], transfer[1].len))
        {
            LATTICE_SPIDEV_ERR("copy_to_user fail");
            return -1;
        }

        
        
        break;

        case ICE40_IOCTL_CMD_META_CHIP_CHECK:

            LATTICE_SPIDEV_LOG("ice40_image_download_success=%d", ice40_image_download_success);
            if(1 == ice40_image_download_success)
            {}
            else
            {
                return -1;
            }
        break;

        
        case ICE40_IOCTL_CMD_DOWNLOAD:

            LATTICE_SPIDEV_LOG("ice40_dl_bin exe download!!!");
            ice40_power_control(1);
            mdelay(1);
            ice40_set_gpio_run_mode();
            if(err = copy_from_user(&ice40_bin_info, ptr, sizeof(ice40_bin_info)))
            {
                LATTICE_SPIDEV_ERR("copy_from_user fail");
                return -1;
            }

           	if(err = copy_from_user(ice40_image_buf, ice40_bin_info.buf, ice40_bin_info.len))
            {
                LATTICE_SPIDEV_ERR("copy_from_user fail");
                return -1;
            }

            if(err = ice40_download_bin(ice40_image_buf, ice40_bin_info.len))
            {
                ice40_image_download_success = -1;
                return -1;
            }
            ice40_image_download_success = 1;


        break;
        
        case ICE40_IOCTL_CMD_POWER_CTRL:
            if(err = copy_from_user(&power_state, ptr, sizeof(power_state)))
            {
                LATTICE_SPIDEV_ERR("copy_from_user fail");
                return -1;
            }
            if(1 == power_state)
            {
                LATTICE_SPIDEV_LOG("chip power on");
                ice40_power_control(1);
                
                mdelay(1);

                ice40_power_state = 1;
                ice40_set_gpio_run_mode();
                
                if(0 == ice40_read_image_in_kernel(ICE40_BIN_PATH, ice40_image_buf, &ice40_image_size))
                {
                    ice40_download_bin(ice40_image_buf, ice40_image_size);
                }
            }
            else if(0 == power_state)
            {
                LATTICE_SPIDEV_LOG("chip power off");
                ice40_power_state = 0;

                ice40_power_control(0);

                ice40_set_gpio_lowpower_mode();
            }

        break;

        default:
        break;
    }

    return 0;
}

static ssize_t ice40_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
    int err;

    return 0;
}

static ssize_t ice40_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

static struct file_operations ice40_fops = {
	.owner = THIS_MODULE,
	.open = ice40_open,
	.release = ice40_release,
	.read = ice40_read,
	.write = ice40_write,
	.unlocked_ioctl = ice40_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ice40_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ice40",
	.fops = &ice40_fops,
};

static int ice40_spi_probe(struct spi_device *spi)
{
    int err = -1;
    
    LATTICE_SPIDEV_LOG("%s", __func__);

    //not power on in probe,do it in ioctl.20140211
    /*
    ice40_power_control(1);
    //reset pin
    mt_set_gpio_mode(GPIO_ICE40_RST_PIN, GPIO_MODE_00);
    //mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
    mt_set_gpio_dir(GPIO_ICE40_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_ICE40_RST_PIN, 1);
    //CDONE pin
    mt_set_gpio_mode(GPIO_ICE40_CDONE_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_ICE40_CDONE_PIN, GPIO_DIR_IN);
    
    ice40_set_gpio_spi_mode(1);
    */
    //CDONE pin
    mt_set_gpio_mode(GPIO_ICE40_CDONE_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_ICE40_CDONE_PIN, GPIO_DIR_IN);
    
    //ice40_power_control(1);
    ice40_set_gpio_lowpower_mode();

    ice40_spi_client = spi;
    ice40_spi_client->bits_per_word = 8;   // 0 表示默认 8bit per word
    ice40_spi_client->mode = SPI_MODE_3;  // 决定极性、相位。即采样、输出是在前沿还是后沿，是上升沿还是下降沿
    if(ice40_spi_client)
    {
        ice40_spi_client->controller_data = (void*)&spi_conf;
        spi_par =&spi_conf;
        
        if(!spi_par)
        {
            LATTICE_SPIDEV_ERR("spi par is NULL");
            return 0;
        }
        spi_par->setuptime = 100;//15;
        spi_par->holdtime = 100;//15;
        spi_par->high_time = 50;//10;  //   与low_time  共同决定着SPI CLK 的周期
        spi_par->low_time = 50;//10;
        spi_par->cs_idletime = 100;//20;
        spi_par->rx_mlsb = 1;       
        spi_par->tx_mlsb = 1;     //mlsb=1  表示高bit位先传，通常不需要改
        spi_par->tx_endian = 0;     //tx_endian  =1 表示大端模式，对于DMA模式，需要根据设备的Spec 来选择，通常为大端。
        spi_par->rx_endian = 0;
        spi_par->cpol = 0;      //  这里不需要再设置 ，  ice40_spi_client->mode = SPI_MODE_0 这里设置即可
        spi_par->cpha = 0;     //   这里不需要再设置 ，  ice40_spi_client->mode = SPI_MODE_0 这里设置即可
        spi_par->com_mod = DMA_TRANSFER;     // DMA  or FIFO
        //spi_par->com_mod = FIFO_TRANSFER;
        spi_par->pause = 0;     //与 deassert  的意思相反。即是否支持暂停模式，如此做SPI_CS 在多次transfer之间 不会被de-active
        spi_par->finish_intr = 1;  
        //spi_par->deassert = 0;
        spi_par->ulthigh = 0;
        spi_par->tckdly = 0;
        LATTICE_SPIDEV_LOG("setuptime=%d \n",spi_conf.setuptime);
        
        if(err = spi_setup(ice40_spi_client))
        {
            LATTICE_SPIDEV_ERR("spi_setup fail");
            goto exit;
        }
    }
    else
    {
        LATTICE_SPIDEV_ERR("ice40_spi_client is NULL");
        goto exit;
    }
	if((err = misc_register(&ice40_device)))
	{
		LATTICE_SPIDEV_ERR("lis3dh_device register failed\n");
		goto exit;
	}
    //bin size>32k,kzalloc support alloc max(128k-16) bytes
	if(!(ice40_image_buf = kzalloc(ICE40_IMAGE_BUFSIZE, GFP_KERNEL)))
	{
	    LATTICE_SPIDEV_ERR("kzalloc fail");
		err = -ENOMEM;
		goto exit;
	}
	if(!(ice40_tx_buf = kzalloc(4 * 1024, GFP_KERNEL)))
	{
	    LATTICE_SPIDEV_ERR("kzalloc fail");
		err = -ENOMEM;
		goto exit;
	}
	if(!(ice40_rx_buf = kzalloc(4 * 1024, GFP_KERNEL)))
	{
	    LATTICE_SPIDEV_ERR("kzalloc fail");
		err = -ENOMEM;
		goto exit;
	}

	if(err = ice40_create_attribute(&spi->dev))
	{
   	    LATTICE_SPIDEV_ERR("ice40_create_attribute fail");
		goto exit;
	}
#if 1
    ice40_power_control(1);
    mdelay(1);
    ice40_set_gpio_run_mode();
    ice40_image_size = sizeof(ice40_bin);
    ice40_download_bin(ice40_bin, ice40_image_size);
    mdelay(5);
    ice40_power_control(0);
    ice40_set_gpio_lowpower_mode();
#endif
    LATTICE_SPIDEV_LOG("%s ok", __func__);
    return 0;
    
exit:
    ice40_spi_client = NULL;
    return -1;
}

static int ice40_spi_remove(struct spi_device *spi)
{
    kfree(ice40_tx_buf);
    kfree(ice40_rx_buf);
    kfree(ice40_image_buf);
    return 0;
}
static int spidev_suspend(struct spi_device *spi, pm_message_t message)
{
    ice40_power_control(0);
    ice40_set_gpio_lowpower_mode();
}
static int spidev_resume(struct spi_device *spi)
{
  return 0;
}
struct spi_device_id ice40_spi_id_table = {"spi-ice40", 0};

static struct spi_driver ice40_spi_driver = {
	.driver = {
		.name = "spi-ice40",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ice40_spi_probe,
	.remove= ice40_spi_remove,
    .suspend =  spidev_suspend,
    .resume  =  spidev_resume,

	.id_table = &ice40_spi_id_table,
};
static struct spi_board_info ice40_spi_board_devs[] __initdata = {
	[0] = {        	
  .modalias="spi-ice40",
	.bus_num = 0,
	.chip_select=0,
	.mode = SPI_MODE_3,
	},
};

static int __init ice40_spi_dev_init(void)
{
	LATTICE_SPIDEV_LOG("SPI_DEV_INIT.\n");
	spi_register_board_info(ice40_spi_board_devs, ARRAY_SIZE(ice40_spi_board_devs));
	return spi_register_driver(&ice40_spi_driver);
}

static void __exit ice40_spi_dev_exit(void)
{
	LATTICE_SPIDEV_LOG("SPI_DEV_EXIT.\n");
	spi_unregister_driver(&ice40_spi_driver);
	
	return;
}

module_init(ice40_spi_dev_init);
module_exit(ice40_spi_dev_exit);

MODULE_DESCRIPTION ( "ICE40 SPI device driver" );
MODULE_AUTHOR ( "Author" );
MODULE_LICENSE("GPL");

