#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/timer.h>

#include <mach/eint.h>

#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>

#include "sw_tx_power.h"

//#include <mach/mtk_ccci_helper.h>
#include <mach/mt_ccci_common.h>

#include "../../tc1_interface/gpt/lg_partition.h"

static struct task_struct *swtp_kthread = NULL;
static wait_queue_head_t swtp_isr_wait;
static int swtp_send_sig = 0;

static struct timer_list swtp_timer;

static DEFINE_MUTEX(swtp_ctrl_lock);

#define SWTP_DEFAULT_MODE	MODE_SWTP(4G_TABLE0,3G_TABLE0,2G_TABLE0)

extern int register_ccci_sys_call_back(int md_id, unsigned int id, ccci_sys_cb_func_t func);

static swtp_state_type swtp_state_reg [SWTP_CTRL_MAX_STATE] = 
{
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_NONE,   2G_NONE  )}, /* SWTP_CTRL_USER_SET0 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_NONE,   2G_TABLE0)}, /* SWTP_CTRL_USER_SET1 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_NONE,   2G_TABLE1)}, /* SWTP_CTRL_USER_SET2 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_NONE,   2G_TABLEX)}, /* SWTP_CTRL_USER_SET3 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE0, 2G_NONE  )}, /* SWTP_CTRL_USER_SET4 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE0, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET5 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE0, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET6 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE0, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET7 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE1, 2G_NONE  )}, /* SWTP_CTRL_USER_SET8 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE1, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET9 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE1, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET10 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLE1, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET11 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLEX, 2G_NONE  )}, /* SWTP_CTRL_USER_SET12 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLEX, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET13 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLEX, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET14 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_NONE,   3G_TABLEX, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET15 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_NONE,   2G_NONE  )}, /* SWTP_CTRL_USER_SET16 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_NONE,   2G_TABLE0)}, /* SWTP_CTRL_USER_SET17 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_NONE,   2G_TABLE1)}, /* SWTP_CTRL_USER_SET18 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_NONE,   2G_TABLEX)}, /* SWTP_CTRL_USER_SET19 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE0, 2G_NONE  )}, /* SWTP_CTRL_USER_SET20 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE0, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET21 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE0, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET22 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE0, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET23 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE1, 2G_NONE  )}, /* SWTP_CTRL_USER_SET24 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE1, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET25 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE1, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET26 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLE1, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET27 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLEX, 2G_NONE  )}, /* SWTP_CTRL_USER_SET28 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLEX, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET29 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLEX, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET30 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE0, 3G_TABLEX, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET31 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_NONE,   2G_NONE  )}, /* SWTP_CTRL_USER_SET32 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_NONE,   2G_TABLE0)}, /* SWTP_CTRL_USER_SET33 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_NONE,   2G_TABLE1)}, /* SWTP_CTRL_USER_SET34 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_NONE,   2G_TABLEX)}, /* SWTP_CTRL_USER_SET35 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE0, 2G_NONE  )}, /* SWTP_CTRL_USER_SET36 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE0, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET37 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE0, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET38 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE0, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET39 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE1, 2G_NONE  )}, /* SWTP_CTRL_USER_SET40 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE1, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET41 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE1, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET42 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLE1, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET43 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLEX, 2G_NONE  )}, /* SWTP_CTRL_USER_SET44 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLEX, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET45 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLEX, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET46 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLE1, 3G_TABLEX, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET47 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_NONE,   2G_NONE  )}, /* SWTP_CTRL_USER_SET48 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_NONE,   2G_TABLE0)}, /* SWTP_CTRL_USER_SET49 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_NONE,   2G_TABLE1)}, /* SWTP_CTRL_USER_SET50 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_NONE,   2G_TABLEX)}, /* SWTP_CTRL_USER_SET51 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE0, 2G_NONE  )}, /* SWTP_CTRL_USER_SET52 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE0, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET53 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE0, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET54 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE0, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET55 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE1, 2G_NONE  )}, /* SWTP_CTRL_USER_SET56 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE1, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET57 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE1, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET58 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLE1, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET59 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLEX, 2G_NONE  )}, /* SWTP_CTRL_USER_SET60 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLEX, 2G_TABLE0)}, /* SWTP_CTRL_USER_SET61 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLEX, 2G_TABLE1)}, /* SWTP_CTRL_USER_SET62 */
    {SWTP_MODE_OFF, SWTP_NORMAL_MODE, MODE_SWTP(4G_TABLEX, 3G_TABLEX, 2G_TABLEX)}, /* SWTP_CTRL_USER_SET63 */

    {SWTP_MODE_OFF, SWTP_SUPER_MODE,  MODE_SWTP(4G_NONE,   3G_NONE,   2G_NONE  )}  /* SWTP_CTRL_SUPER_SET */
};

static int swtp_set_tx_power(unsigned int mode)
{
    int ret1 = 0, ret2 = 0;

    if(get_modem_is_enabled(0))
    ret1 = switch_MD1_Tx_Power(mode);

    if(get_modem_is_enabled(1))    
    ret2 = switch_MD2_Tx_Power(mode);   

    return ((ret1 == 0) && (ret2 == 0));
}

int swtp_reset_tx_power(void)
{
    int ret1 = 0, ret2 = 0;

    if(get_modem_is_enabled(0))
    ret1 = switch_MD1_Tx_Power(SWTP_DEFAULT_MODE);
    
    if(get_modem_is_enabled(1))
    ret2 = switch_MD2_Tx_Power(SWTP_DEFAULT_MODE);

    return ((ret1 == 0) && (ret2 == 0));
}

int swtp_rfcable_tx_power(void)
{
    int ret1 = 0, ret2 = 0;

    if(get_modem_is_enabled(0))
    ret1 = switch_MD1_Tx_Power(swtp_state_reg[SWTP_CTRL_SUPER_SET].setvalue);
    
    if(get_modem_is_enabled(1))
    ret2 = switch_MD2_Tx_Power(swtp_state_reg[SWTP_CTRL_SUPER_SET].setvalue);

    return ((ret1 == 0) && (ret2 == 0));
}

int swtp_change_mode(unsigned int ctrid, unsigned int mode)
{
    if(ctrid >= SWTP_CTRL_MAX_STATE) return -1;

    swtp_state_reg[ctrid].mode = mode;

    return 0;    
}

unsigned int swtp_get_mode(swtp_state_type *swtp_super_state, swtp_state_type *swtp_normal_state)
{
    unsigned int ctrid, run_mode = SWTP_CTRL_MAX_STATE;

    memset(swtp_super_state,  0, sizeof(swtp_state_type));
    memset(swtp_normal_state, 0, sizeof(swtp_state_type));

    mutex_lock(&swtp_ctrl_lock);
    for(ctrid = 0; ctrid < SWTP_CTRL_MAX_STATE; ctrid++) 
    {
        if(swtp_state_reg[ctrid].enable == SWTP_MODE_ON)
        {
            if(swtp_state_reg[ctrid].mode == SWTP_SUPER_MODE)
            {
                memcpy(swtp_super_state, &swtp_state_reg[ctrid], sizeof(swtp_state_type));
                run_mode = ctrid;
            }
            else if(swtp_state_reg[ctrid].mode == SWTP_NORMAL_MODE)
            {
                memcpy(swtp_normal_state, &swtp_state_reg[ctrid], sizeof(swtp_state_type));
                if(run_mode >= SWTP_CTRL_MAX_STATE) run_mode = ctrid;
            }
        }
    }
    mutex_unlock(&swtp_ctrl_lock);

    return run_mode;    
}

static int swtp_clear_mode(unsigned int mode)
{
    unsigned int ctrid;

    for(ctrid = 0; ctrid < SWTP_CTRL_MAX_STATE; ctrid++)
    {
        if(swtp_state_reg[ctrid].mode == mode)
            swtp_state_reg[ctrid].enable = SWTP_MODE_OFF;
    }

    return 0;
}

int swtp_set_mode(unsigned int ctrid, unsigned int enable)
{
    if(ctrid >= SWTP_CTRL_MAX_STATE) return -1;

    mutex_lock(&swtp_ctrl_lock);
    swtp_clear_mode(swtp_state_reg[ctrid].mode);
    swtp_state_reg[ctrid].enable = enable;
    mutex_unlock(&swtp_ctrl_lock);

    swtp_send_sig++;
    wake_up_interruptible(&swtp_isr_wait);

    return 0;    
}

int swtp_reset_mode(void)
{
    mutex_lock(&swtp_ctrl_lock);
    swtp_clear_mode(SWTP_SUPER_MODE);
    swtp_clear_mode(SWTP_NORMAL_MODE);
    mutex_unlock(&swtp_ctrl_lock);

    swtp_send_sig++;
    wake_up_interruptible(&swtp_isr_wait);

    return 0;    
}

int swtp_set_mode_unlocked(unsigned int ctrid, unsigned int enable)
{
    if(ctrid >= SWTP_CTRL_MAX_STATE) return -1;

    swtp_clear_mode(swtp_state_reg[ctrid].mode);
    swtp_state_reg[ctrid].enable = enable;

    swtp_send_sig++;
    wake_up(&swtp_isr_wait);

    return 0;    
}

extern int swtp_mod_eint_enable(void);
extern int swtp_mod_eint_init(void);
extern int swtp_mod_eint_read(void);

#define SWTP_MAX_TIMEOUT_SEC	50
#define SWTP_MAX_TIMER_CNT	8

static void swtp_mod_eint_read_timer(unsigned long data)
{
    printk("[swtp] swtp_mod_eint_read_timer [cnt[%ld]\n", data);
    swtp_mod_eint_read();

    if(data) {
        swtp_timer.data = data - 1; // retry count
        swtp_timer.expires = jiffies + 2 * HZ;
        add_timer(&swtp_timer);
    }
}

static int swtp_state_machine(void *handle)
{
    int run_mode, super_mode, normal_mode;
    int i = 0;
    int timeout;

    init_waitqueue_head(&swtp_isr_wait);

#if 0
    // waiting modem working & set cable status value
    while(i < SWTP_MAX_TIMEOUT_SEC)
    {
        timeout = wait_event_interruptible_timeout(swtp_isr_wait, swtp_send_sig, HZ);
        if(swtp_mod_eint_read()) break;
        i++;
    }

    // ccci channel is ready, but MD L1 part is not ready yet, need to re-send more.
    swtp_timer.expires = jiffies + 2 * HZ;
    add_timer(&swtp_timer);
#endif

    swtp_mod_eint_enable();

    // state machine
    while(1)
    {
        wait_event_interruptible(swtp_isr_wait, swtp_send_sig || kthread_should_stop());

        if(kthread_should_stop())
            break;

        swtp_send_sig--;

        run_mode = super_mode = normal_mode = SWTP_CTRL_MAX_STATE;

        mutex_lock(&swtp_ctrl_lock);
        for(i=0; i < SWTP_CTRL_MAX_STATE; i++)
        {
            if(swtp_state_reg[i].enable == SWTP_MODE_OFF) continue;

            if(swtp_state_reg[i].mode == SWTP_SUPER_MODE) super_mode = i;
            else if(swtp_state_reg[i].mode == SWTP_NORMAL_MODE) normal_mode = i;
            else printk(KERN_ALERT "[swtp error]: need to check 0x%x 0x%x\n", i, swtp_state_reg[i].mode );
        }
        mutex_unlock(&swtp_ctrl_lock);

        if(super_mode < SWTP_CTRL_MAX_STATE)
            run_mode = super_mode;
        else if(normal_mode < SWTP_CTRL_MAX_STATE)
            run_mode = normal_mode;
        else if(run_mode == SWTP_CTRL_MAX_STATE)
        {
            swtp_reset_tx_power();
            printk(KERN_ALERT "[swtp]: swtp_reset_tx_power : 0x%x\n", SWTP_DEFAULT_MODE);			
			LGE_API_test();
            continue;
        }

        swtp_set_tx_power(swtp_state_reg[run_mode].setvalue);		
        printk(KERN_ALERT "[swtp]: swtp_set_tx_power [%d]: 0x%x\n", run_mode, swtp_state_reg[run_mode].setvalue);
		LGE_API_test();
    } 

    return 0;
}

void swtp_mode_restart(void)
{
    printk(KERN_ALERT "swtp_mode_restart.\n");    

    kthread_stop(swtp_kthread);
    del_timer(&swtp_timer);

    swtp_mod_eint_init();

    init_timer(&swtp_timer);
    swtp_timer.function = (void *)&swtp_mod_eint_read_timer;
    swtp_timer.data = SWTP_MAX_TIMER_CNT; // retry count    

    swtp_kthread = kthread_run(swtp_state_machine, 0, "swtp kthread");
}

static int swtp_mode_update_handler(int md_id, int data)
{
    printk("[swtp] swtp_mode_update_handler  md_id[%d], data[%d]\n",md_id, data);

    swtp_mod_eint_read();

    return 0;
}

static int __init swtp_mod_init(void)
{
    int ret = 0;
    printk(KERN_ALERT "swtp_mod_init.\n");    

    swtp_mod_eint_init();

    if(get_modem_is_enabled(0)){
        ret = register_ccci_sys_call_back(0, MD_SW_MD1_TX_POWER_REQ, swtp_mode_update_handler);
        printk("[swtp register MD1 call back [%d]\n", ret);
    }

    if(get_modem_is_enabled(1)) {
        ret = register_ccci_sys_call_back(1, MD_SW_MD2_TX_POWER_REQ, swtp_mode_update_handler);
	    printk("[swtp register MD2call back [%d]\n", ret);
    }

    swtp_kthread = kthread_run(swtp_state_machine, 0, "swtp kthread");

    init_timer(&swtp_timer);
    swtp_timer.function = (void *)&swtp_mod_eint_read_timer;
    swtp_timer.data = SWTP_MAX_TIMER_CNT; // retry count

    printk(KERN_ALERT "[swtp_tr] running swtp thread\n");

    return 0;
}

static void __exit swtp_mod_exit(void)
{
    printk(KERN_ALERT "swtp_tr_mod_exit.\n");  

    kthread_stop(swtp_kthread);
    del_timer(&swtp_timer);  

    return;
}

module_init(swtp_mod_init);
module_exit(swtp_mod_exit);

MODULE_DESCRIPTION("SWTP Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MTK");

