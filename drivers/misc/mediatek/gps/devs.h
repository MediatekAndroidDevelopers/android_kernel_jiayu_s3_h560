#ifndef __DEVS_H__
#define __DEVS_H__

/*
 *superdragonpt July 15 2016
 *This doesn't exist on 3.18 MediaTek Kernel, but we need this for sanity
 *added needed files locally, modified to compile
 *This is one big hack... but works ;)
 */
//#include <board-custom.h>
#include "board.h"

#define CFG_DEV_UART1
#define CFG_DEV_UART2
#define CFG_DEV_UART3
#define CFG_DEV_UART4

/*
 * Define constants.
 */

#define MTK_UART_SIZE 0x100

/*
 * Define function prototype.
 */

extern int mt_board_init(void);

/*
 *superdragonpt July 15 2016
 *This doesn't exist on 3.18, on 3.10 is unknown if this is needed
 *there's multiple changes by Vendor's Here
 *Some Vendor are idiots and don't use guidelines
 */
//extern unsigned int *get_modem_size_list(void);
//extern unsigned int get_nr_modem(void);

#endif  /* !__MT6575_DEVS_H__ */

