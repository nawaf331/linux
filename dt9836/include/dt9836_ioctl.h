
#ifndef _DT9836_IOCTL_
#define _DT9836_IOCTL_

#include <linux/ioctl.h>
#include "dt9836_board_cfg.h"

#ifdef	__cplusplus
extern "C"
{
#endif
#define DT9836_IOCTL		0xf3

/*
 *  IOCTL command to read the board configuration structure
 */
#define IOCTL_READ_BOARD_INFO	_IOR(DT9836_IOCTL, 1, board_info_t)
#define SET_A_SUSPEND_REQ	_IOW(DT9836_IOCTL, 2, int)
#define SET_A_BUS_DROP		_IOW(DT9836_IOCTL, 3, int)
#define SET_A_BUS_REQ		_IOW(DT9836_IOCTL, 4, int)
#define SET_B_BUS_REQ		_IOW(DT9836_IOCTL, 5, int)
#define GET_A_SUSPEND_REQ	_IOR(DT9836_IOCTL, 6, int)
#define GET_A_BUS_DROP		_IOR(DT9836_IOCTL, 7, int)
#define GET_A_BUS_REQ		_IOR(DT9836_IOCTL, 8, int)
#define GET_B_BUS_REQ		_IOR(DT9836_IOCTL, 9, int)
    
#ifdef	__cplusplus
}
#endif

#endif //_DT9836_IOCTL_