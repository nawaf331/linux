
#ifndef _DT9836_IOCTL_
#define _DT9836_IOCTL_

#include <linux/ioctl.h>
#include "dt9836_common_defs.h"

#ifdef	__cplusplus
extern "C"
{
#endif
#define DT9836_IOCTL		0xf3

/*
 *  IOCTL commands to interact with the board
 */
#define IOCTL_READ_BOARD_INFO	_IOR(DT9836_IOCTL,  1, board_info_t)
#define IOCTL_READ_SUBSYS_CFG	_IOWR(DT9836_IOCTL, 2, subsystem_config_t)
#define IOCTL_WRITE_SUBSYS_CFG	_IOWR(DT9836_IOCTL, 3, subsystem_config_t)
#define IOCTL_GET_SINGLE_VALUE	_IOWR(DT9836_IOCTL, 4, single_value_t)
    
#ifdef	__cplusplus
}
#endif

#endif //_DT9836_IOCTL_