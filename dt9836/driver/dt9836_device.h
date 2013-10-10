/*
 *   Driver for DT9836 USB data acquisition module.
 *  APIs, data structures and macros to interface to the device
 *  
 *  (C) Copyright (c) 2013 Data Translation Inc
 *                    www.datatranslation.com
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef DT9836_DEVICE_H
#define	DT9836_DEVICE_H
#include <linux/usb.h>
#include "../include/dt9836_driver_fw_if.h"
#include "../include/dt9836_common_defs.h"

#ifdef	__cplusplus
extern "C"
{
#endif

/*
 * sizeof a struct member
 */
#define member_size(type, member) sizeof(((type *)0)->member)
/*
 * Addresses of endpoints used by the device
 */
#define  EP_ADDR_MSG        (USB_DIR_IN | 0X08)
#define  EP_ADDR_CMD_WR     (USB_DIR_OUT | 0X01)
#define  EP_ADDR_CMD_RD     (USB_DIR_IN | 0X01)
#define  EP_ADDR_STREAM_WR  (USB_DIR_OUT | 0X06)
#define  EP_ADDR_STREAM_RD  (USB_DIR_IN | 0X02)

#define  DT9836_CMD_WR_TIMEOUT  (5*HZ) // 5 seconds in jiffies
#define  DT9836_CMD_RD_TIMEOUT  (5*HZ) // 5 seconds in jiffies
#define  USB_CONNECTED       (1)         //USB cable connected

/*
 * USB end point information
 */    
typedef struct
{
    size_t              size;       //size of endpoint
    unsigned int        pipe;       //pipe id
} ep_info_t;

struct dt9836_device; //forward declaration

/*
 * DA subsystem structure
 */
typedef struct
{
    subsystem_config_t      config;
    int (*set_config)(struct dt9836_device *, const subsystem_config_t *);
    int (*get_single)(struct dt9836_device *, single_value_t *);
}da_subsystem_t;

/*
 * Indeces into dt9836_device_t::da_subsystem[] array
 */
#define     DT9836_DIN_SUBSYSTEM    (0)
#define     DT9836_MAX_SUBSYSTEM    (DT9836_DIN_SUBSYSTEM + 1)
/*
 * DT9836 Device details
 */
typedef struct dt9836_device
{
    struct usb_device       *p_usbdev;      //USB device to communicate with     
    struct usb_interface	*p_usbif;       //USB interface  
	struct kref             kref;           //reference counter
    atomic_t                connected;      //USB connection status
    ep_info_t               message_ep;     //IN message pipe
    ep_info_t               command_rd_ep;  //IN command response pipe
    ep_info_t               command_wr_ep;  //OUT command pipe
    ep_info_t               stream_rd_ep;   //IN stream pipe
    ep_info_t               stream_wr_ep;   //OUT streampipe
    spinlock_t              command_rdwr_spinlock;
    board_info_t            board_info;     //Board information
    struct                  //shadow registers corresponding to device registers
    {
        uint16_t            output_control; //DA control register
        uint16_t            in_control;     //AD control register
    }shadow_reg;
    
    spinlock_t              da_subsystem_spinlock;
    da_subsystem_t          da_subsystem[DT9836_MAX_SUBSYSTEM];
}dt9836_device_t;

typedef dt9836_device_t * dt9836_device_ptr; 

/*
 * Write a USB_CMD to the board
 */
int dt9836_command_write (const dt9836_device_ptr p_dt9836_dev, 
                          USB_CMD *p_usb_cmd);

/*
 * Write a USB_CMD to the board and read back the response to the command
 * into the specified buffer
 */
int dt9836_command_write_read (const dt9836_device_ptr p_dt9836_dev,
                               USB_CMD * p_usb_cmd, 
                               void *p_buff, int buff_len);

/*
 * Read the serial number of the board
 */
int dt9836_serial_num_read(const dt9836_device_ptr p_dt9836_dev,
                           uint32_t * p_sernum);

/*
 * Returns the static configuration of the board
 */
int dt9836_board_info_get(board_type_t board_type, 
                          board_info_t * board_info_ptr);

/*
 * Read the value from a specified register in the board
 */
int dt9836_reg_read(const dt9836_device_ptr p_dt9836_dev,
                    uint16_t reg_addr, uint16_t * value_ptr);

/*
 * Write a value to a register in the board 
 */
int dt9836_reg_write(const dt9836_device_ptr p_dt9836_dev,
                    uint16_t reg_addr, uint16_t reg_value);


/*
 * One time initialization of hardware registers in the board. This is called
 * from the probe function
 */
int dt9836_hw_init(const dt9836_device_ptr p_dt9836_dev);

#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_DEVICE_H */

