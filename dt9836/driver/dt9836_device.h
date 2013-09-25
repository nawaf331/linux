/*
 *   Driver for DT9836 USB data acquisition module
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

#ifdef	__cplusplus
extern "C"
{
#endif

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
    
struct ep_info
{
    size_t              size;       //size of endpoint
    unsigned int        pipe;       //pipe id
};
typedef struct ep_info ep_info;
/*
 * Device details
 */
struct dt9836_device
{
    struct usb_device       *p_usbdev;  //USB device to communicate with     
    struct usb_interface	*p_usbif;   //USB interface  
    ep_info                 message_ep; //IN message pipe
    ep_info                 command_rd_ep; //IN command response pipe
    ep_info                 command_wr_ep; //OUT command pipe
    ep_info                 stream_rd_ep; //IN stream pipe
    ep_info                 stream_wr_ep;  //OUT streampipe
    uint32_t                serial_num;     //serial number
    uint16_t                board_type; //board type
    spinlock_t              command_rdwr_spinlock;
};
typedef struct dt9836_device dt9836_device;
typedef dt9836_device * dt9836_device_ptr; 

int     dt9836_create(struct usb_interface	*p_usbif);
void    dt9836_destroy(struct usb_interface	*p_usbif);
int     dt9836_command_write (const dt9836_device_ptr p_dt9836_dev, 
                              const USB_CMD *p_usb_cmd);
int     dt9836_read_command (const dt9836_device_ptr p_dt9836_dev, 
                             void *p_buff, int buff_len);

#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_DEVICE_H */

