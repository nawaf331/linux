/*
 *	Driver for DT9836 USB data acquisition module
 *  
 *  (C) Copyright (c) 2013 Data Translation Inc
 *                    www.datatranslation.com
 *
 *  Derived from USB Skeleton driver - 2.2
 *  Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * 
 *  Uses APIs exported from ezusb.c authored by
 *  Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include "dt9836_device.h"
#include "../include/dt9836_driver_fw_if.h"
#include "../include/dt9836_hwreg.h"
/**
* usb_endpoint_maxp - get endpoint's max packet size
* @epd: endpoint to be checked
*
* Returns @epd's max packet
*/
static inline int _usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd)
{
    return __le16_to_cpu(epd->wMaxPacketSize);
}

static int _command_write(const dt9836_device_ptr p_dt9836_dev, 
                          const USB_CMD *p_usb_cmd)
{
    int err, len;
	err = usb_bulk_msg (p_dt9836_dev->p_usbdev,
                        p_dt9836_dev->command_wr_ep.pipe,
                        (void *)p_usb_cmd, sizeof(USB_CMD), 
                        &len,
                        DT9836_CMD_WR_TIMEOUT);
    if (!err && (len != sizeof(USB_CMD)))
    {
        err = -EIO;
    }
    return (err);
}
/*
 * Write USB_CMD to device over the pipe with endpoint address EP_ADDR_CMD_WR
 * 
 * Returns :
 *  0 = success, non-zero = failure
 */
int dt9836_command_write (const dt9836_device_ptr p_dt9836_dev, 
                          const USB_CMD *p_usb_cmd)
{
    int err;
    
    spin_lock(&p_dt9836_dev->command_rdwr_spinlock);
	err = _command_write (p_dt9836_dev, p_usb_cmd);
    spin_unlock(&p_dt9836_dev->command_rdwr_spinlock);
    
    if (err)
    {
        dev_err(&p_dt9836_dev->p_usbif->dev, 
                "%s ERROR %d usb_bulk_msg", 
                __func__, err);
    }
    return (err);
}
int dt9836_read_command (const dt9836_device_ptr p_dt9836_dev, 
                             void *p_buff, int buff_len)
{
    int err, len;
	err = usb_bulk_msg (p_dt9836_dev->p_usbdev,
                        p_dt9836_dev->command_rd_ep.pipe,
                        p_buff, buff_len, 
                        &len,
                        DT9836_CMD_RD_TIMEOUT);
    if (err)
    {
        dev_err(&p_dt9836_dev->p_usbif->dev, 
                "%s ERROR %d usb_bulk_msg", 
                __func__, err);
    }
    return (err);
}
int is_big_endian(void)
{
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    printk("%s endian\n", (bint.c[0] == 1)?"BIG":"LITTLE");
    return bint.c[0] == 1; 
}
static int readtest(const dt9836_device_ptr p_dt9836_dev)
{
    USB_CMD dt9836_cmd;
    uint16_t result[3];
    uint32_t serial;
    int err;
    
    is_big_endian();

#if 1    
    dt9836_cmd.CmdCode = R_MULTI_BYTE_I2C_REG;
	dt9836_cmd.d.ReadI2CMultiInfo.NumReads = sizeof(result);

	dt9836_cmd.d.ReadI2CMultiInfo.Read[0].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[0].Register = EEPROM_VENDOR_IDL_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[1].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[1].Register = EEPROM_VENDOR_IDH_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[2].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[2].Register = EEPROM_PRODUCT_IDL_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[3].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[3].Register = EEPROM_PRODUCT_IDH_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[4].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[4].Register = EEPROM_DEVICE_IDL_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[5].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[5].Register = EEPROM_DEVICE_IDH_OFFSET;
    
    err = dt9836_command_write(p_dt9836_dev, &dt9836_cmd);
    
    if (!err)
    {
        err = dt9836_read_command(p_dt9836_dev, result, sizeof(result));
        if (!err)
        {
            printk("vend %x prod %x dev %x\n", result[0], result[1], result[2]);
        }
    }
#endif 
    
#if 1    
    dt9836_cmd.CmdCode = R_SINGLE_WORD_LB;
	dt9836_cmd.d.ReadWordInfo.Address = VERSION_ID;
    err = dt9836_command_write(p_dt9836_dev, &dt9836_cmd);
    
    if (!err)
    {
        err = dt9836_read_command(p_dt9836_dev, result, sizeof(uint16_t));
        if (!err)
        {
            printk("VERSION_ID %x\n", result[0]);
        }
    }
    dt9836_cmd.CmdCode = R_SINGLE_WORD_LB;
	dt9836_cmd.d.ReadWordInfo.Address = cpu_to_be16(VERSION_ID);
    err = dt9836_command_write(p_dt9836_dev, &dt9836_cmd);
    
    if (!err)
    {
        err = dt9836_read_command(p_dt9836_dev, result, sizeof(uint16_t));
        if (!err)
        {
            printk("cpu_to_be16 VERSION_ID %x\n", result[0]);
        }
    }
#endif

#if 1
	dt9836_cmd.CmdCode = R_MULTI_BYTE_I2C_REG;
	dt9836_cmd.d.ReadI2CMultiInfo.NumReads = sizeof (uint32_t);

	dt9836_cmd.d.ReadI2CMultiInfo.Read[0].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[0].Register = EEPROM_SER_NUM0_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[1].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[1].Register = EEPROM_SER_NUM1_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[2].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[2].Register = EEPROM_SER_NUM2_OFFSET;

	dt9836_cmd.d.ReadI2CMultiInfo.Read[3].DevAddress = EEPROM_I2C_ADR;
	dt9836_cmd.d.ReadI2CMultiInfo.Read[3].Register = EEPROM_SER_NUM3_OFFSET;

    err = dt9836_command_write(p_dt9836_dev, &dt9836_cmd);
    
    if (!err)
    {
        err = dt9836_read_command(p_dt9836_dev, &serial, sizeof(serial));
        if (!err)
        {
            printk("serial # 0x%x %u\n", serial, serial);
        }
    }
#endif
    
    return (err);
}

/*
 * Called from the probe function to create and initialize data structs
 */
int dt9836_create(struct usb_interface	*p_usbif)
{
    int err = 0;
    int ep_found = 0;
    int i;
    dt9836_device_ptr p_dt9836_dev;
    struct usb_host_interface * p_if_desc;
     
    //Allocate the device structure
    p_dt9836_dev = kzalloc(sizeof(dt9836_device), GFP_KERNEL);
    if (!p_dt9836_dev)
    {
        dev_err(&p_usbif->dev, "%s ERROR kzalloc", __func__);
        return (-ENOMEM);
    }
    
    p_dt9836_dev->p_usbif = p_usbif;
    p_dt9836_dev->p_usbdev = interface_to_usbdev(p_usbif);
    spin_lock_init(&p_dt9836_dev->command_rdwr_spinlock);
    
    //Get all the endpoints
    p_if_desc = p_usbif->cur_altsetting;
    for (i=0; i < p_if_desc->desc.bNumEndpoints; ++i)
    {
        struct usb_endpoint_descriptor *p_ep_desc;
        p_ep_desc = &p_if_desc->endpoint[i].desc;
        
        switch (p_ep_desc->bEndpointAddress)
        {
            case EP_ADDR_MSG:   //IN
                p_dt9836_dev->message_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->command_rd_ep.pipe = 
                        usb_rcvbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_MSG);
                ++ep_found;
                break;
            case EP_ADDR_CMD_WR:    //OUT
                p_dt9836_dev->command_wr_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->command_wr_ep.pipe =
                        usb_sndbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_CMD_WR);
                ++ep_found;
                break;
            case EP_ADDR_CMD_RD:    //IN
                p_dt9836_dev->command_rd_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->command_rd_ep.pipe = 
                        usb_rcvbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_CMD_RD);
                ++ep_found;
                break;
            case EP_ADDR_STREAM_WR: //OUT
                p_dt9836_dev->stream_wr_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->command_wr_ep.pipe =
                        usb_sndbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_STREAM_WR);
                ++ep_found;
                break;
            case EP_ADDR_STREAM_RD: //IN
                p_dt9836_dev->stream_rd_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->command_rd_ep.pipe = 
                        usb_rcvbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_STREAM_RD);
                ++ep_found;
                break;
            default :   //unknown endpoint
                dev_warn(&p_usbif->dev, "%s WARNING unknown ep 0x%x", 
                        __func__, p_ep_desc->bEndpointAddress);
        }
    }
    
        dev_err(&p_usbif->dev, "%s line %d", __func__, __LINE__);

    if (!err || (ep_found == DT9836_NUM_ENDPOINTS))
    {
printk("%s usb_if.dev %x usbdev.dev %x", 
        __func__, (uint32_t)&p_usbif->dev, (uint32_t)&p_dt9836_dev->p_usbdev->dev);

        usb_set_intfdata(p_usbif, p_dt9836_dev);
        
        //readtest(p_dt9836_dev);
    }
    else
    {
        kfree(p_dt9836_dev);
    }
    
    return (err);
}

/*
 * Called from the disconnect function
 */
void dt9836_destroy(struct usb_interface *p_usbif)
{
    dt9836_device_ptr p_dev;
    
    p_dev = usb_get_intfdata(p_usbif);
	usb_set_intfdata(p_usbif, NULL);
    
    if (!p_dev)
    {
        kfree(p_dev);
    }
}