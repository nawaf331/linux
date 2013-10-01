/*
 *	Driver for DT9836 USB data acquisition module
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include "./dt9836_device.h"
#include "../include/dt9836_driver_fw_if.h"
#include "../include/dt9836_hwreg.h"
#include "dt9836_devinfo.h"

#undef  __DELIM__
#define __DELIM__ ,
#undef  _
#define _(board_type, board_type_id, ad, da, ctr, quad, din, dout) \
{board_type_id, #board_type, ad, da, ctr, quad, din, dout}
const board_info_t g_devinfo_tab[]=
{
      DEV_INFO
      {0, NULL}
};
#undef _
#undef __DELIM__
#define SIZEOF_DEVINFO_TAB  sizeof(g_devinfo_tab)/sizeof(g_devinfo_tab[0])
      
#define member_size(type, member) sizeof(((type *)0)->member)

int dt9836_board_info_get(board_type_t board_type, 
                          board_info_t * board_info_ptr)
{
    int i;
    
    for (i=0; i < SIZEOF_DEVINFO_TAB; ++i)
    {
        if (g_devinfo_tab[i].board_type == board_type)
        {
            memcpy (board_info_ptr,  &g_devinfo_tab[i], sizeof(board_info_t));
            return (0);
        }
    }
    
    return (-EINVAL);
}

/*
 * Write USB_CMD to endpoint EP_ADDR_CMD_WR
 */
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
 * Convert multi-byte parameters in USB_CMD to big endian format. The Cypress
 * FX2 is big endian.
 */
static void _convert_in_params(USB_CMD *p_usb_cmd)
{
    int     num_words =0;
    WORD    *p_param = NULL;
    
    switch (p_usb_cmd->CmdCode)
    {
        case R_SINGLE_WORD_LB :
            num_words = 1;
            p_param = &p_usb_cmd->d.ReadWordInfo.Address;
            break;
        case W_SINGLE_WORD_LB :
            break;
        case R_MULTI_WORD_LB :
            break;
        case W_MULTI_WORD_LB :
            break;
        case RMW_SINGLE_WORD_LB :
            break;
        case RMW_MULTI_WORD_LB :
            break;
        case W_DAC_THRESHOLD :
            break;
        default:
            break;
    }
    for (; (num_words > 0); --num_words, ++p_param)
    {
        *p_param = cpu_to_be16(*p_param);
    }
}


/*
 * Convert multi-byte responses to a USB_CMD from big endian to CPU format.
 * The Cypress FX2 is big endian.
 */
static void _convert_out_params(USB_CMD *p_usb_cmd, void *p_buff, int buff_len)
{
    int     num_words =0;
    WORD    *p_param = (WORD *)p_buff;
    
    switch (p_usb_cmd->CmdCode)
    {
        case R_SINGLE_WORD_LB :
            num_words = 1;
            break;
        case W_SINGLE_WORD_LB :
            break;
        case R_MULTI_WORD_LB :
            break;
        case W_MULTI_WORD_LB :
            break;
        case RMW_SINGLE_WORD_LB :
            break;
        case RMW_MULTI_WORD_LB :
            break;
        case W_DAC_THRESHOLD :
            break;
        default:
            break;
    }
    for (; (num_words > 0); --num_words, ++p_param)
    {
        *p_param = be16_to_cpu(*p_param);
    }
}

/*
 * Write USB_CMD to device over the pipe with endpoint address EP_ADDR_CMD_WR
 * 
 * Params :
 *  p_dt9836_dev    : USB device
 *  p_usb_cmd       : pointer to USB_CMD that must be filled in by caller
 * 
 * Returns :
 *  0 = success, non-zero = failure
 */
int dt9836_command_write (const dt9836_device_ptr p_dt9836_dev, 
                          USB_CMD *p_usb_cmd)
{
    int err;
    
    _convert_in_params(p_usb_cmd);
    
    spin_lock(&p_dt9836_dev->command_rdwr_spinlock);
	err = _command_write (p_dt9836_dev, p_usb_cmd);
    spin_unlock(&p_dt9836_dev->command_rdwr_spinlock);
    
    if (err)
    {
        dev_err(&p_dt9836_dev->p_usbif->dev, 
                "%s ERROR %d CmdCode 0x%x", 
                __func__, err, p_usb_cmd->CmdCode);
    }
    return (err);
}

/*
 * Write USB_CMD to device over the pipe with endpoint address EP_ADDR_CMD_WR
 * and read response to the command over pipe with endpoint address 
 * EP_ADDR_CMD_RD
 * 
 * Params :
 *  p_dt9836_dev    : USB device
 *  p_usb_cmd       : pointer to USB_CMD that must be filled in by caller
 *  p_buff          : pointer to buffer that has response to command
 *  buff_len        : buffer length. 
 * 
 * Returns :
 *  0 = success, non-zero = failure
 */
int dt9836_command_write_read (const dt9836_device_ptr p_dt9836_dev, 
                               USB_CMD * p_usb_cmd, 
                               void *p_buff, int buff_len)
{
    int err1 = 0;
    int err2 = 0;
    int len = 0;
    
    _convert_in_params(p_usb_cmd);
    
    spin_lock(&p_dt9836_dev->command_rdwr_spinlock);
	err1 = _command_write (p_dt9836_dev, p_usb_cmd);
    if (!err1)
    {
        err2 = usb_bulk_msg (p_dt9836_dev->p_usbdev,
                            p_dt9836_dev->command_rd_ep.pipe,
                            p_buff, buff_len, 
                            &len,
                            DT9836_CMD_RD_TIMEOUT);
    }
    spin_unlock(&p_dt9836_dev->command_rdwr_spinlock);
    
    if (err1 || err2 || (len != buff_len))
    {
        dev_err(&p_dt9836_dev->p_usbif->dev, 
                "%s ERROR err1=%d err2=%d usb_bulk_msg", 
                __func__, err1, err2);
    }
    else
    {
        _convert_out_params(p_usb_cmd, p_buff, buff_len);
    }
    return (err1 ? err1: err2);
}
#if 0
int is_big_endian(void)
{
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    printk("%s endian\n", (bint.c[0] == 1)?"BIG":"LITTLE");
    return bint.c[0] == 1; 
}
#endif

int dt9836_board_type_read(const dt9836_device_ptr p_dt9836_dev,
                           board_type_t *p_board_type)
{
    int err = 0;
    USB_CMD cmd;
    //Get the board type
    cmd.CmdCode = R_SINGLE_WORD_LB;
    cmd.d.ReadWordInfo.Address = VERSION_ID;
    err = dt9836_command_write_read(p_dt9836_dev, &cmd,
                                    p_board_type,
                                    sizeof(*p_board_type));
    return (err);
}


int dt9836_serial_num_read(const dt9836_device_ptr p_dt9836_dev,
                           uint32_t * p_sernum)
{
    int err = 0;
    int i;
    USB_CMD cmd;
    //Get the board serial number
    cmd.CmdCode = R_MULTI_BYTE_I2C_REG;
    cmd.d.ReadI2CMultiInfo.NumReads = sizeof (sizeof(p_dt9836_dev->serial_num));
    for (i=0; i < cmd.d.ReadI2CMultiInfo.NumReads; ++i)
    {
        cmd.d.ReadI2CMultiInfo.Read[i].DevAddress = EEPROM_I2C_ADR;
        cmd.d.ReadI2CMultiInfo.Read[i].Register = EEPROM_SER_NUM0_OFFSET + i;
    }
    err = dt9836_command_write_read(p_dt9836_dev, &cmd,
                                    p_sernum,
                                    sizeof(*p_sernum));
    return (err);
}

