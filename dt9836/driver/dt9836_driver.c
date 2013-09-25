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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ezusb.h>
#include <linux/timer.h>

#include "dt9836_driver.h"
#include "dt9836_device.h"

//Cypress vendor command 0xA3 handler to write to external RAM
#define A3LOAD_FW   "datx/a3load.fw"
//Module firmware downloaded for product id DT9836_PRE_ID
#define DT9836_FW   "datx/dt9836.fw"
//#define _DEFER_LOAD_   (1)

/* table of devices that work with this driver */
#undef  __DELIM__
#define __DELIM__ ,
#undef  _
#define _(id) 	{USB_DEVICE(DATX_VENDOR_ID, id)}

static const struct usb_device_id g_dev_id_tab[]= 
{
    DT9836_TAB
    {}      //terminator
};

MODULE_DEVICE_TABLE (usb, g_dev_id_tab);

/*
 * File scope APIs
 */
static int firmware_load(struct usb_interface *p_usb_if);

#ifdef _DEFER_LOAD_ 
static struct download_data
{
    struct delayed_work	work;
    struct usb_device   *p_usb_dev;
    const char          *p_fw;
}g_download_data;

static void firmware_work(struct work_struct *p_work)
{
    struct download_data *p_download_data = 
            container_of(p_work, struct download_data, work.work);
    int err;
    /*
     * Download the a3load.fw which implements the Cypress 0xA3 vendor
     * command that handles writes to external RAM
     */
    dev_info(&p_download_data->p_usb_dev->dev,
             "%s device%d downloading \"%s\"\n",
             __func__, p_download_data->p_usb_dev->devnum, 
             A3LOAD_FW);
    
    err = ezusb_fx2_ihex_firmware_download(p_download_data->p_usb_dev, A3LOAD_FW);
    if (err<0)
    {
        dev_err(&p_download_data->p_usb_dev->dev, 
                "%s ERROR %d downloading \"%s\"\n",
                __func__, err, A3LOAD_FW);
        return;
    }
    
    /*
     * Download the DT USB module firmware.
     */
    dev_info(&p_download_data->p_usb_dev->dev,
             "%s device%d downloading \"%s\"\n",
             __func__, p_download_data->p_usb_dev->devnum, 
             p_download_data->p_fw);
    
    err = ezusb_fx2_ihex_firmware_download(p_download_data->p_usb_dev, 
                                           p_download_data->p_fw);
    if (err<0)
    {
        dev_err(&p_download_data->p_usb_dev->dev, 
                "%s ERROR %d downloading \"%s\"\n",
                __func__, err, p_download_data->p_fw);
        return;
    }
    dev_info(&p_download_data->p_usb_dev->dev,
             "%s device%d reseting\n",
             __func__, p_download_data->p_usb_dev->devnum);
   

}
#endif

static int firmware_load(struct usb_interface *p_usb_if)
{
    int err;
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    /*
     * Download the a3load.fw which implements the Cypress 0xA3 vendor
     * command that handles writes to external RAM
     */
    dev_info(&p_usb_if->dev,
             "%s device%d id %x downloading \"%s\"\n",
             __func__, p_usb_dev->devnum, 
             p_usb_dev->descriptor.idProduct,
             A3LOAD_FW);
    
    err = ezusb_fx2_ihex_firmware_download(p_usb_dev, A3LOAD_FW);
    if (err<0)
    {
        dev_err(&p_usb_if->dev, "%s ERROR %d downloading \"%s\"\n",
                __func__, err, A3LOAD_FW);
        return (err);
    }
    
    /*
     * Download the DT USB module firmware.
     */
    dev_info(&p_usb_if->dev,
             "%s device%d id %x downloading \"%s\"\n",
             __func__, p_usb_dev->devnum, 
             p_usb_dev->descriptor.idProduct,
             DT9836_FW);
    
    err = ezusb_fx2_ihex_firmware_download(p_usb_dev, DT9836_FW);
    if (err<0)
    {
        dev_err(&p_usb_if->dev, "%s ERROR %d downloading \"%s\"\n",
                __func__, err, DT9836_FW);
        return (err);
    }
    dev_info(&p_usb_if->dev,
             "%s device%d id %x reseting\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
   
    /*
     * After downloading firmware Renumeration will occur and the new device
     * will bind to the real driver. Returning non-zero prevents the 
     * .disconnect callback
     */
  
    return (-ENODEV);
}

static int device_probe (struct usb_interface *p_usb_if,
                            const struct usb_device_id *p_usb_devid)
{
    int err = 0;
    dev_info(&p_usb_if->dev,
             "%s idProduct %x bcdDevice %x:%x version \"%s\"\n",
             __func__, 
             p_usb_devid->idProduct, 
             p_usb_devid->bcdDevice_hi, p_usb_devid->bcdDevice_lo,
             DRIVER_VERSION);
    
    if (p_usb_devid->idProduct == DT9836_PRE_ID)
    {
#ifdef _DEFER_LOAD_
        g_download_data.p_usb_dev = p_usb_dev;
        g_download_data.p_fw = g_dev_idfw_tab[index].p_dev_fw;
        INIT_DELAYED_WORK(&g_download_data.work, firmware_work);
        schedule_delayed_work(&g_download_data.work, msecs_to_jiffies(250));
#else        
        err = firmware_load(p_usb_if);
#endif    
    }
    else
    {
        err = dt9836_create(p_usb_if);
    }
    return (err);
}

static void device_disconnect (struct usb_interface *p_usb_if)
{
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    //struct usb_driver *p_usb_driver = to_usb_driver(p_usb_dev->dev.driver);
    dt9836_destroy(p_usb_if);
#ifdef _DEFER_LOAD_ 
    cancel_delayed_work_sync(&g_download_data.work);
#endif    
    dev_info(&p_usb_if->dev,
             "%s device%d id %x disconnected\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
}
#if 0
static int device_suspend(struct usb_interface *p_usb_if, pm_message_t message)
{
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    dev_info(&p_usb_if->dev,
             "%s device%d id %x\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
	return 0;
}

static int device_resume(struct usb_interface *p_usb_if)
{
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    dev_info(&p_usb_if->dev,
             "%s device%d id %x\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
	return 0;
}

static int device_pre_reset(struct usb_interface *p_usb_if)
{
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    dev_info(&p_usb_if->dev,
             "%s device%d id %x\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
	return 0;
}

static int device_post_reset(struct usb_interface *p_usb_if)
{
    struct usb_device *p_usb_dev=interface_to_usbdev(p_usb_if);
    dev_info(&p_usb_if->dev,
             "%s device%d id %x\n",
             __func__, p_usb_dev->devnum, p_usb_dev->descriptor.idProduct);
	return 0;
}
#endif
static struct usb_driver dt9836_driver = 
{
    .name       = "dt9836",
    .probe      = device_probe,
    .disconnect = device_disconnect,
	//.suspend    = device_suspend,
	//.resume     = device_resume,
	//pre_reset  = device_pre_reset,
	//.post_reset = device_post_reset,
    .id_table   = g_dev_id_tab
};

/*
 * Helper macro for USB drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 * module_init() will invoke usb_register() and module_exit() invoked
 * module_deregister()
 */
#define module_driver(__driver, __register, __unregister, ...) \
static int __init __driver##_init(void) \
{ \
         return __register(&(__driver) , ##__VA_ARGS__); \
} \
module_init(__driver##_init); \
static void __exit __driver##_exit(void) \
{ \
        __unregister(&(__driver) , ##__VA_ARGS__); \
} \
module_exit(__driver##_exit);

module_driver (dt9836_driver, usb_register, usb_deregister);

MODULE_AUTHOR       (DRIVER_AUTHOR);
MODULE_DESCRIPTION  (DRIVER_DESC);
MODULE_VERSION      (DRIVER_VERSION);
MODULE_ALIAS        (DRIVER_ALIAS);
MODULE_LICENSE      ("GPL");

//Cypress vendor command 0xA3 handler to write to external RAM
MODULE_FIRMWARE(A3LOAD_FW);
//Module firmware downloaded for product id DT9836_PRE_ID
MODULE_FIRMWARE(DT9836_FW);
