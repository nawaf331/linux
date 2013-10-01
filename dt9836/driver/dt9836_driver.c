/*
 *	Driver for DT9836 USB data acquisition module
 *  
 *  (C) Copyright (c) 2013 Data Translation Inc
 *                    www.datatranslation.com
 *
 *  Derived from USB Skeleton driver - 2.2
 *  Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * 
 *  Uses APIs exported from ezusb.c 
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

#include "./dt9836_driver.h"
#include "./dt9836_device.h"
#include "./dt9836_sysfs.h"

//Cypress vendor command 0xA3 handler to write to external RAM
#define A3LOAD_FW   "datx/a3load.fw"
//Module firmware downloaded for product id DT9836_PRE_ID
#define DT9836_FW   "datx/dt9836.fw"


/*
 * File scope APIs
 */
static int _firmware_load(struct usb_interface *p_usb_if);

static int _dt9836_open(struct inode *inode, struct file *file);
static int _dt9836_release(struct inode *inode, struct file *file);
static ssize_t _dt9836_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos);


static int _device_probe (struct usb_interface *p_usb_if,
                            const struct usb_device_id *p_usb_devid);
static void _device_disconnect (struct usb_interface *p_usb_if);
static int  _dt9836_create(struct usb_interface	*p_usbif);
static void _dt9836_destroy(struct kref *p_kref);


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

static struct usb_driver dt9836_driver = 
{
    .name       = "dt9836",
    .probe      = _device_probe,
    .disconnect = _device_disconnect,
	//.suspend    = device_suspend,
	//.resume     = device_resume,
	//pre_reset  = device_pre_reset,
	//.post_reset = device_post_reset,
    .id_table   = g_dev_id_tab
};
static const struct file_operations _dt9836_fops = 
{
	.owner =	THIS_MODULE,
	.read =		_dt9836_read,
	//.write =	skel_write,
	.open =		_dt9836_open,
	.release =	_dt9836_release,
	//.flush =	skel_flush,
	//.llseek =	noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 * 
 * The .name device will be created under /dev. The following initialization 
 * for .name will not work
 *  /somedir/dt9836%d   => somedir is ignored and /dev/dt98360 is created
 *  dt9836-%d           => No dev node created 
 */
static struct usb_class_driver _dt9836_class = 
{
	.name =		"dt9836.%d",
	.fops =		&_dt9836_fops,
};

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


static int _dt9836_open(struct inode *p_inode, struct file *p_file)
{
	dt9836_device_ptr p_dt9836_dev;
	struct usb_interface *p_usb_if;
	int subminor;
	int err = 0;

	subminor = iminor(p_inode);

	p_usb_if = usb_find_interface(&dt9836_driver, subminor);
	if (!p_usb_if) 
    {
		pr_err("%s - ERROR, can't find device for minor %d\n",
			__func__, subminor);
		err = -ENODEV;
		goto exit;
	}

	p_dt9836_dev = usb_get_intfdata(p_usb_if);
	if (!p_dt9836_dev) 
    {
		pr_err("%s - ERROR usb_get_intfdata\n", __func__);
		err = -ENODEV;
		goto exit;
	}
#if 0
	err = usb_autopm_get_interface(p_usb_if);
	if (err)
		goto exit;
#endif

	/* increment our usage count for the device */
	kref_get(&p_dt9836_dev->kref);

	/* save our object in the file's private structure */
	p_file->private_data = p_dt9836_dev;
exit:
    pr_info("%s returned %d", __func__, err);
	return err;
}

static int _dt9836_release(struct inode *inode, struct file *p_file)
{
    dt9836_device_ptr p_dt9836_dev;
    pr_info("%s\n", __func__);
    
    p_dt9836_dev = p_file->private_data;
    if (!p_dt9836_dev)
    {
		return -ENODEV;
    }

	/* decrement the count on our device */
	kref_put(&p_dt9836_dev->kref, _dt9836_destroy);
    
	return 0;
#if 0
	struct usb_skel *dev;

	dev = p_file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, skel_delete);
#endif    
	return 0;
}

static ssize_t _dt9836_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos)
{
    pr_info("%s", __func__);
    return 0;
}

static int _firmware_load(struct usb_interface *p_usb_if)
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

/*
 * Called from the probe function to create and initialize data structs
 */
static int _dt9836_create(struct usb_interface	*p_usbif)
{
    int err = 0;
    int ep_found = 0;
    int i;
    dt9836_device_ptr p_dt9836_dev;
    struct usb_host_interface * p_if_desc;
     
    //Allocate the device structure
    p_dt9836_dev = kzalloc(sizeof(dt9836_device_t), GFP_KERNEL);
    if (!p_dt9836_dev)
    {
        dev_err(&p_usbif->dev, "%s ERROR kzalloc\n", __func__);
        return (-ENOMEM);
    }
    
    p_dt9836_dev->p_usbif = p_usbif;
    p_dt9836_dev->p_usbdev = usb_get_dev(interface_to_usbdev(p_usbif));
    kref_init(&p_dt9836_dev->kref);
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
                p_dt9836_dev->message_ep.pipe = 
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
                p_dt9836_dev->stream_wr_ep.pipe =
                        usb_sndbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_STREAM_WR);
                ++ep_found;
                break;
            case EP_ADDR_STREAM_RD: //IN
                p_dt9836_dev->stream_rd_ep.size = _usb_endpoint_maxp(p_ep_desc);
                p_dt9836_dev->stream_rd_ep.pipe = 
                        usb_rcvbulkpipe(p_dt9836_dev->p_usbdev, EP_ADDR_STREAM_RD);
                ++ep_found;
                break;
            default :   //unknown endpoint
                dev_warn(&p_usbif->dev, "%s WARNING unknown ep 0x%x\n", 
                        __func__, p_ep_desc->bEndpointAddress);
        }
    }
    if (ep_found != DT9836_NUM_ENDPOINTS) 
    {
        dev_err(&p_usbif->dev, "%s ERROR #of ep ", __func__);
        err = -EINVAL;
    }

    while (!err)
    {
#if 0        
printk("%s usb_if.dev %x\n usbdev %x\nusbdev.dev %x\np_dt9836_dev %x\n", 
        __func__, (uint32_t)&p_usbif->dev, 
       p_dt9836_dev->p_usbdev,
       (uint32_t)&p_dt9836_dev->p_usbdev->dev,
       (uint32_t)p_dt9836_dev);
#endif        
        err = dt9836_board_type_read(p_dt9836_dev, &p_dt9836_dev->board_type);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d board type", 
                    __func__, err);
            break;
        }
        
        err = dt9836_serial_num_read(p_dt9836_dev, &p_dt9836_dev->serial_num);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d read serail#", 
                    __func__, err);
            break;
        }
         
        err = dev_set_drvdata(&p_dt9836_dev->p_usbdev->dev, p_dt9836_dev);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d dev_set_drvdata\n", 
                    __func__,err);
            break;
        }
   
        /*
         * Create the sysfs files 
         */
        err = dt9836_sysfs_register(p_dt9836_dev);
        if (err)
        {
            break;
        }
        
        err = usb_register_dev(p_usbif, &_dt9836_class);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d usb_register_dev\n", 
                    __func__, err);
            break;
        }
        
        usb_set_intfdata(p_usbif, p_dt9836_dev);
        break;
    }
    
    /*
     * NOTE that any errors will prevent the device from being used and the
     * sole remedy is disconnecting it. Therefore clean up is handled in 
     * dt9836_destroy()
     */
    if (err)
    {
        usb_set_intfdata(p_usbif, NULL);
        dev_set_drvdata(&p_dt9836_dev->p_usbdev->dev, NULL);

        /* decrement our usage count */
        kref_put(&p_dt9836_dev->kref, _dt9836_destroy);
    }
    return (err);
}

static int _device_probe (struct usb_interface *p_usb_if,
                            const struct usb_device_id *p_usb_devid)
{
    int err = 0;
     
    if (p_usb_devid->idProduct == DT9836_PRE_ID)
    {
        err = _firmware_load(p_usb_if);
    }
    else
    {
        err = _dt9836_create(p_usb_if);
    }
    if (!err)
    {
    dev_info(&p_usb_if->dev,
              "%s idProduct %x bcdDevice %x:%x version \"%s\" attached to %d\n",
              __func__, 
              p_usb_devid->idProduct, 
              p_usb_devid->bcdDevice_hi, p_usb_devid->bcdDevice_lo,
              DRIVER_VERSION,p_usb_if->minor);
    }
    return (err);
}

/*
 * Free memory when the reference count goes to zero
 */
static void _dt9836_destroy(struct kref *p_kref)
{
    dt9836_device_ptr p_dt9836_dev;
    p_dt9836_dev = container_of(p_kref, dt9836_device_t, kref);
    
    //dt9836_sysfs_deregister(p_dt9836_dev);
    dev_set_drvdata(&p_dt9836_dev->p_usbdev->dev, NULL);
    usb_put_dev(p_dt9836_dev->p_usbdev);
    kfree(p_dt9836_dev);
   printk("%s\n", __func__);
}

static void _device_disconnect (struct usb_interface *p_usb_if)
{
    dt9836_device_ptr p_dt9836_dev;
    
    printk("%s\n", __func__);
    
    p_dt9836_dev = usb_get_intfdata(p_usb_if);
	
    usb_set_intfdata(p_usb_if, NULL);
	/* give back our minor */
	usb_deregister_dev(p_usb_if, &_dt9836_class);
	
    /* decrement our usage count */
	kref_put(&p_dt9836_dev->kref, _dt9836_destroy);
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


/*
 * Helper macro for USB drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 * module_init() will invoke usb_register() and module_exit() invoked
 * usb_deregister()
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
