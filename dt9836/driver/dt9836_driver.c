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
#include <linux/mutex.h>
#include <asm-generic/errno.h>

#include "dt9836_driver.h"
#include "dt9836_device.h"
#include "dt9836_sysfs.h"
#include "dt9836_din.h"

#include "../include/dt9836_ioctl.h"
#include "../include/dt9836_hwreg.h"

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
//static ssize_t _dt9836_read(struct file *file, char *buffer, size_t count,
//			 loff_t *ppos);
static long _dt9836_ioctl(struct file *file, unsigned int cmd,
                          unsigned long arg);
static long _dt9836_get_config(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg);
static long _dt9836_set_config(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg);
static long _dt9836_get_single(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg);

static int _device_probe (struct usb_interface *p_usb_if,
                            const struct usb_device_id *p_usb_devid);
static void _device_disconnect (struct usb_interface *p_usb_if);
static int  _dt9836_create(struct usb_interface	*p_usbif);
static void _dt9836_destroy(struct kref *p_kref);
static void _dt9836_subsystems_init(dt9836_device_ptr p_dt9836_dev);


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

static struct usb_driver g_dt9836_driver = 
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
	//.read =		_dt9836_read,
	//.write =	skel_write,
	.open =		_dt9836_open,
	.release =	_dt9836_release,
	//.flush =	skel_flush,
	//.llseek =	noop_llseek,
    .unlocked_ioctl = _dt9836_ioctl,
};

/*
 * Mutex to serialize kref_get and kref_get. This is to abide by rule 3 in
 * Documentation/kref.txt
 * If the code attempts to gain a reference to a kref-ed structure
   without already holding a valid pointer, it must serialize access
   where a kref_put() cannot occur during the kref_get(), and the
   structure must remain valid during the kref_get().
 */
static DEFINE_MUTEX(g_mutex);

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core. The %d is the minor
 * number assigned
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

/*
 * Initialize all members in da_subsystem[]  
 */
static void _dt9836_subsystems_init(dt9836_device_ptr p_dt9836_dev)
{
    int i;
    da_subsystem_t *p_sub;
    //Configuration parameters
    for (i=0, p_sub = p_dt9836_dev->da_subsystem ; 
         i < DT9836_MAX_SUBSYSTEM; 
         ++i, ++p_sub)
    {
        p_sub->config.data_flow = OL_DF_SINGLEVALUE;
    }
    
    //DIN subsystem
    p_sub = &p_dt9836_dev->da_subsystem[DT9836_DIN_SUBSYSTEM];
    p_sub->config.subsystem_type = OLSS_DIN;
    p_sub->set_config = din_set_config;
    p_sub->get_single = din_get_single;
}

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

/*
 * Handler for IOCTL_READ_SUBSYS_CFG
 */
static long _dt9836_get_config(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg)
{
    OLSS olss;
    int i;
    long retval = 0;
    const subsystem_config_t *cfg_ptr = NULL;
    
    //Get the subsystem type in the icotl params
    if (get_user(olss, (OLSS __user*)arg))
    {
        return (-EFAULT);
    }
    /*Iterate over all subsystems and get the config for the
      specified one */
    for (i=0; i < DT9836_MAX_SUBSYSTEM; ++i)
    {
        if (p_dt9836_dev->da_subsystem[i].config.subsystem_type==olss)
        {
            cfg_ptr = &p_dt9836_dev->da_subsystem[i].config;
            break;
        }
    }
    //Bad subsystem
    if (!cfg_ptr)
    {
        return (-EINVAL);
    }
    
    spin_lock(&p_dt9836_dev->da_subsystem_spinlock);
    if (copy_to_user((void __user *)arg,  (void *)cfg_ptr,
                     sizeof(subsystem_config_t))) 
    {
        retval = (-EFAULT);
    }                           
    spin_unlock(&p_dt9836_dev->da_subsystem_spinlock);
    
    return (retval);
}


/*
 * Handler for IOCTL_WRITE_SUBSYS_CFG
 */
static long _dt9836_set_config(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg)
{
    OLSS olss;
    int i;
    long retval = -EINVAL; //default error
    
    //Get the subsystem type in the icotl params
    if (get_user(olss, (OLSS __user*)arg))
    {
        return (-EFAULT);
    }
    /*Iterate over all subsystems and set the config for the
      specified one */
    for (i=0; i < DT9836_MAX_SUBSYSTEM; ++i)
    {
        if (p_dt9836_dev->da_subsystem[i].config.subsystem_type == olss)
        {
            if (p_dt9836_dev->da_subsystem[i].set_config)
            {
                subsystem_config_t cfg;
                if (copy_from_user((void *)&cfg, (void __user *)arg,  
                                    sizeof(subsystem_config_t))) 
                {
                    retval = -EFAULT;
                }                           
                else
                {
                    retval = p_dt9836_dev->da_subsystem[i].set_config
                                ((struct dt9836_device *)p_dt9836_dev, &cfg);
                }
            }
            else
            {
                retval = -ENOSYS; /* 38 Function not implemented */
            }
            break;
        }
    }

    return (retval);
}

/*
 * Handler for IOCTL_GET_SINGLE_VALUE
 */
static long _dt9836_get_single(dt9836_device_ptr p_dt9836_dev,
                               unsigned long arg)
{
    OLSS olss;
    int i;
    long retval = -EINVAL; //default error
    
    //Get the subsystem type in the icotl params
    if (get_user(olss, (OLSS __user*)arg))
    {
        return (-EFAULT);
    }
    /*
     * Iterate over all subsystems and get the single value from the specified
     * one 
     */
    for (i=0; i < DT9836_MAX_SUBSYSTEM; ++i)
    {
        if (p_dt9836_dev->da_subsystem[i].config.subsystem_type == olss)
        {
            if (p_dt9836_dev->da_subsystem[i].get_single)
            {
                single_value_t single_val;
                if (copy_from_user((void *)&single_val, (void __user *)arg,  
                                    sizeof(single_value_t))) 
                {
                    retval = -EFAULT;
                }                           
                else
                {
                    retval = p_dt9836_dev->da_subsystem[i].get_single
                                ((struct dt9836_device *)p_dt9836_dev, &single_val);
                    if (!retval)
                    {
                        if (copy_to_user((void __user *)arg,(void *)&single_val,   
                                           sizeof(single_value_t))) 
                        {
                            retval = -EFAULT;
                        }                           
                    }
                }
            }
            else
            {
                retval = -ENOSYS; /* 38 Function not implemented */
            }
            break;
        }
    }

    return (retval);
}

/*
 * IOCTL handlers
 */
static long _dt9836_ioctl(struct file *p_file, unsigned int cmd,
                          unsigned long arg)
{
    dt9836_device_ptr p_dt9836_dev;
    long retval = 0;
     
    p_dt9836_dev = p_file->private_data;
    if (!p_dt9836_dev)
    {
		return (-ENODEV);
    }
	
    dev_info(&p_dt9836_dev->p_usbif->dev,
		"%s: cmd=0x%x (nr=%d len=%d dir=%d)\n", 
        __func__, cmd,_IOC_NR(cmd), _IOC_SIZE(cmd), _IOC_DIR(cmd));

    //Prevent any commands if the device is disconnected
    if (atomic_read (&p_dt9836_dev->connected) != USB_CONNECTED)
    {
        retval = -ENOTCONN;
    }
    else
    {
        switch (cmd) 
        {
            case IOCTL_READ_BOARD_INFO:
            {
                if (copy_to_user((void __user *)arg, 
                                 (unsigned char *)&p_dt9836_dev->board_info,
                                  _IOC_SIZE(cmd))) 
                {
                    retval = -EFAULT;
                }          
            }
            break;
            
            case IOCTL_READ_SUBSYS_CFG:
            {
                retval = _dt9836_get_config(p_dt9836_dev, arg);
            }
            break;
            
            case IOCTL_WRITE_SUBSYS_CFG:
            {
                retval = _dt9836_set_config(p_dt9836_dev, arg);
            }
            break;
            
            case IOCTL_GET_SINGLE_VALUE:
            {
                retval = _dt9836_get_single(p_dt9836_dev, arg);
            }
            break;
            
            default:
                retval = -EBADRQC;   /*56 Invalid request code */
                break;
        }
    }
    
    if (retval)
    {
       dev_err(&p_dt9836_dev->p_usbif->dev,"%s ERROR %d\n", 
               __func__, (unsigned int)retval);
    }
	return retval;
}

static int _dt9836_open(struct inode *p_inode, struct file *p_file)
{
	dt9836_device_ptr p_dt9836_dev;
	struct usb_interface *p_usb_if;
	int subminor;
	int err = 0;

	subminor = iminor(p_inode);

	p_usb_if = usb_find_interface(&g_dt9836_driver, subminor);
	if (!p_usb_if) 
    {
		pr_err("%s - ERROR, can't find device for minor %d\n",
			__func__, subminor);
		return (-ENODEV);
	}

    mutex_lock(&g_mutex); //Rule #3 of kref_get/put
    
	p_dt9836_dev = usb_get_intfdata(p_usb_if);
	if (!p_dt9836_dev) 
    {
		pr_err("%s - ERROR usb_get_intfdata\n", __func__);
		return (-ENODEV);
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
    
    mutex_unlock(&g_mutex);    //Rule #3 of kref_get/put

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
#if 0
static ssize_t _dt9836_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos)
{
    pr_info("%s", __func__);
    return 0;
}
#endif
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
    spin_lock_init(&p_dt9836_dev->da_subsystem_spinlock);
    
#warning Are shadow regs needed??    
    //Initialize the shadow registers
    p_dt9836_dev->shadow_reg.output_control = DAC3_CLEAR_N | DAC2_CLEAR_N |
                                              DAC1_CLEAR_N | DAC0_CLEAR_N |
                                              DAC_RESET_ALL_N;
    p_dt9836_dev->shadow_reg.in_control = 0;
    
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
        dev_err(&p_usbif->dev, "%s ERROR #of ep %d", __func__, ep_found);
        err = -EINVAL;
    }

    while (!err)
    {
        board_type_t board_type;
#if 0        
printk("%s usb_if.dev %x\n usbdev %x\nusbdev.dev %x\np_dt9836_dev %x\n", 
        __func__, (uint32_t)&p_usbif->dev, 
       p_dt9836_dev->p_usbdev,
       (uint32_t)&p_dt9836_dev->p_usbdev->dev,
       (uint32_t)p_dt9836_dev);
#endif        
        //Read the board type
        err = dt9836_reg_read(p_dt9836_dev, VERSION_ID, &board_type);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d board type", 
                    __func__, err);
            break;
        }
        
        //Get the board's static configuration corresponding to the board type
        err = dt9836_board_info_get(board_type, &p_dt9836_dev->board_info);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d dt9836_board_info_get", 
                    __func__, err);
            break;
        }
        
        //Read the serial number
        err = dt9836_serial_num_read(p_dt9836_dev, 
                                     &p_dt9836_dev->board_info.serial_num);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d read serail#", 
                    __func__, err);
            break;
        }
        
        //Initialize a bunch of h/w registers
        err = dt9836_hw_init(p_dt9836_dev);
        if (err)
        {
            dev_err(&p_usbif->dev, "%s ERROR %d device register init\n", 
                    __func__,err);
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
        
        //Initialize all subsystem's configuration
        _dt9836_subsystems_init(p_dt9836_dev);
        
        //Flag the device as being connected now
        atomic_set(&p_dt9836_dev->connected, USB_CONNECTED);
        
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
              "%s idProduct %x bcdDevice %x:%x version \"%s\" minor# %d\n",
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
    
    mutex_lock(&g_mutex);
    p_dt9836_dev = usb_get_intfdata(p_usb_if);
	
    usb_set_intfdata(p_usb_if, NULL);
	/* give back our minor */
	usb_deregister_dev(p_usb_if, &_dt9836_class);
            
    //Flag the device as disconnected now
    atomic_set(&p_dt9836_dev->connected, (!USB_CONNECTED));
	
    /* decrement our usage count */
	kref_put(&p_dt9836_dev->kref, _dt9836_destroy);
    mutex_unlock(&g_mutex);
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

module_driver (g_dt9836_driver, usb_register, usb_deregister);

MODULE_AUTHOR       (DRIVER_AUTHOR);
MODULE_DESCRIPTION  (DRIVER_DESC);
MODULE_VERSION      (DRIVER_VERSION);
MODULE_ALIAS        (DRIVER_ALIAS);
MODULE_LICENSE      ("GPL");

//Cypress vendor command 0xA3 handler to write to external RAM
MODULE_FIRMWARE(A3LOAD_FW);
//Module firmware downloaded for product id DT9836_PRE_ID
MODULE_FIRMWARE(DT9836_FW);
