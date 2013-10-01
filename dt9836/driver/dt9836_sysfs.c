
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include "dt9836_device.h"
#include "../include/dt9836_driver_fw_if.h"
#include "../include/dt9836_hwreg.h"

static ssize_t _devinfo_read(struct device *p_dev,
							struct device_attribute *attr, char *p_buf)
{
    int len = 0;
    
    dt9836_device_ptr p_dt9836_dev = dev_get_drvdata(p_dev);
    if (!p_dt9836_dev)
    {
        return 0;
    }
    
    len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                        "%s 0x%x\n", 
                        (p_dt9836_dev->board_info.board_type_str)?
                            p_dt9836_dev->board_info.board_type_str:
                            "unknown board type", 
                        p_dt9836_dev->board_info.board_type);
    if (p_dt9836_dev->board_info.board_type_str)
    {
        len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                            "ad=%d da=%d ctr=%d quad=%d din=%d dout=%d\n", 
                            p_dt9836_dev->board_info.num_chan_ad,
                            p_dt9836_dev->board_info.num_chan_da,
                            p_dt9836_dev->board_info.num_chan_counter,
                            p_dt9836_dev->board_info.num_chan_quadenc,
                            p_dt9836_dev->board_info.num_chan_din,
                            p_dt9836_dev->board_info.num_chan_dout);
    }
    len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                        "Serial# %d\n", 
                        p_dt9836_dev->serial_num);
    len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                        "kref %d\n", 
                     atomic_read(&p_dt9836_dev->kref.refcount));
	return (len);
}
static const DEVICE_ATTR(devinfo, 
                   S_IRUGO,
					_devinfo_read,
                   NULL);

#define ATTR_LIST(x)	& dev_attr_ ##x.attr
static const struct attribute * _p_attributes[] = 
{
    ATTR_LIST(devinfo),
    NULL
};

static const struct attribute_group _attr_group = 
{
    .name = "dt9836_debug",
	.attrs = (struct attribute **)_p_attributes,
};

int dt9836_sysfs_register(dt9836_device_ptr p_dt9836_dev)
{
    int err = 0;
    
    err = sysfs_create_group(&p_dt9836_dev->p_usbdev->dev.kobj, 
                                 &_attr_group);
    if (err != 0)
    {
        dev_err(&p_dt9836_dev->p_usbif->dev, 
                "[%s] ERROR sysfs_create_group\n", __func__);
    }
    
    return (err);
}
#if 0
void dt9836_sysfs_deregister(dt9836_device_ptr p_dt9836_dev)
{

    sysfs_remove_group(&p_dt9836_dev->p_usbdev->dev.kobj, &_attr_group);
}
#endif

