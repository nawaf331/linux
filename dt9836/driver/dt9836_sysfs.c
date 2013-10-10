
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include "dt9836_device.h"
#include "../include/dt9836_driver_fw_if.h"
#include "../include/dt9836_hwreg.h"

#define HW_REG \
_(OUTPUT_CONTROL)       __DELIM__ \
_(INPUT_CONTROL0)       __DELIM__ \
_(IO_OP_CONTROL)        __DELIM__ \
_(INT_STATUS)           __DELIM__ \
_(INT_MASK)             __DELIM__ \
_(COUNTER_INT_MASK)     __DELIM__ \
_(DIG_IN_DATA)          __DELIM__ \
_(DIG_IN_INT_DATA)      __DELIM__ \
_(DIG_IN_INT_MASK)      __DELIM__ \
_(DIG_IN_CTRL_STAT)     __DELIM__ \

#undef  __DELIM__
#define __DELIM__ ,
#undef  _
#define _(reg) {reg, #reg}

static const struct
{
    uint16_t        reg_addr;
    const char *    reg_name_ptr;
}g_reg_tab[]=
{
    HW_REG
    {0, NULL}
};

#define SIZEOF_REG_TAB  (sizeof(g_reg_tab)/sizeof(g_reg_tab[0]) -1)
/*
 * Reads all 16-bit registers from the board
 */
static ssize_t _sysfs_reg_read(struct device *p_dev,
							struct device_attribute *attr, char *p_buf)
{
    int len = 0;
    int i, j;
    USB_CMD cmd;
   
    dt9836_device_ptr p_dt9836_dev = dev_get_drvdata(p_dev);
    if (!p_dt9836_dev)
    {
        return 0;
    }
    
    cmd.CmdCode = R_MULTI_WORD_LB; //read array of regs instead of one at a time
    for (i=0; i < SIZEOF_REG_TAB; )
    {
        int err;
        int i_save = i;
        for (j=0; (j < (SIZEOF_REG_TAB -i)) && (j < MAX_NUM_MULTI_WORD_RDS);
             ++j, ++i)
        {
            cmd.d.ReadMultiWordInfo.Addresses[j] = g_reg_tab[i].reg_addr;
        }
        cmd.d.ReadMultiWordInfo.NumReads = j;
        
        //Write the command and read back reg values into the same array as the
        //register addresses. This avoids allocating another array
        err = dt9836_command_write_read(p_dt9836_dev, &cmd, 
                                    (void *)cmd.d.ReadMultiWordInfo.Addresses,
                                    cmd.d.ReadMultiWordInfo.NumReads * sizeof(uint16_t));
        if (!err)
        {
            for (j=0; j < cmd.d.ReadMultiWordInfo.NumReads; ++j, ++i_save)
            {
                int strln = strlen(g_reg_tab[i_save].reg_name_ptr);
                len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                                 "[%#04x] %s%c%#06x\n", 
                                 g_reg_tab[i_save].reg_addr,
                                  g_reg_tab[i_save].reg_name_ptr, 
                                 (strln<16)?'\t':' ',
                                 cmd.d.ReadMultiWordInfo.Addresses[j]);
            }
        }
        else
        {
            len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                             "ERROR reading regs\n");
            break;
        }
    }
	return (len);
}

/*
 * Returns the device's static information
 */
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
    len += scnprintf(p_buf +len , PAGE_SIZE-len, 
                        "Serial# %d\n", 
                        p_dt9836_dev->board_info.serial_num);
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
                        "kref %d\n", 
                     atomic_read(&p_dt9836_dev->kref.refcount));
	return (len);
}

static const DEVICE_ATTR(devinfo, 
                   S_IRUGO,
					_devinfo_read,
                   NULL);

static const DEVICE_ATTR(reg, 
                   S_IRUGO,
					_sysfs_reg_read,
                   NULL);

#define ATTR_LIST(x)	& dev_attr_ ##x.attr
static const struct attribute * _p_attributes[] = 
{
    ATTR_LIST(devinfo),
    ATTR_LIST(reg),
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

