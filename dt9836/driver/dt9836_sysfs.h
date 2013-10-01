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

#ifndef DT9836_SYSFS_H
#define	DT9836_SYSFS_H
#include <linux/usb.h>

#include "./dt9836_device.h"

#ifdef	__cplusplus
extern "C"
{
#endif

int dt9836_sysfs_register(dt9836_device_ptr p_dev);

#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_SYSFS_H */

