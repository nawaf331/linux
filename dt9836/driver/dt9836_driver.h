/*
 *  DT9836 driver
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

#ifndef DT98xx_DRIVER
#define	DT98xx_DRIVER

#ifdef	__cplusplus
extern "C" {
#endif
#define DRIVER_AUTHOR	"Data Translation Inc."
#define DRIVER_DESC	"DT9836 driver"
#define DRIVER_ALIAS    "dt9836"
#define DATX_VENDOR_ID	(0x0867)
#define DT9836_PRE_ID   (0x9836)        //Product id *before* firmware download

//TO_DO  move to version.h file
#define DRIVER_VERSION  "0.0.0.1"

/*
 * Table of USB product ids handled by this driver
 * Each entry has the following parameters.
 * 1) idProduct of USB Device Descriptor handled by this driver
 */
#define DT9836_TAB      \
_(DT9836_PRE_ID         ) __DELIM__ \
_(0x3698                ) __DELIM__ \
    
#ifdef	__cplusplus
}
#endif

#endif	/* DT98xx_DRIVER */

