/*
 *	Driver for DT9836 USB data acquisition module.
 *  This file has the declarations for the digital input subsystem
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
#ifndef DT9836_DIN_H
#define	DT9836_DIN_H

#include "dt9836_device.h"
#ifdef	__cplusplus
extern "C"
{
#endif

int din_set_config(struct dt9836_device *, const subsystem_config_t *);
int din_get_single(struct dt9836_device *, single_value_t *); 

#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_DIN_H */

