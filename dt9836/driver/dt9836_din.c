/*
 *	Driver for DT9836 USB data acquisition module.
 *  This file has the definitions for the digital input subsystem
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
#include "../include/dt9836_hwreg.h"

#include "dt9836_din.h"

/*
 * Configure the digital input subsystem
 */
int din_set_config(struct dt9836_device * p_dt9836_dev,
                   const subsystem_config_t *config_ptr)
{
    int err = 0;
    da_subsystem_t *p_sub = &p_dt9836_dev->da_subsystem[DT9836_DIN_SUBSYSTEM];    
    
    //Check data flow
    if ((config_ptr->data_flow != OL_DF_CONTINUOUS) &&
        (config_ptr->data_flow != OL_DF_SINGLEVALUE))
    {
        dev_err(&p_dt9836_dev->p_usbif->dev,
            "%s: ERROR dataflow=%d\n",__func__,config_ptr->data_flow);
        err = (-EINVAL);
    }
    else
    {
#warning TBD more checks here
        
        //Save the configuration
        spin_lock(&p_dt9836_dev->da_subsystem_spinlock);
        memcpy(&p_sub->config, config_ptr, sizeof(subsystem_config_t));
        spin_unlock(&p_dt9836_dev->da_subsystem_spinlock);
    }
    
    return (err);
}

/*
 * Read all digital inputs
 */
int din_get_single(struct dt9836_device *p_dt9836_dev, 
                   single_value_t * single_value_ptr)
{
    int err = 0;
    //da_subsystem_t *p_sub = &p_dt9836_dev->da_subsystem[DT9836_DIN_SUBSYSTEM]; 
    
    single_value_ptr->value = 0;
    err = dt9836_reg_read(p_dt9836_dev, DIG_IN_DATA, 
                          (uint16_t *)&single_value_ptr->value);
    return (err);
}




