/*
 *	Data structures, macros shared between the DT9836 kernel mode USB driver
 *  and user applications
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
#ifndef DT9836_COMMON_H
#define	DT9836_COMMON_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifdef	__cplusplus
extern "C"
{
#endif
    
#define _PACK_  __attribute__ ((__packed__))
#define ENUM_MAX    (0xffffffff)
/*
 * Note that each enum has __attribute__ ((__packed__)) AND defines a maximum
 * value of 0xffffffff to ensure that the sizeof enums are always 4-bytes
 */
/*
 * Subsystem types
 */    
typedef enum _PACK_
{
   OLSS_AD,
   OLSS_DA,
   OLSS_DIN,
   OLSS_DOUT,
   OLSS_SRL,
   OLSS_QUAD = OLSS_SRL,
   OLSS_CT,
   OLSS_TACH,
   OLSS_MAX = ENUM_MAX
} OLSS;
    
/*
 * Subsystem data flow types
 */
typedef enum _PACK_
{
    OL_DF_CONTINUOUS            =800,   
    OL_DF_SINGLEVALUE           =801,
    OL_DF_DTCONNECT_CONTINUOUS  =802,   
    OL_DF_DTCONNECT_BURST       =803,  
    OL_DF_CONTINUOUS_PRETRIG    =804,   
    OL_DF_CONTINUOUS_ABOUTTRIG  =805,  
    OL_DF_MAX                   =ENUM_MAX
} OLSS_DATA_FLOW;

/*
 * Subsystem configuration
 */
typedef struct  _PACK_
{
    OLSS                subsystem_type;    //Must be first member            
    OLSS_DATA_FLOW      data_flow;
}subsystem_config_t;

/*
 * Single value type
 */
typedef struct  _PACK_
{
    OLSS                subsystem_type;     //Must be first member  
    uint32_t            channel;
    uint32_t            value;
    uint32_t            gain;
}single_value_t;

typedef uint16_t    board_type_t;

typedef struct _PACK_
{
    board_type_t  board_type;         //board type
    int8_t        num_chan_ad;        //#of AD channels
    int8_t        num_chan_da;        //#of DA channels
    int8_t        num_chan_counter;   //#of counters                 
    int8_t        num_chan_quadenc;   //#of quadrature encoders
    int8_t        num_chan_din;       //#of digital inputs
    int8_t        num_chan_dout;      //#of digital outputs
    char          board_type_str[16]; //board type string
    uint32_t      serial_num;         //serial number
}board_info_t ;

#undef _PACK_
#undef ENUM_MAX

#ifdef	__cplusplus
}
#endif

#endif	