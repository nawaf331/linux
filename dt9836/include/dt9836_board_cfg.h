
#ifndef DT9836_IOCTL_H
#define	DT9836_IOCTL_H

#include <linux/types.h>

#ifdef	__cplusplus
extern "C"
{
#endif

typedef __u16    board_type_t;

struct board_info_t
{
    board_type_t    board_type;         //board type
    const char      *board_type_str;    //board type string
    int             num_chan_ad;        //#of AD channels
    int             num_chan_da;        //#of DA channels
    int             num_chan_counter;   //#of counters                 
    int             num_chan_quadenc;   //#of quadrature encoders
    int             num_chan_din;       //#of digital inputs
    int             num_chan_dout;      //#of digital outputs
};
typedef struct board_info_t board_info_t;
    
#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_BRDCFG_H */