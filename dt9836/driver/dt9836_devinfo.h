/* 
 * File:   dt9836_devinfo.h
 * Author: root
 *
 * Created on October 1, 2013, 1:13 PM
 */

#ifndef DT9836_DEVINFO_H
#define	DT9836_DEVINFO_H

#ifdef	__cplusplus
extern "C"
{
#endif

/*
 * Table definition for board types and board information. Each line lists
 * board type, board type number, #of AD, #of DA, #of Counter, #of quads,
 * #of digital in, #of digital out
 */

#define DEV_INFO \
_(DT9836_12_2,  0x04,   12, 2,  2,  3,  0,  0 ) __DELIM__ \
_(DT9836_12_0,	0x05,   12, 0,  2,  3,  0,  0  ) __DELIM__ \
_(DT9836_6_2,	0x06,   6,  2,  2,  3,  0,  0  ) __DELIM__ \
_(DT9836_6_0,	0x07,   6,  0,  2,  3,  0,  0  ) __DELIM__ \
_(DT9836_6_4,	0x016,  6,  4,  2,  3,  0,  0 ) __DELIM__ \
_(DT9836_6_4_MC,0x8016, 6,  4,  2,  3,  0,  0 ) __DELIM__ \
_(DT9832_4_2,   0x104,  4,  2,  2,  3,  0,  0 ) __DELIM__ \
_(DT9832_4_0,   0x105,  4,  0,  2,  3,  0,  0 ) __DELIM__ \
_(DT9832A_2_2,  0x202,  2,  2,  2,  3,  0,  0 ) __DELIM__ \
_(DT9832A_2_0,  0x203,  2,  0,  2,  3,  0,  0 ) __DELIM__ \
_(DT9836S_6_2,  0x406,  6,  2,  2,  3,  0,  0 ) __DELIM__ \
_(DT9836S_6_0,  0x407,  6,  0,  2,  3,  0,  0 ) __DELIM__ \


#ifdef	__cplusplus
}
#endif

#endif	/* DT9836_DEVINFO_H */

