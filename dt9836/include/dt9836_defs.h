#include "dt9836ids.h"

#ifndef _DT9836DEFS_H_
#define _DT9836DEFS_H_  

// for DT9862 only
//		set the fifo threshold to half it's sample size.
#define DT9862_FIFO_SAMPLE_SIZE		0x10000
#define DT9862_FIFO_THRESHOLD		(DT9862_FIFO_SAMPLE_SIZE >> 1)


// DT9836 IDs...these must match those found on the EEPROM on the board
//
// now in dt9836ids.h

#ifdef __cplusplus
  const int MAX_IDLE_COUNT	= 100;
  const int READ_SOFTWARE_FIFO_SIZE	= 0x1000000;				// Use this for high speed boartd test
  const int DT9836_CONFIG_NUM		= 1;
  const int READ_URB_QUEUE_DEPTH	= 16;
  const int WRITE_URB_QUEUE_DEPTH	= 8;
  const int MIN_CONTINUOUS_DATA_XFER_REQUEST_BYTES = 512;
  const int CONTINUOUS_DATA_XFER_REQUEST_ALIGNMENT = 512;
  const int DT_USBD_DT9862_MAXIMUM_TRANSFER_SIZE = 131072;
#endif __cplusplus


//#define DT_9836_REFERENCE_CLOCK		18000000  // 18MHz 
#define DT9836_AD_TIMER_TIMEOUT		20000	
#define DT9836_CTR_TIMER_TIMEOUT	20000			// in microseconds

#define NUM_FAKE_AD_CHANS		11		// 10 counter, 1 DigIn


typedef enum
{
   DT9836_HW_REV_0,  
   DT9836_HW_REV_1,
   DT9836_HW_REV_2
} DT9836_HARDWARE_REVISION;

#define TEST_BUS_FAILURE( ntStatus )         \
        do                                   \
        {                                    \
           if( !NT_SUCCESS( (ntStatus) ) )   \
           {                                 \
              return OLGENERALFAILURE;       \
           }                                 \
        } while (0)


// Product ID codes set by on board resistors mapped to the fpga VersionId register
#define DT9836_12_2		0x04	// 0x04 = 12 AD chans, w/2 DACs, 2 Ctrs, 3 Quads
#define DT9836_12_0		0x05	// 0x05 = 12 AD chans, wo/ DACs, 2 Ctrs, 3 Quads
#define DT9836_6_2		0x06	// 0x06 = 6 AD chans, w/2 DACs, 2 Ctrs, 3 Quads
#define DT9836_6_0		0x07	// 0x07 = 6 AD chans, wo/ DACs, 2 Ctrs, 3 Quads

#define DT9836_6_4		0x016	// 0x16 = 6 AD chans, w/4 DACs, 2 Ctrs, 3 Quads
#define DT9836_6_4_MC	0x8016	// 0x8016 = 6 AD chans, w/4 DACs, 2 Ctrs, 3 Quads MicroCal special.
								// Digital outputs default to high on power up

#define DT9832_BIT		0x100
#define DT9832_4_2		0x104	// 0x104 = 4 AD chans, w/ 2 DACs, 2 Ctrs, 3 Quads
#define DT9832_4_0		0x105	// 0x105 = 4 AD chans, wo/ DACs, 2 Ctrs, 3 Quads

#define DT9832A_BIT		0x200
#define DT9832A_2_2		0x202	// 0x202 = 2 AD chans, w/ 2 DACs, 2 Ctrs, 3 Quads
#define DT9832A_2_0		0x203	// 0x203 = 2 AD chans, wo/ DACs, 2 Ctrs, 3 Quads

#define DT9836S_BIT		(0x400)
#define DT9836S_6_2		(DT9836S_BIT | 0x06)	//6 AD, 2 DACs, 2 Ctrs, 3 Quads
#define DT9836S_6_0		(DT9836S_BIT | 0x07)	//6 AD , no DACs, 2 Ctrs, 3 Quads

#define DT9836_INVALID_TYPE	0xffff

#define VERSION_ID_MASK	0x0F00					// bits<11-8>
#define DT9862_BIT		0x0800					// bit<11> = '1' for 9862 flavors
#define DT9862S_BIT		0x0900					// bit<11..8> = '1001' for 9862S
#define DT9862_2_2		(DT9862_BIT | 0x0002)	// 16 bits, 2 DACs, 2 counter/timers, 3 quadrature
#define DT9862_2_0		(DT9862_BIT | 0x0003)	// 16 bits, 0 DACs, 2 counter/timers, 3 quadrature
#define DT9862S_2_0		(DT9862S_BIT | 0x0003)	// 16 bits, 0 DACs, 2 counter/timers, 3 quadrature

// NOTE 1: Various bits define the boards used. But no bits also defines a board; so be very careful here.
// NOTE 2: Some boards have 0, 1, or 2 bits set.
//
// (none> == DT9836
//    <8> == DT9832
//    <9> == DT9832A
//   <10> == DT9836S
//   <11> == DT9862
//   <11> & <8> == DT9862S

		// both 9832 and 9832A
#define BOARD_TYPE_9832_series(x)		\
	(((x & VERSION_ID_MASK) == DT9832_BIT) || ((x & VERSION_ID_MASK) == DT9832A_BIT))

		// both 9862 and 9862S
#define BOARD_TYPE_9862_series(x)	(x & DT9862_BIT)

		// only 9862S
#define BOARD_TYPE_9862S(x)			((x & VERSION_ID_MASK) == DT9862S_BIT)

		// only 9862
#define BOARD_TYPE_9862(x)			((BOARD_TYPE_9862_series(x)) && (!BOARD_TYPE_9862S(x)))

		// both 9836 and 9836S
#define BOARD_TYPE_9836(x)			((!BOARD_TYPE_9832_series(x)) && (!BOARD_TYPE_9862_series(x)))

#endif  // _DT9836DEFS_H_ 
