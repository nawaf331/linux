#ifndef _FIRMWARETODRIVER_H_
#define _FIRMWARETODRIVER_H_
/******************************************************************************
 Copyright 2004 by Data Translation Incorporated.
 All rights reserved. Property of Data Translation Incorporated.
 Restricted rights to use, duplicate or disclose this code are
 granted through contract.

 Description : Data structures and macros used to communicate between the
               Windows Device Driver and the USB firmware in the board.

 ******************************************************************************/

/********************************************************************
  NOTE : "Each enumerated type is represented by an implementation defined
        integer type and is compatible with that type" [ref. C - A Reference
        Manual by Harbison & Steele].
        In the Keil C51 compiler an integer is 2 bytes. However,  if an enum
        can be represented with an 8 bit value, the enum is also stored in a
        single byte. [ref. Keil Software Cx51 Compiler User�s Guide pg 177]

        Therefore, while compiling using the Keil C51 compiler, the lowest
        value of any enumerated type should be less than 255. Structures
        that have a member of an enumerated type, should have a 3-byte padding
        following that member.

        While compiling for Windows, structure members must be alligned to
        1-byte.

 ********************************************************************/


// If building windows driver, force structures to be packed to a byte
//boundary
#ifdef WIN_DRIVER
#include "AppDefs.h"
#pragma pack (push, 1)
#define _PACK_
#else
#define _PACK_  __attribute__ ((__packed__))
#endif


//
// This structure converts a 4 byte array to unsigned long
// to ease conversion between big and little endian values
//
typedef union _DWORD_STRUCT 
{
   uint8_t  ByteVals[4];
   uint32_t DWordVal;
} DWORD_STRUCT, *PDWORD_STRUCT;



/********************************************************************


   USB Pipe Definitions

 ********************************************************************/

typedef enum _PACK_
{
   READ_MSG_PIPE,           // Async messages to driver
   WRITE_CMD_PIPE,          // Driver sends commands to firmware
   READ_CMD_PIPE,           // Firmware returns data in response to cmd
   WRITE_STREAM_PIPE,       // DAC output stream
   READ_STREAM_PIPE         // AD input stream
 } DT9836_PIPES;

#define DT9836_NUM_ENDPOINTS	 (5)

// Maximum size of packets on the WRITE_CMD_PIPE
#define MAX_WRITE_CMD_PIPE_SIZE     (64)
// Maximum size of packets on the READ_MSG_PIPE
#define MAX_READ_MSG_PIPE_SIZE       (64)

/******************************************************************

  USB_FIRMWARE_CMD_CODE - defines all of the commands that the DT9810
  firmware implements

  Each command has its own associated structure that contains the
  data associated with that particular command.

  The firmware sends back data for some commands (the R_ commands).
  In those cases the data is returned on the COMMAND_READ_PIPE.

  Important note:
  Each enumerated type number should not be changed since its used
  by jump tables. Any added item should be inserted in the bottom 
  ( and above MAX_USB_FIRMWARE_CMD_CODE) with a number associated with
  it.

  *****************************************************************/

typedef enum _PACK_
{
   LEAST_USB_FIRMWARE_CMD_CODE = 0,

         // USB registers read/write commands
   R_BYTE_USB_REG = 0,			// Read a single byte of USB memory
   W_BYTE_USB_REG = 1,			// Write a single byte of USB memory
   R_MULTI_BYTE_USB_REG = 2,	// Multiple Reads of USB memory
   W_MULTI_BYTE_USB_REG = 3,	// Multiple Writes of USB memory
   RMW_BYTE_USB_REG = 4,		// Read, (AND) with mask, OR value, then write (single)
   RMW_MULTI_BYTE_USB_REG = 5,	// Read, (AND) with mask, OR value, then write (multiple)

      // I2C Register read/write commands  
   R_BYTE_I2C_REG = 10,			// Read a single byte of an I2C device
   W_BYTE_I2C_REG = 11,			// Write a single byte of an I2C device
   R_MULTI_BYTE_I2C_REG = 12,	// Multiple Reads of a device
   W_MULTI_BYTE_I2C_REG = 13,	// Multiple Writes of a device

       // read/write commands for Local Bus
   R_SINGLE_WORD_LB = 20,		// Read single word of registers mapped through the local bus
   W_SINGLE_WORD_LB = 21,       // Write a single word of registers mapped through the local bus
   R_MULTI_WORD_LB = 22,        // Multiple Reads of registers mapped through the local bus
   W_MULTI_WORD_LB = 23,        // Multiple Writes of registers mapped through the local bus
   RMW_SINGLE_WORD_LB = 24,     // Read, (AND) with mask, OR value, then write (single word)
   RMW_MULTI_WORD_LB = 25,      // Read, (AND) with mask, OR value, then write (multiple word)

   // Subsystem commands
   START_SUBSYSTEM = 30,		// Issue a start command to a given subsystem 
   STOP_SUBSYSTEM = 31,         // Issue a stop command to a given subsystem 
   R_SINGLE_VALUE_CMD = 32,		// Read a single value from a subsystem
   W_SINGLE_VALUE_CMD = 33,		// Write a single value to a subsystem
   R_SINGLE_VALUES_CMD = 34,	// Read a single value from all chans in subsystem

	// Miscellaneous
   W_DAC_THRESHOLD = 40,		// Not sure if we'll need this
   W_DAC_FIFO_SIZE = 41,		// set the DAC FIFO size
   W_ADC_FIFO_SIZE = 42,		// set the DAC FIFO size
   W_INT_ON_CHANGE_MASK = 43,	// Set interrupt on change mask
   START_8051_DEBUGGER = 44,	// Start the debugger on the USB processor

   // Calibration Pots
   WRITE_CAL_POT = 50,			// Write to a calibration pot
   READ_CAL_POT = 51,			// Read a calibration pot             

   MAX_USB_FIRMWARE_CMD_CODE    // Valid USB_FIRMWARE_CMD_CODE's will be less than
                                // this number
 } USB_FIRMWARE_CMD_CODE;



 /******************************************************************************
 Structures used for I/O to USB registers 
 ******************************************************************************/ 

/* This structure is used with the W_BYTE_USB_REG */
typedef struct _PACK_ 
{
   uint8_t             Address;
   uint8_t             DataVal;
} WRITE_BYTE_INFO, *pWRITE_BYTE_INFO;


/* This structure is used with the R_BYTE_USB_REG */
typedef struct _PACK_ 
{
   uint8_t             Address;
} READ_BYTE_INFO, *pREAD_BYTE_INFO;


#define MAX_NUM_MULTI_BYTE_WRTS  (( MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (WRITE_BYTE_INFO))
/* This structure is used with the R_MULTI_BYTE_USB_REG */
typedef struct _PACK_ 
{
   uint8_t             NumWrites;
   WRITE_BYTE_INFO  Write[MAX_NUM_MULTI_BYTE_WRTS];
} WRITE_MULTI_INFO, *pWRITE_MULTI_INFO;


#define MAX_NUM_MULTI_BYTE_RDS  ((MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (uint8_t))
/* This structure is used with the R_MULTI_BYTE_USB_REG */
typedef struct _PACK_ 
{
   uint8_t             NumReads;
   uint8_t             Addresses[MAX_NUM_MULTI_BYTE_RDS];
} READ_MULTI_INFO, *pREAD_MULTI_INFO;



/* This structure is used with the RMW_BYTE_USB_REG,
   and RMW_MULTI_BYTE_USB_REG */
typedef struct _PACK_ 
{
   uint8_t             Address;
   uint8_t             AndMask;
   uint8_t             OrVal;
} RMW_BYTE_INFO, *pRMW_BYTE_INFO;

   /* This structure is used with the RMW_MULTI_BYTE_USB_REG and
   RMW_MULTI_BYTE_IOP_REG  */
#define MAX_NUM_MULTI_BYTE_RMWS  ((MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (RMW_BYTE_INFO))
typedef struct _PACK_ 
{
   uint8_t             NumRMWs;
   RMW_BYTE_INFO    ByteInfo[MAX_NUM_MULTI_BYTE_RMWS];
} RMW_MULTI_INFO, *pRMW_MULTI_INFO;



 /******************************************************************************
 Structures used for I/O to I2C devices
 ******************************************************************************/ 

/* This structure is used with the W_BYTE_I2C_REG */
typedef struct _PACK_ 
{
   uint8_t        DevAddress;    // Device address
   uint8_t        Register;      // A register or entity within the device
   uint8_t        DataVal;       // Data value to write to the Register
} WRITE_I2C_BYTE_INFO, *pWRITE_I2C_BYTE_INFO;

/* This structure is used with the R_BYTE_I2C_REG */
typedef struct _PACK_
{
   uint8_t        DevAddress;    // Device address
   uint8_t        Register;      // A register or entity within the device
} READ_I2C_BYTE_INFO, *pREAD_I2C_BYTE_INFO;

#define MAX_NUM_I2C_MULTI_BYTE_WRTS  (( MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (WRITE_I2C_BYTE_INFO))

/* This structure is used with the W_MULTI_BYTE_I2C_REG */
typedef struct _PACK_
{
   uint8_t             NumWrites;
   WRITE_I2C_BYTE_INFO  Write[MAX_NUM_I2C_MULTI_BYTE_WRTS];
} WRITE_I2C_MULTI_INFO, *pWRITE_I2C_MULTI_INFO;

#define MAX_NUM_I2C_MULTI_BYTE_RDS  (( MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (READ_I2C_BYTE_INFO))

/* This structure is used with the R_MULTI_BYTE_I2C_REG */
typedef struct _PACK_
{
   uint8_t             NumReads;
   READ_I2C_BYTE_INFO  Read[MAX_NUM_I2C_MULTI_BYTE_RDS];
} READ_I2C_MULTI_INFO, *pREAD_I2C_MULTI_INFO;




 /******************************************************************************
 Structures used for I/O to device registers on Local Bus
 ******************************************************************************/ 

/* This structure is used with the R_SINGLE_WORD_LB */

typedef struct _PACK_
{
   uint16_t             Address;
   uint16_t             DataVal;
} WRITE_WORD_INFO, *pWRITE_WORD_INFO;

/* This structure is used with the W_SINGLE_WORD_LB */

typedef struct _PACK_
{
   uint16_t             Address;
} READ_WORD_INFO, *pREAD_WORD_INFO;

#define MAX_NUM_MULTI_WORD_WRTS  (( MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (WRITE_WORD_INFO))
/* This structure is used with the W_MULTI_WORD_LB */

typedef struct _PACK_
{
   uint8_t             NumWrites;
   WRITE_WORD_INFO  Write[MAX_NUM_MULTI_WORD_WRTS];
} WRITE_MULTI_WORD_INFO, *pWRITE_MULTI_WORD_INFO;


#define MAX_NUM_MULTI_WORD_RDS  ((MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (uint16_t))
/* This structure is used with the R_MULTI_BYTE_REG */

typedef struct _PACK_
{
   uint8_t             NumReads;
   uint16_t             Addresses[MAX_NUM_MULTI_WORD_RDS];
} READ_MULTI_WORD_INFO, *pREAD_MULTI_WORD_INFO;

/* This structure is used with the RMW_SINGLE_WORD_LB,
   and RMW_MULTI_WORD_LB */

typedef struct _PACK_
{
   uint16_t             Address;
   uint16_t             AndMask;
   uint16_t             OrVal;
} RMW_WORD_INFO, *pRMW_WORD_INFO;

   /* This structure is used with the RMW_MULTI_WORD_LB and
   RMW_MULTI_BYTE_IOP_REG  */

#define MAX_NUM_MULTI_WORD_RMWS  ((MAX_WRITE_CMD_PIPE_SIZE - 4 - 1) / sizeof (RMW_WORD_INFO))//
typedef struct _PACK_
{
   uint8_t             NumRMWs;
   RMW_WORD_INFO    WordInfo[MAX_NUM_MULTI_WORD_RMWS];
} RMW_MULTI_WORD_INFO, *pRMW_MULTI_WORD_INFO;




 /******************************************************************************
 Structures used for subsystem commands
 ******************************************************************************/ 

//  This enum is a mirror image of OLSS enumerated type declared in oldacfg.h
 typedef enum 
{
   SS_AD,
   SS_DA,
   SS_DIN,
   SS_DOUT,
   SS_SRL,
   SS_CT
} SUBSYSTEM_TYPE;


// Structure used by board drivers to pass Subsystem information to the firmware.
typedef struct _PACK_
{
   SUBSYSTEM_TYPE    SubsystemType; // Specifies  the subsystem type
   
   // If compiling USB firmware then add filler since Windows compiler
   // uses 4 byte enums rather than Keil's 1 byte
   #ifndef WIN_DRIVER
   uint8_t                  Filler[3];
   #endif
   uint16_t             ExtTrig;          // board/subsystem  specific flags

} SUBSYSTEM_INFO, *pSUBSYSTEM_INFO;



//   Structure used for accessing ReadSingleValue commands
typedef struct _PACK_
{
  uint8_t			Channel;
} READ_SINGLE_VALUE_INFO, *pREAD_SINGLE_VALUE_INFO;


//   Structure used for accessing ReadSingleValue commands
typedef struct _PACK_
{
  uint8_t			NumChans;
} READ_SINGLE_VALUES_INFO, *pREAD_SINGLE_VALUES_INFO;



//   Structure used for accessing WriteSingleValue commands
typedef struct _PACK_
{
  SUBSYSTEM_TYPE    SubsystemType; // Specifies  the subsystem type
  
  #ifndef WIN_DRIVER
  uint8_t                  Filler[3];
  #endif

  uint8_t  Channel;
  uint16_t	DataValue;
} WRITE_SINGLE_VALUE_INFO, *pWRITE_SINGLE_VALUE_INFO;




 /******************************************************************************
 Structures used for misc commands
 ******************************************************************************/ 

// This structure is used with the W_DAC_THRESHOLD command
typedef struct _PACK_
{
  DWORD_STRUCT  Threshold;
} DAC_THRESHOLD_INFO, *pDAC_THRESHOLD_INFO;


// This structure is used with the W_DAC_FIFO_SIZE and W_ADC_FIFO_SIZE command
typedef struct _PACK_
{
  DWORD_STRUCT    FifoSize;
} FIFO_SIZE_INFO, *pFIFO_SIZE_INFO;


// This structure is used with the W_INT_ON_CHANGE_MASK
typedef struct _PACK_
{
   uint8_t             PortNum;
   uint8_t             MaskVal;
} INT_ON_CHANGE_MASK_INFO, *pINT_ON_CHANGE_MASK_INFO;



 /******************************************************************************
 Structure used for accessing the calibration potentiometers
 ******************************************************************************/ 

typedef struct _PACK_ 
{
    uint8_t    ChipNum;
    uint8_t    PotNum;
    uint8_t    RegNum;
    uint8_t    DataVal; 
} WRITE_CAL_POT_INFO;

typedef struct _PACK_ 
{
    uint8_t    ChipNum;
    uint8_t    PotNum;
    uint8_t    RegNum;
} READ_CAL_POT_INFO;




/***********************************************************************

  USB_CMD - This structure defines all of the Commands that can be
  sent via USB to the firmware.

  Each command contains at minimum a CmdType.  Most commands also
  include associated data that are defined in the "d" union within
  the USB_CMD STRUCTURE.

  The R_ (READ) commands cause the firmware to send back data via the
  COMMAND_READ_PIPE.  Refer to DT9810DriverFirmwareAPI.doc for the
  details of what data comes back from each command.

  ************************************************************************/

typedef struct _PACK_
{
   USB_FIRMWARE_CMD_CODE        CmdCode;

   // If compiling USB firmware then add filler since Windows compiler
   // uses 4 byte enums rather than Keil's 2 byte. See note at the begining
   // of this file
   #ifndef WIN_DRIVER
      uint8_t                      Filler[3];
   #endif
   union _PACK_
   {
      WRITE_BYTE_INFO           WriteByteInfo;
      READ_BYTE_INFO            ReadByteInfo;
      WRITE_MULTI_INFO          WriteMultiInfo;
      READ_MULTI_INFO           ReadMultiInfo;
      RMW_BYTE_INFO             RMWByteInfo;
      RMW_MULTI_INFO            RMWMultiInfo;
  
      WRITE_I2C_BYTE_INFO       WriteI2CByteInfo;
      READ_I2C_BYTE_INFO        ReadI2CByteInfo;
      WRITE_I2C_MULTI_INFO      WriteI2CMultiInfo;
      READ_I2C_MULTI_INFO       ReadI2CMultiInfo;


      WRITE_WORD_INFO           WriteWordInfo;
      READ_WORD_INFO            ReadWordInfo;
      WRITE_MULTI_WORD_INFO     WriteMultiWordInfo;
      READ_MULTI_WORD_INFO      ReadMultiWordInfo;
      RMW_WORD_INFO             RMWWordInfo;
      RMW_MULTI_WORD_INFO       RMWMultiWordInfo;

      SUBSYSTEM_INFO            SubsystemInfo;
      READ_SINGLE_VALUE_INFO    ReadSingleValueInfo;
      READ_SINGLE_VALUES_INFO   ReadSingleValuesInfo;
      WRITE_SINGLE_VALUE_INFO   WriteSingleValueInfo;

      DAC_THRESHOLD_INFO        DacThresholdInfo;
      INT_ON_CHANGE_MASK_INFO   IntOnChangeMaskInfo;
	  FIFO_SIZE_INFO			FifoSizeInfo;

	  WRITE_CAL_POT_INFO        WriteCalPotInfo;
      READ_CAL_POT_INFO         ReadCalPotInfo;
	} d;

} USB_CMD, *PUSB_CMD;






/******************************************************************

  USB_FIRMWARE_MSG_CODE - defines all of the messages that the
  DT981 firmware can asynchronously send to the driver via USB,

  *****************************************************************/
typedef enum _PACK_
{
    CTR_OVERFLOW_MSG = 0x01,	// Fill out CtrOverflowInfo
    DIN_CHANGED_MSG,			// Fill out DinChangedInfo
    DAC_THRESHOLD_REACHED_MSG,
    ADC_OVER_SAMPLE_MSG,
	INPUT_FIFO_OVERFLOW_MSG,
	OUTPUT_FIFO_UNDERFLOW_MSG,
    DAC_OVER_SAMPLE_MSG,
	OUTPUT_DONE_MSG,
    MAX_USB_FIRMWARE_MSG_CODE   //Valid USB_FIRMWARE_MSG_CODE's will be less than
                                //this number
} USB_FIRMWARE_MSG_CODE;


// This structure is used with the INT_STATUS_MSG
typedef struct _PACK_
{
   uint8_t             CtrNum;
} CTR_OVERFLOW_INFO, *pCTR_OVERFLOW_INFO;


// This structure is used with the INT_ON_CHANGE_MSG
typedef struct _PACK_
{
   uint8_t             CurVal;
} DIN_CHANGED_INFO, *pDIN_CHANGED_INFO;




/***********************************************************************

  USB_MSG - This structure defines all of the messages along with
  their associated data that can be sent from the firmware to the driver.

  Each message contains at minimum a MsgType followed by associated data.



  ************************************************************************/

typedef struct _PACK_
{
   USB_FIRMWARE_MSG_CODE MsgType;

   // If compiling USB firmware then add filler since Windows compiler
   // uses 4 byte enums rather than Keil's 1 byte
   #ifndef WIN_DRIVER
      uint8_t                  Filler[3];
   #endif

   union _PACK_
   {
      CTR_OVERFLOW_INFO     CtrOverflowInfo;
      DIN_CHANGED_INFO      DinChangedInfo;
   } d;
} USB_MSG, *pUSB_MSG;

// If building windows driver, restore normal structure packing
#ifdef WIN_DRIVER
#pragma pack (pop)
#else
#undef _PACK_
#endif

#endif // _FIRMWARETODRIVER_H_
