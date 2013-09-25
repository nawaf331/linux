// Dt9836HwRegs.h  defines HW register memory map
//				   and control bit definitions  	 

#ifndef __Dt9836HwRegs_h__
#define __Dt9836HwRegs_h__



#define DT9836_BASE_CRYSTAL_FREQUENCY 36000000L
#define DT9832_BASE_CRYSTAL_FREQUENCY 48000000L
#define DT9862_BASE_CRYSTAL_FREQUENCY 100000000L



/***************************************************
 *
 * EEPROM offsets and definitions
 *
 ***************************************************/
/*  EEPROM Memory Map 

	There are 128 bits of EEPROM for at total of 16 bytes of non-volatile storage. The offsets
	for these 16 bytes are numered 0 to F. Of these 16 bytes, the first 8 are used by the 
	Cypress chip for USB enumeration. The remaining 8 bytes are used by our software to 
	store non-volatile values.

	 Offset     Item
	+-------+-----------------------
	|	0	|	fixed value = 0xC0. Tells the Cypress chip to load the remaining USB values
	|	1	|	Vendor ID (low byte)
	|	2	|	Vendor ID (high byte)
	|	3	|	Product ID (low byte)
	|	4	|	Product ID (high byte)
	|	5	|	Device ID (low byte)
	|	6	|	Device ID (high byte)
	|	7	|	Cypress chip configuration byte
	|	8	|	Serial Number (byte 0)
	|	9	|	Serial Number (byte 1)
	|	10	|	Serial Number (byte 2)
	|	11	|	Serial Number (byte 3)
	|	12	|	Misc options; <not sure if this is really used>
	|	13	|	<Unused>
	|	14	|	Termination resistor mask - for 9862 bits <1:0> used
	|	15	|	Digital Input Interrupt on Change Mask; all bits used
	+-------+-----------------------
 */
#define EEPROM_I2C_ADR				0x50
#define EEPROM_ID					0xc0

#define EEPROM_ID_OFFSET			0x00
#define EEPROM_VENDOR_IDL_OFFSET	0x01
#define EEPROM_VENDOR_IDH_OFFSET	0x02
#define EEPROM_PRODUCT_IDL_OFFSET	0x03
#define EEPROM_PRODUCT_IDH_OFFSET	0x04
#define EEPROM_DEVICE_IDL_OFFSET	0x05
#define EEPROM_DEVICE_IDH_OFFSET	0x06
#define EEPROM_CONFIG_OFFSET		0x07
#define EEPROM_SER_NUM0_OFFSET		0x08
#define EEPROM_SER_NUM1_OFFSET		0x09
#define EEPROM_SER_NUM2_OFFSET		0x0a
#define EEPROM_SER_NUM3_OFFSET		0x0b
#define EEPROM_MISC_OPTIONS_OFFSET	0x0c
#define EEPROM_UNUSED				0x0d
#define EEPROM_TERM_RESISTOR_OFFSET 0x0e		//  used on DT9862
#define EEPROM_DIN_MASK_OFFSET		0x0f

// Bit definitions within MISC_OPTIONS byte
#define MISC_OPTIONS_NQ_BIT			0x01

/***************************************************
		 DT9836 HW REGISTER IO Address MAP 
****************************************************/
/* Control regs */
#define		INPUT_CONTROL0				0x000	
#define		SAMPLES_PER_TRIG_LOW		0x001	
#define		SAMPLES_PER_TRIG_HIGH		0x002
#define		INPUT_CHAN_MASK_LOW			0x003	
#define		INPUT_CHAN_MASK_HIGH		0x004	
#define		OUTPUT_CONTROL				0x005	
#define		IO_OP_CONTROL				0x006	
#define		INT_STATUS					0x007	
#define		INT_MASK					0x008
#define		IN_SAMPLE_CLK_PERIOD_LOW	0x009
#define		IN_SAMPLE_CLK_PERIOD_HIGH	0x00a
#define		OUT_SAMPLE_CLK_PERIOD_LOW	0x00b
#define		OUT_SAMPLE_CLK_PERIOD_HIGH	0x00c
#define		TRIG_SCAN_PERIOD_LOW		0x00d
#define		TRIG_SCAN_PERIOD_HIGH		0x00e
#define		POST_TRIG_COUNT				0x00f
#define		MISC_CTRLS					0x010
#define		INPUT_CONTROL1				0x011		// used on DT9862
#define		SYNC_CLOCK_WINDOW_COUNT		0x012		// used on DT9862

/* Version */
#define		VERSION_ID					0x020	

#define		ANALOG_IN_CAL_POTS			0x040	
#define		ANALOG_TRIG_THRESHOLD		0x041
#define		ANALOG_INPUT_TREM_MUX		0x041	

/* CT regs*/
#define		COUNTER0_PERIOD_LOW		0x0C0	
#define		COUNTER0_PERIOD_HIGH	0x0C1	
#define		COUNTER0_PULSE_LOW		0x0C2	
#define		COUNTER0_PULSE_HIGH		0x0C3	
#define		COUNTER0_CONTROL_LOW	0x0C4	
#define		COUNTER0_CONTROL_HIGH	0x0C5	

#define		COUNTER1_PERIOD_LOW		0x0C6	
#define		COUNTER1_PERIOD_HIGH	0x0C7	
#define		COUNTER1_PULSE_LOW		0x0C8	
#define		COUNTER1_PULSE_HIGH		0x0C9	
#define		COUNTER1_CONTROL_LOW	0x0Ca	
#define		COUNTER1_CONTROL_HIGH	0x0Cb	

#define		COUNTER2_PERIOD_LOW		0x0CC	
#define		COUNTER2_PERIOD_HIGH	0x0CD	
#define		COUNTER2_PULSE_LOW		0x0CE	
#define		COUNTER2_PULSE_HIGH		0x0CF	
#define		COUNTER2_CONTROL_LOW	0x0D0	
#define		COUNTER2_CONTROL_HIGH	0x0D1	

#define		COUNTER3_PERIOD_LOW		0x0D2
#define		COUNTER3_PERIOD_HIGH	0x0D3	
#define		COUNTER3_PULSE_LOW		0x0D4	
#define		COUNTER3_PULSE_HIGH		0x0D5	
#define		COUNTER3_CONTROL_LOW	0x0D6	
#define		COUNTER3_CONTROL_HIGH	0x0D7	

#define		COUNTER4_PERIOD_LOW		0x0D8	
#define		COUNTER4_PERIOD_HIGH	0x0D9	
#define		COUNTER4_PULSE_LOW		0x0DA	
#define		COUNTER4_PULSE_HIGH		0x0DB	
#define		COUNTER4_CONTROL_LOW	0x0DC	
#define		COUNTER4_CONTROL_HIGH	0x0DD	


#define		COUNTER0_COUNT_LOW		0x0E0	
#define		COUNTER0_COUNT_HIGH		0x0E1	
#define		COUNTER1_COUNT_LOW		0x0E2	
#define		COUNTER1_COUNT_HIGH		0x0E3	
#define		COUNTER2_COUNT_LOW		0x0E4	
#define		COUNTER2_COUNT_HIGH		0x0E5	
#define		COUNTER3_COUNT_LOW		0x0E6	
#define		COUNTER3_COUNT_HIGH		0x0E7	
#define		COUNTER4_COUNT_LOW		0x0E8	
#define		COUNTER4_COUNT_HIGH		0x0E9	

/* Counter interrupt control regs */
#define		COUNTER_INT_STATUS		0x0f0
#define		COUNTER_INT_MASK		0x0f1

/* Dig In data and ctrl regs */
#define		DIG_IN_DATA				0x100	
#define		DIG_IN_INT_DATA			0x101
#define		DIG_IN_INT_MASK			0x102	   
#define		DIG_IN_CTRL_STAT		0x103	

/* Dig Out data reg */
#define		DIGITAL_OUTPUT			0x110	

/* Quadrature Encoder Regs */
#define		QUAD_DEC0_CTRL			0x120
#define		QUAD_DEC0_COUNT_LOW		0x121
#define		QUAD_DEC0_COUNT_HIGH	0x122
#define		QUAD_DEC1_CTRL			0x123
#define		QUAD_DEC1_COUNT_LOW		0x124
#define		QUAD_DEC1_COUNT_HIGH	0x125
#define		QUAD_DEC2_CTRL			0x126
#define		QUAD_DEC2_COUNT_LOW		0x127
#define		QUAD_DEC2_COUNT_HIGH	0x128

#define		ANALOG_OUT_CAL_POTS		0x140	

/* Input fifo ctrls */
#define		INPUT_FIFO_PRGM_DATA	0x180	
#define		INPUT_FIFO_CTRL			0x181	
#define		INPUT_FIFO_INIT			0x182	// does not exist on on 9862

/* output fifo ctrls */
#define		OUTPUT_FIFO_CTRL		0x1C0	
#define		OUTPUT_FIFO_INIT		0x1C1	



/***************************************************
		 DT9836 HW CONTROL BIT  definitions 
****************************************************/

/* Input Subsystem Control Register 0 */
#define IGNORE_AD_ERROR				0x8000
#define INPUT_ACTIVE_STATUS			0x2000	// RO Bit .. Input subsystem is active
#define PRETRIGGER_EN				0x1000
#define TRIGGERED_SCAN				0x800
#define INITIAL_TRIGGER_SEL			0x400
#define RETRIGGER_SEL				0x200
#define EXT_INPUT_TRIGGER_SEL		0x100
#define EXT_INPUT_TRIGGER_POLARITY	0x80
#define EXT_INPUT_SAMPLE_CLK		0x40
#define ADC_STANDBY					0x20	// not on 9862
#define ADC_2V_RANGE				0x10	// not on 9862
#define ADC_RESET				    0x8		// not on 9862
#define ADC_REF_ENABLE				0x4		// not on 9862
#define ADC_SINGLE_ENDED			0x1		// not on 9862

/* Input Subsystem Control Register 1; on DT9862 */
#define IN_CLOCK_COUNTER_ENABLE		0x8
#define AD_RANDOMIZE				0x4
#define CH1_TERMINATION_ENABLE		0x2
#define CH0_TERMINATION_ENABLE		0x1
#define TERMINATION_ENABLE_MASK		(CH1_TERMINATION_ENABLE | CH0_TERMINATION_ENABLE)

/* Sync Clock Window Count Register; on DT9862 */
#define SCWC_ALL_BITS				0x03FF

/* Channel Mask Low register */
#define DT9862_QD2_SEL				0x80
#define DT9862_QD1_SEL				0x40
#define DT9862_QD0_SEL				0x20
#define DT9862_CT1_SEL				0x10
#define DT9862_CT0_SEL				0x8
#define DT9862_DIGIN_SEL			0x4

#define DT9832A_QD2_SEL				0x80
#define DT9832A_QD1_SEL				0x40
#define DT9832A_QD0_SEL				0x20
#define DT9832A_CT1_SEL				0x10
#define DT9832A_CT0_SEL				0x8
#define DT9832A_DIGIN_SEL			0x4

#define DT9832_QD2_SEL				0x200
#define DT9832_QD1_SEL				0x100
#define DT9832_QD0_SEL				0x80
#define DT9832_CT1_SEL				0x40
#define DT9832_CT0_SEL				0x20
#define DT9832_DIGIN_SEL			0x10

#define DT9836_QD0_SEL				0x8000
#define DT9836_CT1_SEL				0x4000
#define DT9836_CT0_SEL				0x2000
#define DT9836_DIGIN_SEL			0x1000 
#define ADC_CHAN11_SEL				0x800
#define ADC_CHAN10_SEL				0x400
#define ADC_CHAN9_SEL				0x200
#define ADC_CHAN8_SEL				0x100
#define ADC_CHAN7_SEL				0x80
#define ADC_CHAN6_SEL				0x40
#define ADC_CHAN5_SEL				0x20
#define ADC_CHAN4_SEL				0x10
#define ADC_CHAN3_SEL				0x8
#define ADC_CHAN2_SEL				0x4
#define ADC_CHAN1_SEL				0x2
#define ADC_CHAN0_SEL				0x1

/* Channel Mask High register */
#define DT9836_QD2_SEL				0x2
#define DT9836_QD1_SEL				0x1


/* Output Subsystem Control Register */
#define DAC3_CLEAR_N				0x8000   // DACS held clear
#define DAC2_CLEAR_N				0x4000
#define DAC1_CLEAR_N				0x2000
#define DAC0_CLEAR_N				0x1000  
#define DAC_RESET_ALL_N				0x0800
#define OUTOUT_ACTIVE_STATUS		0x400	// RO Bit .. Output subsystem is active
#define TRIGGER_SEL					0x200
#define EXT_OUTPUT_TRIGGER_SEL		0x100	
#define EXT_OUTPUT_TRIGGER_POLARITY	0x80
#define EXT_OUTPUT_SAMPLE_CLK		0x40
#define WAVEFORM_GENERATOR			0x20
#define DIG_OUT_SEL					0x10
#define DA_CHANNEL3_SEL				0x8
#define DA_CHANNEL2_SEL				0x4
#define DA_CHANNEL1_SEL				0x2
#define DA_CHANNEL0_SEL				0x1	

/*Input/Output Operational Control Register  */
#define SINGLE_INPUT_SAMPLE			0x8000
#define OUTPUT_ACTIVE_RO_BIT		0x400   // RO bit == output subsystem is active
#define INPUT_ACTIVE_RO_BIT			0x200	// RO bit == input subsystem is active
#define OUTPUT_ARM					0x100
#define OUTPUT_SOFT_TRIGGER			0x80
#define OUTPUT_FIRST_REG_LD			0x40
#define INPUT_SCAN_ARM				0x20
#define INPUT_SCAN_STOP				0x10
#define INPUT_SOFT_TRIGGER			0x8	
#define CNTR_TMR_RESET				0x1

/* **** Interrupt Status Register */
#define CTR_TIMER_INTERRUPT			0x20
#define DAC_OVER_SAMPLE				0x10
#define OUTPUT_DONE					0x8
#define OUTPUT_FIFO_UNDERFLOW		0x4
#define ADC_OVER_SAMPLE				0x2
#define INPUT_FIFO_OVERFLOW			0x1

/* Interrupt Mask Register*/
/*  ****  Same bit definitions as Interrupt Status Register .. above
#define CTR_TIMER_INTERRUPT			0x20
#define DAC_OVER_SAMPLE				0x10
#define OUTPUT_DONE					0x8
#define OUTPUT_FIFO_UNDERFLOW		0x4
#define ADC_OVER_SAMPLE				0x2
#define INPUT_FIFO_OVERFLOW			0x1
*/

/* Misc Control register */
#define QUAD_DECODE_DISABLE 		0x4
#define DAC_TWOS_COMP				0x2
#define ADC_TWOS_COMP	q			0x1


/* Version ID Register */
#define HW_VERSION_ID				0x7	 //	3 bit mask (bits 0 - 2)	
		/*	0x4 = 16-bit, 12 channels in, w/  DACs, 3 quad, 2 ctrs
			0x6 = 16-bit, 6  cannels in, w/  DACs, 3 quad, 2 ctrs */
			
/*Analog Input Calibration Pots Register */
#define ADC_CAL_POT_CTRL_RD_DATA	0x4
#define ADC_CAL_POT_CTRL_DATA		0x2
#define ADC_CAL_POT_CTRL_CLK		0x1 

/* Counter Control Registers (Low) */
#define MEASURE_TRIG_EN				0x4000

#define MEASURE_STOP_SEL			0x3000	// 2 bit mask (bits 12 - 13)
#define STOP_ON_EXT_CLK_FALLING		0x3000	// 
#define STOP_ON_EXT_CLK_RISING		0x2000	// 
#define STOP_ON_GATE_FALLING		0x1000	// 
#define STOP_ON_GATE_RISING			0x0000	// 

#define MEASURE_START_SEL			0xC00	// 2 bit mask (bits 10 - 11)	
#define START_ON_EXT_CLK_FALLING	0xc00	// 
#define START_ON_EXT_CLK_RISING		0x800	// 
#define START_ON_GATE_FALLING		0x400	// 
#define START_ON_GATE_RISING		0x000	// 


#define AUX_MODE_SEL				0x300	// 2 bit mask (bits 8 - 9)	
#define AUX_MODE_UP_DOWN			0x100	// 
#define AUX_MODE_MEASURE			0x000	// 

#define ONE_SHOT_TRIG_EN			0x80
#define OUTPUT_POLARITY_LO_2_HI		0x40
#define OUTPUT_POLARITY_HI_2_LO		0x00


#define MODE_SELECT					0x30	// 2 bit mask (bits 4 - 5)	
#define MODE_AUXILIARY				0x30	// 
#define MODE_CONTINUOUS_INC			0x20	// 
#define MODE_RETRIG_ONE_SHOT		0x10	// 
#define MODE_NON_RETRIG_ONE_SHOT	0x00	// 

#define GATE_SELECT					0xC		// 2 bit mask (bits 2 - 3)	
#define GATE_INVERTED_EXTERNAL		0xC		// 
#define GATE_EXTERNAL				0x8		// 
#define GATE_SW_ENABLED				0x4		// 
#define GATE_SW_DISABLED			0x0		// 

#define CLOCK_SELECT				0x3		// 2 bit mask (bits 0 - 1)	
#define CLOCK_CASCADED				0x2		// illegal
#define CLOCK_EXTERNAL				0x1		// 
#define CLOCK_INTERNAL				0x0		// 

/* Counter Control Registers (High) */
#define CONT_MEASURE_SELF_CLEAR		0x2
#define CONT_MEASURE				0x1

/* Counter/Timer Status Register */
#define CTR1_MEASURE_COMPLETE		0x40
#define CTR0_MEASURE_COMPLETE		0x20
#define CTR1_OVERFLOW				0x2
#define CTR0_OVERFLOW				0x1

/* Digital Input Control/Status Register */
#define FAST_INT_ON_CHANGE_PER		0x10
#define INTERRUPT_CLR				0x4
#define INTERRUPT					0x2
#define INTERRUPT_EN				0x1

/* Quadrature Decode Control Registers */
#define FILTER_CLK_PRESCALE_BITS	0xff00
#define A_LEADS_B_CCW				0x20
#define X4							0x10
#define INDEX_HIGH_TRUE				0x08
#define INDEX_ENABLE				0x04
#define DECODER_RESET				0x02
#define DECODER_ENABLE				0x01


/* Analog Output Calibration Pots Register */
#define DAC_CAL_POT_CTRL_RD_DATA	0x4
#define DAC_CAL_POT_CTRL_DATA		0x2
#define DAC_CAL_POT_CTRL_CLK		0x1

/* Input FIFO Program Data Register */
#define PROGRAM_WRITE				0x8000	// in Input FIFO Program Register, 0x180
#define PROGRAM_WRITE_9862			0x1000	// in Input FIFO Control Register, 0x181
#define INPUT_FIFO_PRGM_DATA_MASK	0x0FFF  // 12 bit mask (bits 0 - 11)

/* **** Input FIFO Control Register    *****/
#define MASTER_RESET				0x8000
#define PARTIAL_RESET				0x4000
#define FULL_FLAG					0x100
#define ALMOST_FULL_FLAG			0x80
#define HALF_FULL_FLAG				0x40
#define ALMOST_EMPTY_FLAG			0x20
#define EMPTY_FLAG					0x10
#define LOAD						0x2
#define PROGRAM_EN					0x1

/* ***** Input FIFO Initialization Register ***** */
#define SERIAL_ENABLE				0x40
#define FALL_THROUGH_SER_IN			0x20
#define PRG_FLAG_MODE				0x10	
#define FLAG_SEL1					0x8
#define FLAG_SEL0					0x4
#define ASYNC_READ					0x2
#define ASYNC_WRITE					0x1

/* Output FIFO Control Register */
/* ***** See Input fifo Control Register above
#define MASTER_RESET				0x8000
#define PARTIAL_RESET				0x4000
#define FULL_FLAG					0x100
#define ALMOST_FULL_FLAG			0x80
#define HALF_FULL_FLAG				0x40
#define ALMOST_EMPTY_FLAG			0x20
#define EMPTY_FLAG					0x10
#define LOAD						0x2
#define PROGRAM_EN					0x1
****NOTE  .. 'ProgramEn' bit not required here  
*/

/* Output FIFO Initialization Register */
/* ***** See Input Fifo Initialization Register above
#define SERIAL_ENABLE				0x40
#define FALL_THROUGH_SER_IN			0x20
#define PRG_FLAG_MODE				0x10	
#define FLAG_SEL1					0x8
#define FLAG_SEL0					0x4
#define ASYNC_READ					0x2
#define ASYNC_WRITE					0x1
*/

#endif // __Dt9836HwRegs_h__










































