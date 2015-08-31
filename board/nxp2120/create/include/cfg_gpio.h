/*------------------------------------------------------------------------------
 *
 *	Copyright (C) 2005 Nexell Co., Ltd All Rights Reserved
 *	Nexell Co. Proprietary & Confidential
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 *  FOR A PARTICULAR PURPOSE.
 *
 *	Module     : System memory config
 *	Description:
 *	Author     : Platform Team
 *	Export     :
 *	History    :
 *	   2009/05/13 first implementation
 ------------------------------------------------------------------------------*/
#ifndef __CFG_GPIO_H__
#define __CFG_GPIO_H__

/*------------------------------------------------------------------------------
 *
 *	(GROUP_A)
 *
 *	0 bit  	  		 			     4 bit	   					     8 bit		  12 bit   				     16 bit
 *	| GPIO'A'OUTENB and GPIO'A'ALTFN | GPIO'A'OUT or GPIO'A'DETMODE0 | GPIO'A'PUENB| CLKPWR.PADSTRENGTHGPIO'A'|
 *
 -----------------------------------------------------------------------------*/
#define PAD_GPIOA0      (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)   	// [ALT1: PVD0_0 ], Touch Pen Control
#define PAD_GPIOA1      (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_1 ], Touch Pen Detect
#define PAD_GPIOA2      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_2 ]
#define PAD_GPIOA3      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_3 ]
#define PAD_GPIOA4      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_4 ]
#define PAD_GPIOA5      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_5 ]
#define PAD_GPIOA6      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_6 ]
#define PAD_GPIOA7      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_7 ]
#define PAD_GPIOA8      (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_8 ], Touch XmonP
#define PAD_GPIOA9      (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_9 ], Touch XmonM
#define PAD_GPIOA10     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_10]
#define PAD_GPIOA11     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_11]
#define PAD_GPIOA12     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_12]
#define PAD_GPIOA13     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_13]
#define PAD_GPIOA14     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_14]
#define PAD_GPIOA15     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_15]
#define PAD_GPIOA16     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_16], USB Hub Reset (Active Low, MID), Touch YmonP (CREATE)
#define PAD_GPIOA17     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_17], Touch YmonM
#define PAD_GPIOA18     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_18]
#define PAD_GPIOA19     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_29]
#define PAD_GPIOA20     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_20]
#define PAD_GPIOA21     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_21]
#define PAD_GPIOA22     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_22]
#define PAD_GPIOA23     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVD0_23]
#define PAD_GPIOA24     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_8) 	// [ALT1: PVCLK ]
#define PAD_GPIOA25     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_8)     // [ALT1: PDE ]
#define PAD_GPIOA26     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PHSYNC ]
#define PAD_GPIOA27     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PVSYNC, External USB Enable (Active High, CREATE) ]
#define PAD_GPIOA28     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SPI0_FRM ]
#define PAD_GPIOA29     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SPI0_CLK ]
#define PAD_GPIOA30     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SPI0_RXD ]
#define PAD_GPIOA31		(PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SPI0_TXD ]

/*------------------------------------------------------------------------------
 *	(GROUP_B)
 *
 *	0 bit  	  		 			     4 bit	   					     8 bit		  12 bit   				     16 bit
 *	| GPIO'B'OUTENB and GPIO'B'ALTFN | GPIO'B'OUT or GPIO'B'DETMODE0 | GPIO'B'PUENB| CLKPWR.PADSTRENGTHGPIO'B'|
 *
 -----------------------------------------------------------------------------*/
#define PAD_GPIOB0		(PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: UART0_TX]
#define PAD_GPIOB1      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)    	// [ALT1: UART1_TX]
#define PAD_GPIOB2      (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PWMOUT0]
#define PAD_GPIOB3      (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: PWMOUT1]
#define PAD_GPIOB4      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SCL0]
#define PAD_GPIOB5      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SDA0]
#define PAD_GPIOB6      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: SCL1]
#define PAD_GPIOB7      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SDA1]
#define PAD_GPIOB8      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: I2S_DOUT]
#define PAD_GPIOB9      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: I2S_BCLK]
#define PAD_GPIOB10     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: I2S_SYNC]
#define PAD_GPIOB11     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL    | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: I2S_DIN]
#define PAD_GPIOB12     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL    | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: I2S_MCLK]
#define PAD_GPIOB13     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_8)		// [ALT1: SD0_CLK   ]
#define PAD_GPIOB14     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_8)		// [ALT1: SD0_CMD   ]
#define PAD_GPIOB15     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_8)		// [ALT1: SD0_DAT0]
#define PAD_GPIOB16     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_8)		// [ALT1: SD0_DAT1]
#define PAD_GPIOB17     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_8)		// [ALT1: SD0_DAT2]
#define PAD_GPIOB18     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_8)     // [ALT1: SD0_DAT3]
#define PAD_GPIOB19     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD8 ], USB Hub Reset (Active Low, CREATE BOARD)
#define PAD_GPIOB20     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD9 ]
#define PAD_GPIOB21     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD10]
#define PAD_GPIOB22     (PAD_MODE_OUT  | PAD_OUT_HIGHLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD11], USB Select L:device H:Host
#define PAD_GPIOB23     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD12], CAM Power Down
#define PAD_GPIOB24     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD13], AMP Enable
#define PAD_GPIOB25     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD14], WIFI Power Enable
#define PAD_GPIOB26     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SD15], USB CAM Power Enable
#define PAD_GPIOB27     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SA0 ], DC In Detect
#define PAD_GPIOB28     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: SA1 ], BAT_CHARGE
#define PAD_GPIOB29     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)  	// [ALT1: SA2 ]
#define PAD_GPIOB30     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: DMB_INT], SD Write Protect
#define PAD_GPIOB31     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)     // [ALT1: VIP_D0]

/*------------------------------------------------------------------------------
 *	(GROUP_C)
 *
 *	0 bit  	  		 			     4 bit	   					     8 bit		  12 bit   				     16 bit
 *	| GPIO'C'OUTENB and GPIO'C'ALTFN | GPIO'C'OUT or GPIO'C'DETMODE0 | GPIO'C'PUENB| CLKPWR.PADSTRENGTHGPIO'C'|
 *
 -----------------------------------------------------------------------------*/
#define PAD_GPIOC0 		(PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D1]
#define PAD_GPIOC1      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D2]
#define PAD_GPIOC2      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D3]
#define PAD_GPIOC3      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D4]
#define PAD_GPIOC4      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D5]
#define PAD_GPIOC5      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D6]
#define PAD_GPIOC6      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VIP_D7]
#define PAD_GPIOC7      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: VCLK]
#define PAD_GPIOC8      (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: UART1_RX]
#define PAD_GPIOC9      (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [GPIO: SD Card Detect]
#define PAD_GPIOC10     (PAD_MODE_INT  | PAD_INT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_2)		// [GPIO: Touch Pen Detect]
#define PAD_GPIOC11     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [GPIO: TS Irq (G-sensor)]
#define PAD_GPIOC12     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_2)		// [GPIO: Volume Down]
#define PAD_GPIOC13     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_8)		// [GPIO: Volume Up]
#define PAD_GPIOC14     (PAD_MODE_OUT  | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [GPIO: Shake Motor]
#define PAD_GPIOC15     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [GPIO: Board Detect]
#define PAD_GPIOC16     (PAD_MODE_IN   | PAD_OUT_LOWLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_2)		// [GPIO: Menu Key]
#define PAD_GPIOC17     (PAD_MODE_OUT  | PAD_OUT_HIGHLEVEL	 | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [GPIO: NAND Write Protect]
#define PAD_GPIOC18     (PAD_MODE_INT  | PAD_OUT_HIGHLEVEL	 | PAD_PULLUP_ON  | PAD_STRENGTH_2)		// [ALT1: nSCS0], Ethernet IRQ
#define PAD_GPIOC19     (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL    | PAD_PULLUP_OFF | PAD_STRENGTH_2)		// [ALT1: nSCS1]
#define PAD_GPIOC20     (PAD_MODE_ALT1 | PAD_OUT_HIGHLEVEL   | PAD_PULLUP_ON  | PAD_STRENGTH_2)		// [ALT1: UART0_RX]

/*------------------------------------------------------------------------------
 *	(GROUPALV)
 *	0  	   		   			4	   							8			12
 *	| MODE(IN/OUT/DETECT)	| ALIVE OUT or ALIVE DETMODE0 	|   PullUp	|
 *
 -----------------------------------------------------------------------------*/
#define	PAD_GPIOALV0	(PAD_MODE_IN  | PAD_OUT_LOWLEVEL	 		| PAD_PULLUP_ON )				// BUTTON0
#define	PAD_GPIOALV1	(PAD_MODE_IN  | PAD_OUT_LOWLEVEL	 		| PAD_PULLUP_ON )				// BUTTON1
#define	PAD_GPIOALV2	(PAD_MODE_IN  | PAD_OUT_LOWLEVEL	 		| PAD_PULLUP_ON )				// BUTTON2
#define	PAD_GPIOALV3	(PAD_MODE_IN  | PAD_OUT_LOWLEVEL		    | PAD_PULLUP_ON )				// BUTTON3

/*------------------------------------------------------------------------------
 *	(BUS signal)		[BIT] :	0  	      8			12
 *								| BUS Pad | Strength |
 *
 -----------------------------------------------------------------------------*/
#define	PAD_BUS_STATIC_CNTL		(NX_CLKPWR_BUSPAD_STATIC_CNTL 	| PAD_STRENGTH_8)
#define	PAD_BUS_STATIC_ADDR		(NX_CLKPWR_BUSPAD_STATIC_ADDR 	| PAD_STRENGTH_8)
#define	PAD_BUS_STATIC_DATA		(NX_CLKPWR_BUSPAD_STATIC_DATA 	| PAD_STRENGTH_8)
#define	PAD_BUS_VSYNC			(NX_CLKPWR_BUSPAD_VSYNC 		| PAD_STRENGTH_8)
#define	PAD_BUS_HSYNC			(NX_CLKPWR_BUSPAD_HSYNC 		| PAD_STRENGTH_8)
#define	PAD_BUS_DE				(NX_CLKPWR_BUSPAD_DE 			| PAD_STRENGTH_8)

/*------------------------------------------------------------------------------
 *	GPIO I2C
 */
#define	CFG_PIO_I2C0_SCL						(PAD_GPIO_B +  4)
#define	CFG_PIO_I2C0_SDA						(PAD_GPIO_B +  5)
#define	CFG_PIO_I2C1_SCL						(PAD_GPIO_B +  6)
#define	CFG_PIO_I2C1_SDA						(PAD_GPIO_B +  7)

/*------------------------------------------------------------------------------
 *	GPIO SDHC
 */
#define CFG_PIO_SDHC_0_WP						(-1)
#define CFG_PIO_SDHC_0_DETECT          			(PAD_GPIO_C +  9)

/*------------------------------------------------------------------------------
 *	Touch
 */
#define CFG_PIO_TOUCH_YMON_P					(PAD_GPIO_A + 16)
#define CFG_PIO_TOUCH_YMON_N					(PAD_GPIO_A + 17)
#define CFG_PIO_TOUCH_XMON_P					(PAD_GPIO_A +  8)
#define CFG_PIO_TOUCH_XMON_N					(PAD_GPIO_A +  9)
#define CFG_PIO_TOUCH_PENDOWN_CON				(PAD_GPIO_A +  0)
#define CFG_PIO_TOUCH_PENDOWN_DETECT			(PAD_GPIO_A +  1)

/*------------------------------------------------------------------------------
 *	Auido AMP
 */
#define CFG_PIO_AUDIO_AMP						(PAD_GPIO_B + 24)

/*------------------------------------------------------------------------------
 *	USB Hub Reset
 */
#define CFG_PIO_USB_HUB_RESET					(PAD_GPIO_B + 19)

/*------------------------------------------------------------------------------
 *	USB Ether Reset
 */
#define CFG_PIO_USB_ETHER_RESET					(PAD_GPIO_A + 27)

/*------------------------------------------------------------------------------
 *	VIBRATOR
 */
#define CFG_PIO_VIBRATION_ENABLE				(PAD_GPIO_C + 14)

#endif	/* __CFG_GPIO_H__ */
