/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file hb2002.h
 * @brief This file contains the HB2002 Motor Control register definitions, access macros, and
 * device access functions.
 */
#ifndef HB2002_H_
#define HB2002_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "fsl_lpadc.h"

/**
 * @brief PWM default configuration macro
 */

#define BOARD_PWM_BASEADDR 				(FLEXPWM0)
#define PWM_SRC_CLK_FREQ 				(CLOCK_GetFreq(kCLOCK_BusClk))
#define DEMO_PWM_CLOCK_DEVIDER			(kPWM_Prescale_Divide_4)
#define DEMO_PWM_FAULT_LEVEL   			true


/**
 * @brief LPADC default configuration macro
 */

#define DEMO_LPADC_BASE         		ADC0
#define DEMO_LPADC_IRQn                 ADC0_IRQn
#define DEMO_LPADC_IRQ_HANDLER_FUNC     ADC0_IRQHandler
#define DEMO_LPADC_USER_CHANNEL         0U
#define DEMO_LPADC_USER_CMDID           1U
#define DEMO_LPADC_VREF_SOURCE          kLPADC_ReferenceVoltageAlt3;

/**
 * @brief The HB2002 types
 */

/**
 **
 **  @brief The HB2002 Motor Control Internal Register Map.
 */
enum
{
	HB2002_DeviceIdentification = 0x00,
	HB2002_Status = 0x01,
	HB2002_FaultStatusMask = 0x02,
	HB2002_ConfigurationControl = 0x03,
};


/*--------------------------------
 ** Register: STATUS
 ** Enum: HB2002_STATUS
 ** --
 ** Offset : 0x01 Status register.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t ov : 1;     /*  VPWR overvoltage */

		uint8_t uv : 1;     /*  VPWR undervoltage */

		uint8_t cp_u : 1;   /*  Charge pump undervoltage */

		uint8_t frm : 1;    /* SPI framing error */

		uint8_t _reserved_0 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_1 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_2 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_3 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t scp2 : 1;   /*  Short-circuit to power output 1 */

		uint8_t scp1 : 1;   /*  Short-circuit to power output 1 */

		uint8_t scg2 : 1;   /*  Short-circuit to ground output 2 */

		uint8_t scg1 : 1;   /*  Short-circuit to ground output 1  */

		uint8_t ol : 1;     /*  Open load */

		uint8_t oc : 1;     /*  Over current - current limit has been activated */

		uint8_t tw : 1;     /*  Thermal warning  */

		uint8_t ot : 1;     /*  Overtemperature shutdown  */
	} b;
	uint16_t w;
} HB2002_STATUS;

/*--------------------------------
 ** Register: FAULT STATUS
 ** Enum: HB2002_FAULT_STATUS
 ** --
 ** Offset : 0x02 Fault Status register.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t ot : 1;     /*  Overtemperature shutdown  */

		uint8_t tw : 1;     /*  Thermal warning  */

		uint8_t oc : 1;     /*  Over current - current limit has been activated */

		uint8_t ol : 1;     /*  Open load */

		uint8_t scg1 : 1;   /*  Short-circuit to ground output 1  */

		uint8_t scg2 : 1;   /*  Short-circuit to ground output 2 */

		uint8_t scp1 : 1;   /*  Short-circuit to power output 1 */

		uint8_t scp2 : 1;   /*  Short-circuit to power output 1 */

		uint8_t ov : 1;     /*  VPWR overvoltage */

		uint8_t uv : 1;     /*  VPWR undervoltage */

		uint8_t cp_u : 1;   /*  Charge pump undervoltage */

		uint8_t frm : 1;    /* SPI framing error */

		uint8_t dov : 1;    /*  Disable overvoltage  */

		uint8_t _reserved_1 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_2 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_3 : 1;   /*  Reserved Bit (Unused)  */
	} b;
	uint16_t w;
} HB2002_FAULT_STATUS;

/*
 * STATUS/FAULT_STATUS - Bit field mask definitions
 */

#define HB2002_OT_SHIFT                 ((uint8_t)0x00)
#define HB2002_OT_MASK                  ((uint16_t)0x0001)

#define HB2002_TW_SHIFT                 ((uint8_t)0x01)
#define HB2002_TW_MASK                  ((uint16_t)0x0002)

#define HB2002_OC_SHIFT                 ((uint8_t)0x02)
#define HB2002_OC_MASK                  ((uint16_t)0x0004)

#define HB2002_OL_SHIFT                 ((uint8_t)0x03)
#define HB2002_OL_MASK                  ((uint16_t)0x0008)

#define HB2002_SCG1_SHIFT               ((uint8_t)0x04)
#define HB2002_SCG1_MASK                ((uint16_t)0x0010)

#define HB2002_SCG2_SHIFT               ((uint8_t)0x05)
#define HB2002_SCG2_MASK                ((uint16_t)0x0020)

#define HB2002_SCP1_SHIFT               ((uint8_t)0x06)
#define HB2002_SCP1_MASK                ((uint16_t)0x0040)

#define HB2002_SCP2_SHIFT               ((uint8_t)0x07)
#define HB2002_SCP2_MASK                ((uint16_t)0x0080)

#define HB2002_OV_SHIFT                 ((uint8_t)0x08)
#define HB2002_OV_MASK                  ((uint16_t)0x0100)

#define HB2002_UV_SHIFT                 ((uint8_t)0x09)
#define HB2002_UV_MASK                  ((uint16_t)0x0200)

#define HB2002_CP_U_SHIFT               ((uint8_t)0x0A)
#define HB2002_CP_U_MASK                ((uint16_t)0x0400)

#define HB2002_FRM_SHIFT                ((uint8_t)0x0B)
#define HB2002_FRM_MASK                 ((uint16_t)0x0800)

#define HB2002_DOV_SHIFT                ((uint8_t)0x0C)
#define HB2002_DOV_MASK                 ((uint16_t)0x1000)


/*
 * DEvice ID - Bit field mask definitions
 */

#define HB2002_DEV_ID_SHIFT              ((uint8_t)0x04)
#define HB2002_DEV_ID_MASK               ((uint16_t)0xFFFF)

/*--------------------------------
 ** Register: CONFIGURATION AND CONTROL
 ** Enum: HB2002_CNG_CTL
 ** --
 ** Offset : 0x03 Configuration and control.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t vin1 : 1;   /* Virtual Input 1 (SPI equivalent of IN1) */

		uint8_t vin2 : 1;   /* Virtual Input 2 (SPI equivalent of IN2)  */

		uint8_t input : 1;  /* Active INPUT Control mode */

		uint8_t mode : 1;  /* Input Control mode */

		uint8_t en : 1;    /*  Disable Outputs  */

		uint8_t sr : 3;   /*  Slew Rate Bit */

		uint8_t iml : 2;    /*  ILIM Bit */

		uint8_t al : 1;   /*  Active Current Limit mode */

		uint8_t tm : 1;    /* Thermal Management mode */

		uint8_t cl : 1;    /*  Check for open load  */

		uint8_t _reserved_1 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_2 : 1;   /*  Reserved Bit (Unused)  */

		uint8_t _reserved_3 : 1;   /*  Reserved Bit (Unused)  */
	} b;
	uint16_t w;
} HB2002_CNG_CTL;

/*
 * HB2002_CNG_CTL - Bit field mask definitions
 */

#define HB2002_VIN1_SHIFT                 ((uint8_t)0x00)
#define HB2002_VIN1_MASK                  ((uint16_t)0x0001)

#define HB2002_VIN2_SHIFT                 ((uint8_t)0x01)
#define HB2002_VIN2_MASK                  ((uint16_t)0x0002)

#define HB2002_INPUT_SHIFT                ((uint8_t)0x02)
#define HB2002_INPUT_MASK                 ((uint16_t)0x0004)

#define HB2002_MODE_SHIFT                 ((uint8_t)0x03)
#define HB2002_MODE_MASK                  ((uint16_t)0x0008)

#define HB2002_EN_SHIFT                   ((uint8_t)0x04)
#define HB2002_EN_MASK                    ((uint16_t)0x0010)

#define HB2002_SR_SHIFT                   ((uint8_t)0x05)
#define HB2002_SR_MASK                    ((uint16_t)0x00E0)

#define HB2002_ILM_SHIFT                 ((uint8_t)0x08)
#define HB2002_ILM_MASK                  ((uint16_t)0x0300)

#define HB2002_AL_SHIFT                   ((uint8_t)0x0A)
#define HB2002_AL_MASK                    ((uint16_t)0x0400)

#define HB2002_TM_SHIFT                   ((uint8_t)0x0B)
#define HB2002_TM_MASK                    ((uint16_t)0x0800)

#define HB2002_CL_SHIFT                   ((uint8_t)0x0C)
#define HB2002_CL_MASK                    ((uint16_t)0x1000)

/* Slew rate */
typedef uint8_t SlewRate;

/* I limit */
typedef uint8_t Ilimit;

/* one bit mask */
#define  HB2002_ONE_BIT_MASK              ((uint16_t)0x01)

#endif /* HB2002_H_ */
