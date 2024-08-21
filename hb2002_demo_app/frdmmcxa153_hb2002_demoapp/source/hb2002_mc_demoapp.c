/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------

#include "issdk_hal.h"
#include "gpio_driver.h"
#include "systick_utils.h"
#include "Driver_GPIO.h"

#include "hb2002_dvr.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (RTE_SPI1_DMA_EN)
#define EXAMPLE_DMA_BASEADDR (DMA0)
#define EXAMPLE_DMA_CLOCK    kCLOCK_GateDMA
#endif


//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

int pwm_active_channel = -1;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------

/*! -----------------------------------------------------------------------
 *  @brief       Initialize PWM channel
 *  @details     Initialize PWM channel with user define frequency and duty cycle
 *  @return      return the status of PWM initialization
 *  -----------------------------------------------------------------------*/
int setupPWMFromScratch()
{
	int dutyCycle;
	int channel;
	int freq;
	int nonPWMLevel;

	PRINTF("\r\n Enter PWM Channel Number (0 or 1) :- ");
	SCANF("%d",&channel);
	PRINTF("%d\r\n",channel);
	if(channel != 0 && channel != 1)
	{
		PRINTF("\r\n \033[31m Wrong channel number \033[37m");
		return SENSOR_ERROR_INVALID_PARAM;
	}

	PRINTF("\r\n duty cycle (0 to 100 in multiples of 10) :- ");
	SCANF("%d",&dutyCycle);
	PRINTF("%d\r\n", dutyCycle);
	if(dutyCycle < 0 ||  dutyCycle > 100 || dutyCycle % 10 != 0 )
	{
		PRINTF("\r\n \033[31m Wrong duty cycle value entered \033[37m");
		return SENSOR_ERROR_INVALID_PARAM;
	}

	PRINTF("\r\n Enter Frequency in HZ (1500/2000/3000/4000) :- ");
	SCANF("%d", &freq);
	PRINTF("%d\r\n", freq);
	if(freq != 1500 && freq != 2000 && freq != 3000 && freq != 4000)
	{
		PRINTF("\r\n \033[31m Wrong frequency value entered \033[37m");
		return SENSOR_ERROR_INVALID_PARAM;
	}

	pwm_active_channel = channel;

	PRINTF("\r\n Enter Non PWM Channel Level (0/1) :- ");
	SCANF("%d", &nonPWMLevel);
	PRINTF("%d\r\n", nonPWMLevel);
	if(nonPWMLevel != 0 && nonPWMLevel != 1)
	{
		PRINTF("\r\n \033[31m Wrong PWM channel level value entered \033[37m");
		return SENSOR_ERROR_INVALID_PARAM;
	}

	HB2002_SetPin_PWM(channel);
	HB2002_Start_Pwm(channel, freq, dutyCycle);

	HB2002_SetPin_GPIO(1 - channel);
	if(1 - channel)
		HB2002_Pin_write(&HB2002_IN1, nonPWMLevel);
	else
		HB2002_Pin_write(&HB2002_IN0, nonPWMLevel);

	return SENSOR_ERROR_NONE;
}

/*! -----------------------------------------------------------------------
 *  @brief       change duty cycle
 *  @details     change the duty cycle of previously selected channel
 *  @return      return the status of change of duty cycle
 *  -----------------------------------------------------------------------*/
int changeDutyCycle()
{
	int dutyCycle;
	if(pwm_active_channel < 0)
	{
		PRINTF("\r\n \033[31m Please Set PWM Channel first, to change duty cycle \033[37m\r\n");
		return SENSOR_ERROR_INVALID_PARAM;
	}
	else
	{
		PRINTF("\r\n duty cycle (0 to 100 in multiples of 10) :- ");
		SCANF("%d",&dutyCycle);
		PRINTF("%d\r\n", dutyCycle);
		if(dutyCycle < 0 ||  dutyCycle > 100 || dutyCycle % 10 != 0)
		{
			PRINTF("\r\n \033[31m Wrong duty cycle value entered \033[37m");
			return SENSOR_ERROR_INVALID_PARAM;
		}
	}

	HB2002_Update_Duty_Cycle(pwm_active_channel, dutyCycle);
	return SENSOR_ERROR_NONE;
}

/*! -----------------------------------------------------------------------
 *  @brief       initialize PWM and change the duty cycle
 *  @details     initialize PWM with user define frequency and duty cycle
 *               and change the duty cycle  of previously selected channel.
 *  @return      return the status of change of duty cycle or PWM initialization
 *  -----------------------------------------------------------------------*/
int setPWM(void)
{

	int dutyCycle;
	int character;
	int status = SENSOR_ERROR_NONE;
	PRINTF("\r\n 1.  Set PWM channel \r\n");
	PRINTF("\r\n 2.  Change duty cycle  \r\n");
	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
	case 1:       /* initialize PWM */
		status = setupPWMFromScratch();
		break;
	case 2:      /* change duty cycle */
		status = changeDutyCycle();
		break;
	default:
		PRINTF("\r\n Invalid option...chose correct one from Main Menu\r\n");
		break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Get the hb2002 faults from user
 *  @details     Get the hb2002 faults from user to Enable/Disable FS_B
 *  			 and get/clear fault status
 *  @return      return the hb2002 fault
 *  -----------------------------------------------------------------------*/
int get_fault()
{
	int character;

	PRINTF("\r\n 1.  Temperature shutdown \r\n");
	PRINTF("\r\n 2.  Thermal warning \r\n");
	PRINTF("\r\n 3.  Overcurrent - current limit has been activated \r\n");
	PRINTF("\r\n 4.  Open load \r\n");
	PRINTF("\r\n 5.  Short-circuit to ground output 1  \r\n");
	PRINTF("\r\n 6.  Short-circuit to ground output 2  \r\n");
	PRINTF("\r\n 7.  Short-circuit to power output 1  \r\n");
	PRINTF("\r\n 8.  Short-circuit to power output 2  \r\n");
	PRINTF("\r\n 9.  VPWR overvoltage \r\n");
	PRINTF("\r\n 10. VPWR undervoltage \r\n");
	PRINTF("\r\n 11. Charge pump undervoltage \r\n");
	PRINTF("\r\n 12. SPI framing error \r\n");
	PRINTF("\r\n 13. Exit \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	return character;
}

/*! -----------------------------------------------------------------------
 *  @brief       control FS_B pin on faults.
 *  @details     Enable/Disable FS_B pin to follow the hb2002 faults and
 *  			 read FS_B pin.
 *  @return      return the status of  Enable/Disable FS_B pin to follow
 *  		     the hb2002 faults.
 *  -----------------------------------------------------------------------*/
int32_t fs_b_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character, fault;
	uint32_t status = SENSOR_ERROR_NONE, level;
	bool loopcontrol = true, loop = true;

	while(loopcontrol)
	{
		PRINTF("\r\n 1. Enable FS_B (FS_B to become active when fault is active) \r\n");
		PRINTF("\r\n 2. Disable FS_B \r\n");
		PRINTF("\r\n 3. Read FS_B \r\n");
		PRINTF("\r\n 4. Exit \r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&character);
		PRINTF("%d\r\n",character);

		switch (character)
		{
			case 1:  /* Enable FS_B pin to follow the hb2002 faults */
				loop = true;
				while(loop)
				{
					fault = get_fault();
					if(fault >= 1 && fault <= 12)
					{
						status = HB2002_Enable_FS_B(hb2002Dvr, (fault -1));
					}
					else if(fault == 13)
					{
						loop = false;
					}
					else
					{
						PRINTF("\r\n Invalid fault...chose correct one \r\n");
					}
				}
				break;
			case 2:  /* Disable FS_B pin to follow the hb2002 faults */
				loop = true;
				while(loop)
				{
					fault = get_fault();
					if(fault >= 1 && fault <= 12)
					{
						status = HB2002_Disable_FS_B(hb2002Dvr, (fault -1));
					}
					else if(fault == 13)
					{
						loop = false;
					}
					else
					{
						PRINTF("\r\n Invalid fault...chose correct one \r\n");
					}
				}
				break;
			case 3:  /* Read FS_B pin */
				level = HB2002_SF_B_Pin_Read();
				if(!level)
				{
					PRINTF("\r\n \033[31m Error detected \033[37m \r\n");
				}
				else
				{
					PRINTF("\r\n \033[32m NO error detected \033[37m \r\n");
				}
				break;
			case 4:
				loopcontrol = false;
				break;
			default:
				PRINTF("\r\n Invalid option...chose correct one \r\n");
				break;
		}
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       clear the hb2002 fault
 *  @details     get the fault from user and clear the hb2002 fault status
 *  @return      return the status of fault clear
 *  -----------------------------------------------------------------------*/
int32_t Clear_fault_status(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	uint32_t status= SENSOR_ERROR_NONE;
	int fault;

	fault = get_fault();
	if(fault >= 1 && fault <= 12)
	{
		status = HB2002_Clear_Faults(hb2002Dvr, (fault -1));
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       check the hb2002 fault status
 *  @details     get the fault from user and check the status of hb2002 fault
 *  @return      return the status of fault clear
 *  -----------------------------------------------------------------------*/
int32_t Check_fault_status(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	uint32_t status= SENSOR_ERROR_NONE;
	uint8_t  fault_status, fault_status_hold =0 ;
	int fault;

    for(fault = 0; fault < 12 ; fault++)
    {
		switch(fault)
		{
			case OT:  /* check Overtemperature shutdown status */
				status |= HB2002_Check_OT(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == overTempSd)
						PRINTF("\r\n Overtemperature shutdown \r\n");
				}
				else
					PRINTF("\r\n Overtemperature shutdown check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case TW: /* check Thermal warning status */
				status |= HB2002_Check_TW(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == thermalWarn)
						PRINTF("\r\n Thermal warning \r\n");
				}
				else
					PRINTF("\r\n Thermal warning check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case OC:   /* check Overcurrent (current limit has been activated) status */
				status |= HB2002_Check_OC(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == overCurrent)
						PRINTF("\r\n Over current \r\n");
				}
				else
					PRINTF("\r\n Over current check failed\r\n");
				fault_status_hold |= fault_status;
				break;
			case OL:   /* check Open load status */
				status |= HB2002_Check_OL(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == openLoad)
						PRINTF("\r\n Open load \r\n");
				}
				else
					PRINTF("\r\n Open load check failed\r\n");
				fault_status_hold |= fault_status;
				break;
			case SCG_1:  /* check Short-circuit to ground output 1 status */
				status |= HB2002_Check_SC(hb2002Dvr, SCG1, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == shortCircuit)
						PRINTF("\r\n Short-circuit to ground output 1  \r\n");
				}
				else
					PRINTF("\r\n Short-circuit to ground output 1 check failed\r\n");
				fault_status_hold |= fault_status;
				break;
			case SCG_2:  /* check Short-circuit to ground output 2 status */
				status |= HB2002_Check_SC(hb2002Dvr, SCG2, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == shortCircuit)
						PRINTF("\r\n Short-circuit to ground output 2 \r\n");
				}
				else
					PRINTF("\r\n Short-circuit to ground output 2 check failed\r\n");
				fault_status_hold |= fault_status;
				break;
			case SCP_1:  /* check Short-circuit to power output 1 status */
				status |= HB2002_Check_SC(hb2002Dvr, SCP1, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == shortCircuit)
						PRINTF("\r\n Short-circuit to power output 1 \r\n");
				}
				else
					PRINTF("\r\n Short-circuit to power output 1 check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case SCP_2:  /* check Short-circuit to power output 2 status */
				status |= HB2002_Check_SC(hb2002Dvr, SCP2, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == shortCircuit)
						PRINTF("\r\n Short-circuit to power output 2 \r\n");
				}
				else
					PRINTF("\r\n Short-circuit to power output 2 check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case OV:  /* check VPWR overvoltage status */
				status |= HB2002_Check_OV_Vpwr(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == overVoltage)
						PRINTF("\r\n Over voltage \r\n");
				}
				else
					PRINTF("\r\n Over voltage check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case UV:  /* check VPWR undervoltage status */
				status |= HB2002_Check_UV_Vpwr(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == underVoltage)
						PRINTF("\r\n Under voltage \r\n");
				}
				else
					PRINTF("\r\n Under voltage check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case CP_U:  /* check Charge pump undervoltage status */
				status |= HB2002_Check_CP_UV(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == underVoltage)
						PRINTF("\r\n Charge Pump Under voltage \r\n");
				}
				else
					PRINTF("\r\n Charge Pump Under voltage check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			case FRM:  /* check SPI framing error status */
				status |= HB2002_Check_SPI_FRM(hb2002Dvr, &fault_status);
				if(status == SENSOR_ERROR_NONE)
				{
					if(fault_status == spiFrmErr)
						PRINTF("\r\n SPI Frame Error \r\n");
				}
				else
					PRINTF("\r\n SPI Frame Error check failed \r\n");
				fault_status_hold |= fault_status;
				break;
			default:
				break;
		}
	}

    if((status == SENSOR_ERROR_NONE) && (fault_status_hold == 0 ))
    	PRINTF("\r\n No fault detected \r\n");

    return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       overvoltage protection control.
 *  @details     enable/disable overvoltage protection.
 *  @return      return the status of enable/disable overvoltage protection.
 *  -----------------------------------------------------------------------*/
int32_t overvoltage_protection_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;
	FrmError frmerror;

	PRINTF("\r\n 1. Enable Overvoltage protection \r\n");
	PRINTF("\r\n 2. Disable Overvoltage protection \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:    /* Enable overvoltage protection */
			status = HB2002_Enable_OV_Protection(hb2002Dvr);
			break;
		case 2:   /* Disable overvoltage protection */
			status = HB2002_Disable_OV_Protection(hb2002Dvr);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Check for open load control.
 *  @details     Check for open load control on transition from Standby to Normal mode.
 *  @return      return the status of Check for open load control.
 *  -----------------------------------------------------------------------*/
int32_t Check_for_open_load_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;
	FrmError frmerror;

	PRINTF("\r\n 1. Enable Check for open load on transition from Standby to Normal mode \r\n");
	PRINTF("\r\n 2. Disable Check for open load on transition from Standby to Normal mode \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:    /* Enable Check for open load on transition */
			status = HB2002_Enable_CL(hb2002Dvr);
			break;
		case 2:   /* Disable Check for open load on transition */
			status = HB2002_Disable_CL(hb2002Dvr);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Thermal Management control.
 *  @details     enable/disable Thermal Management (ILIM derate to ILIM/2 on OT warning).
 *  @return      return the status of enable/disable Thermal Management.
 *  -----------------------------------------------------------------------*/
int32_t Thermal_Management_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;
	FrmError frmerror;

	PRINTF("\r\n 1. Enable Thermal Management (ILIM derate to ILIM/2 on OT warning) \r\n");
	PRINTF("\r\n 2. Disable Thermal Management (ILIM derate to ILIM/2 from beginning) \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:    /* Enable Thermal Management */
			status = HB2002_Enable_TM(hb2002Dvr);
			break;
		case 2:   /* Disable Thermal Management */
			status = HB2002_Disable_TM(hb2002Dvr);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Set ILimit value.
 *  @details     get the ILimit value from user and set this value.
 *  @return      return the status of Setting of ILimit value.
 *  -----------------------------------------------------------------------*/
int32_t Set_Ilimit(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int ilimit, status = SENSOR_ERROR_NONE;

	PRINTF("\r\n Enter I Limit ( 0 to 3 ) \r\n");
	SCANF("%d",&ilimit);
	PRINTF("%d\r\n",ilimit);

	if(ilimit <=  HB2002_I_MAX  )
		status = HB2002_Set_I_limit(hb2002Dvr, ilimit);
	else
		PRINTF("\r\n Invalid option... Please use correct one \r\n");

	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Active Current Limit mode control.
 *  @details     Enable/Disable active current limit and set ILimit.
 *  @return      return the status of Enable/Disable active current limit or
 *  			 setting of ILimit.
 *  -----------------------------------------------------------------------*/
int32_t Active_Current_Limit_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;

	PRINTF("\r\n 1. Enable Active Current Limit \r\n");
	PRINTF("\r\n 2. Disable Active Current Limit \r\n");
	PRINTF("\r\n 3. Set Active Current Limit \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:  /* Enable Active Current Limit */
			status = HB2002_Enable_I_limit(hb2002Dvr);
			break;
		case 2:   /*  Disable Active Current Limit */
			status = HB2002_Disable_SPI_I_limit(hb2002Dvr);
			break;
		case 3: /* Set Active Current Limit */
			status = Set_Ilimit(hb2002Dvr);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}


/*! -----------------------------------------------------------------------
 *  @brief       motor control using SPI.
 *  @details     Enable/Disable motor control using SPI (set/clear the VIN1 or VIN2).
 *  @return      return the status of motor control operation.
 *  -----------------------------------------------------------------------*/

int32_t Motor_control_using_SPI(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;
	bool loopcontrol = true;

	while(loopcontrol)
	{
		PRINTF("\r\n 1. Enable motor control through SPI \r\n");
		PRINTF("\r\n 2. Disable motor control through SPI \r\n");
		PRINTF("\r\n 3. Set virtual Input one (VIN1) \r\n");
		PRINTF("\r\n 4. Clear virtual Input one (VIN1) \r\n");
		PRINTF("\r\n 5. Set virtual Input two (VIN2) \r\n");
		PRINTF("\r\n 6. Clear virtual Input two (VIN2) \r\n");
		PRINTF("\r\n 7. Exit...Going back to main menu \r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&character);
		PRINTF("%d\r\n",character);

		switch (character)
		{
			case 1:  /* Enable motor control through SPI */
				status |= HB2002_Enable_SPI_Motor_Control(hb2002Dvr);
				break;
			case 2:  /* Disable motor control through SPI */
				status |= HB2002_Disable_SPI_Motor_Control(hb2002Dvr);
				break;
			case 3:  /* Set virtual Input one (VIN1) */
				status |= HB2002_Set_SPI_VIN1(hb2002Dvr);
				break;
			case 4:   /* Clear virtual Input one (VIN1) */
				status |= HB2002_Clear_SPI_VIN1(hb2002Dvr);
				break;
			case 5:   /* Set virtual Input two (VIN2) */
				status |= HB2002_Set_SPI_VIN2(hb2002Dvr);
				break;
			case 6:   /* Clear virtual Input two (VIN2) */
				status |= HB2002_Clear_SPI_VIN2(hb2002Dvr);
				break;
			case 7:  /* exit */
				loopcontrol = false;
				status = SENSOR_ERROR_NONE;
				break;
			default:
				PRINTF("\r\n Invalid option...chose correct one \r\n");
				break;
		}
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Input Control mode.
 *  @details     set input as H-bridge control mode or Half-bridge control mode.
 *  @return      return the status of setting of Input Control mode.
 *  -----------------------------------------------------------------------*/

int32_t hb2002_mode_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status;
	FrmError frmerror;

	PRINTF("\r\n 1. H-bridge control mode \r\n");
	PRINTF("\r\n 2. Half-bridge control mode \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:   /* H-bridge control mode */
			status = HB2002_Mode_Control(hb2002Dvr, hBridge);
			break;
		case 2:  /* Half-bridge control mode */
			status = HB2002_Mode_Control(hb2002Dvr, halfBridge);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Disable pin control.
 *  @details     set/clear  Disable pin.
 *  @return      return the status of Disable pin state change.
 *  -----------------------------------------------------------------------*/

int32_t Disable_pin_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status = SENSOR_ERROR_NONE;
	FrmError frmerror;

	PRINTF("\r\n 1. Set Disable pin High [Standby mode]\r\n");
	PRINTF("\r\n 2. Set Disable pin Low  \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:  /* Set Disable pin */
			HB2002_Pin_write(&HB2002_DIS, HB2002_PIN_HIGH);
			break;
		case 2: /* clear Disable pin */
			HB2002_Pin_write(&HB2002_DIS, HB2002_PIN_LOW);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       output enable pin control.
 *  @details     set/clear  output enable pin.
 *  @return      return the status of output enable pin state change.
 *  -----------------------------------------------------------------------*/
int32_t Output_Enable_control(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int character;
	int status = SENSOR_ERROR_NONE;
	FrmError frmerror;

	PRINTF("\r\n 1. Enable Output using SPI \r\n");
	PRINTF("\r\n 2. Disable Output using SPI \r\n");
	PRINTF("\r\n 3. Enable Output using GPIO \r\n");
	PRINTF("\r\n 4. Disable Output using GPIO [Sleep mode] \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:  /* Enable Output using SPI  */
			status = HB2002_Enable_Output_SPI(hb2002Dvr);
			break;
		case 2:  /* Disable Output using SPI */
			status = HB2002_Disable_Output_SPI(hb2002Dvr);
			break;
		case 3:  /* Enable Output using GPIO */
			HB2002_Pin_write(&HB2002_ENBL, HB2002_PIN_HIGH);
			break;
		case 4:  /* Disable Output using GPIO */
			HB2002_Pin_write(&HB2002_ENBL, HB2002_PIN_LOW);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
	return status;
}

/*! -----------------------------------------------------------------------
 *  @brief       Reset.
 *  @details     When a short-circuit or overtemperature condition is detected,
 *  			 the power outputs are tri-state latched-OFF, independent of
 *  			 the input signals, and the status flag is latched to a logic
 *  			 LOW. To reset from this condition requires the toggling of either
 *  			 DIS, ENBL.
 *  @return      return the status of Setting of slew rate value.
 *  -----------------------------------------------------------------------*/
void HB2002_reset()
{
	int character;

	PRINTF("\r\n 1. Reset using ENBL pin \r\n");
	PRINTF("\r\n 2. Reset using DIS pin \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&character);
	PRINTF("%d\r\n",character);

	switch (character)
	{
		case 1:  /* Reset using ENBL pin  */
			HB2002_enbl_reset(&HB2002_ENBL);
			break;
		case 2:  /* Reset using DIS pin */
			HB2002_dis_reset(&HB2002_DIS);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one \r\n");
			break;
	}
}

/*! -----------------------------------------------------------------------
 *  @brief       Set slew rate value.
 *  @details     get the slew rate value from user and set this value.
 *  @return      return the status of Setting of slew rate value.
 *  -----------------------------------------------------------------------*/
int32_t Set_SlewRate(hb2002_spi_sensorhandle_t *hb2002Dvr)
{
	int swrate, status;

	PRINTF("\r\n Enter Slew Rate ( 0 to 7 )\r\n");
	SCANF("%d",&swrate);
	PRINTF("%d\r\n",swrate);

	if(swrate <= HB2002_MAX_SLEW_RATE )
		status = HB2002_Set_Slew_Rate(hb2002Dvr, swrate);
	else
		PRINTF("\r\n Invalid option... Please use correct one \r\n");

	return status;
}


/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
	int32_t status, i;
	uint8_t character, level;
	uint8_t reg[2], temp;
	uint16_t deviceId;
	int adcVal;

	ARM_DRIVER_SPI *pSPIdriver = &SPI_S_DRIVER;
	GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	hb2002_spi_sensorhandle_t  hb2002Driver;
	
/* Enable EDMA for SPI */
#if (RTE_SPI1_DMA_EN)
	/* Enable DMA clock. */
	CLOCK_EnableClock(EXAMPLE_DMA_CLOCK);
	edma_config_t edmaConfig = {0};
	EDMA_GetDefaultConfig(&edmaConfig);
	EDMA_Init(EXAMPLE_DMA_BASEADDR, &edmaConfig);
#endif

	/* Release peripheral RESET */
	RESET_PeripheralReset(kLPSPI1_RST_SHIFT_RSTn);

	/*! Initialize the MCU hardware. */
	BOARD_InitPins();
	BOARD_InitBootClocks();
	BOARD_SystickEnable();
	BOARD_InitDebugConsole();

	/* initialize Hb2002 pin */
	HB2002_InitPins();

	/* initialize ADC */
	HB2002_Init_adc();

	PRINTF("\r\n ISSDK hb2002 MC driver example demonstration for SPI with interrupt mode.\r\n");

	/*! Initialize the SPI driver. */
	status = pSPIdriver->Initialize(SPI_S_SIGNAL_EVENT);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Initialization Failed\r\n");
		return -1;
	}

	/*! Set the SPI Power mode. */
	status = pSPIdriver->PowerControl(ARM_POWER_FULL);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Power Mode setting Failed\r\n");
		return -1;
	}

	/*! Set the SPI Slave speed. */
	status = pSPIdriver->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1, SPI_S_BAUDRATE);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Control Mode setting Failed\r\n");
		return -1;
	}

	/* Initialize the PCA9957 driver. */
	status = HB2002_SPI_Initialize(&hb2002Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &HB2002_CS);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n hb2002 Sensor Initialization Failed\r\n");
		return -1;
	}

	/* Set HB2002_ENBL pin high */
	HB2002_Pin_write(&HB2002_ENBL, HB2002_PIN_HIGH);

	/* Set HB2002_ENBL pin low */
	HB2002_Pin_write(&HB2002_DIS, HB2002_PIN_LOW);

	while(1)
		{
			PRINTF("\r\n *********** Main Menu ***************\r\n");

			PRINTF("\r\n 1.  Set PWM \r\n");
			PRINTF("\r\n 2.  Stop PWM \r\n");
			PRINTF("\r\n 3.  Device ID \r\n");
			PRINTF("\r\n 4.  FS_B Control \r\n");
			PRINTF("\r\n 5.  Check faults status \r\n");
			PRINTF("\r\n 6.  Clear faults status \r\n");
			PRINTF("\r\n 7.  Over Voltage Protection Control \r\n");
			PRINTF("\r\n 8.  Motor control using SPI \r\n");
			PRINTF("\r\n 9.  Active Current Limit Control \r\n");
			PRINTF("\r\n 10. Input mode Control \r\n");
			PRINTF("\r\n 11. Set SlewRate for SPI \r\n");
			PRINTF("\r\n 12. Output Enable Control \r\n");
			PRINTF("\r\n 13. Disable PIN Control \r\n");
			PRINTF("\r\n 14. Read feedback current \r\n");
			PRINTF("\r\n 15. Reset using DIS/ENB \r\n");
			PRINTF("\r\n 16. Thermal Management Control \r\n");
			PRINTF("\r\n 17. Check for open load control \r\n");

			PRINTF("\r\n Enter your choice :- ");
			SCANF("%d",&character);
			PRINTF("%d\r\n",character);

			switch (character)
			{
			case 1:  /* set frequency, duty cycle of pwm */
				status = setPWM();
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Press enter to go back to main menu \r\n");
					GETCHAR();
				}
				else
				{
					PRINTF("\r\n PWM Generated, Press enter to go back to main menu \r\n");
					GETCHAR();
				}
				break;
			case 2:   /* stop PWM */
				if(pwm_active_channel >= 0)
				{
					HB2002_Stop_Pwm(pwm_active_channel);
				}
				break;
			case 3: /* read Device ID */
				status = HB2002_Read_DevId(&hb2002Driver, &deviceId);
				if (ARM_DRIVER_OK != status)
				{
					PRINTF("\r\n DeviceID get Failed\r\n");
				}
				else
					PRINTF("\r\n DeviceID %d \r\n", deviceId);
				break;
			case 4: /* fs_b control */
				status = fs_b_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Faults control Failed\r\n");
				}
				break;
			case 5: /* check fault status */
				status = Check_fault_status(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Check Fault status Failed\r\n");
				}
				break;
			case 6: /* clear faults status */
				status = Clear_fault_status(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n clear Fault status Failed\r\n");
				}
				break;
			case 7:  /* over voltage protection control */
				status = overvoltage_protection_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Overvoltage Protection Failed\r\n");
				}
				break;
			case 8:  /* motor control using SPI */
				status = Motor_control_using_SPI(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Motor control using SPI Failed\r\n");
				}
				break;
			case 9:  /* Active Current Limit control*/
				status = Active_Current_Limit_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Active current limit set Failed\r\n");
				}
				break;
			case 10: /* input mode control (H-bridge/Half-bridge) */
				status = hb2002_mode_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n hb2002 mode control Failed\r\n");
				}
				break;
			case 11:   /* Set slew rate */
				status = Set_SlewRate(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\nSet slew rate Failed\r\n");
				}
				break;
			case 12: /* control Enable Pin */
				status = Output_Enable_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Output Enable control Failed\r\n");
				}
				break;
			case 13: /* control disable Pin */
				status = Disable_pin_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Disable Pin control Failed\r\n");
				}
				break;
			case 14: /* Read ADC value */
				adcVal = HB2002_Read_adc();
				PRINTF("\r\n Feedback current %d \r\n",adcVal);
				break;
			case 15: /* Reset */
				HB2002_reset();
				break;
			case 16:  /* Thermal Management control */
				status = Thermal_Management_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Thermal Management control Failed\r\n");
				}
				break;
			case 17:  /* Check for open load */
				status = Check_for_open_load_control(&hb2002Driver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Check for open load control Failed\r\n");
				}
				break;
			default:
				break;
			}

		}
		GETCHAR();
}
