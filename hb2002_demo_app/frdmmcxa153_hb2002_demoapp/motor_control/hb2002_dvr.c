/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file  hb2002_drv.c
 * @brief The hb2002_drv.c file implements the hb2002 functional interface.
 */

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "hb2002_dvr.h"

#include "gpio_driver.h"
#include "systick_utils.h"
#include "fsl_pwm.h"
#include "issdk_hal.h"

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

lpadc_conv_result_t g_LpadcResultConfigStruct;
volatile bool g_LpadcConversionCompletedFlag = false;
volatile uint32_t g_LpadcInterruptCounter    = 0U;
const uint32_t g_LpadcFullRange   = 65536U;
const uint32_t g_LpadcResultShift = 0U;

uint8_t hb2002_spiRead_CmdBuffer[HB2002_SPI_MAX_MSG_SIZE] = {0};
uint8_t hb2002_spiRead_DataBuffer[HB2002_SPI_MAX_MSG_SIZE] = {0};
uint8_t hb2002_spiWrite_CmdDataBuffer[HB2002_SPI_MAX_MSG_SIZE] = {0};

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------

void DEMO_LPADC_IRQ_HANDLER_FUNC(void)
{
	g_LpadcInterruptCounter++;

	if (LPADC_GetConvResult(DEMO_LPADC_BASE, &g_LpadcResultConfigStruct))
	{
		g_LpadcConversionCompletedFlag = true;
	}
	SDK_ISR_EXIT_BARRIER;
}

void HB2002_SPI_ReadCmdPreprocess(void *pCmdOut, uint32_t offset, uint32_t size)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = hb2002_spiRead_CmdBuffer;

	/* Formatting for Read command of HB2002 LED driver. */
	*(pWBuff) = (HB2002_SPI_RD_CMD | ( offset << 5));    /* offset is the internal register address of the sensor at which Read is performed. */

	/* Create the slave read command. */
	pSlaveCmd->size = size  ;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = NULL;
}

void HB2002_SPI_ReadPreprocess(void *pCmdOut, uint32_t offset, uint32_t size)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = hb2002_spiRead_CmdBuffer;
	uint8_t *pRBuff = hb2002_spiRead_DataBuffer;

	/* Formatting for Read command of HB2002 LED driver. */
	*(pWBuff) = (HB2002_SPI_RD_CMD | ( offset << 5));  /* offset is the internal register address of the sensor at which Read is performed. */

	/* Create the slave read command. */
	pSlaveCmd->size = size  ;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = pRBuff;
}

void HB2002_SPI_WritePreprocess(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer)
{
    spiCmdParams_t *pSlaveCmd = pCmdOut;

    uint8_t *pWBuff = hb2002_spiWrite_CmdDataBuffer;

    /* Copy the slave write command */
    memcpy(pWBuff, pWritebuffer, size);

    /* Formatting for Write command of HB2002 Motor Controller. */
    *(pWBuff) =  (HB2002_SPI_WR_CMD | (offset << 5)) | (0x1F & *(pWBuff));

    /* Create the slave command. */
    pSlaveCmd->size = size;
    pSlaveCmd->pWriteBuffer = pWBuff;
    pSlaveCmd->pReadBuffer = NULL;
}

int32_t HB2002_SPI_BlockedWrite_With_Mask(hb2002_spi_sensorhandle_t *pSensorHandle, uint8_t offset, uint16_t value, uint16_t mask)
{
    int32_t status;
    uint8_t reg[HB2002_REG_SIZE_BYTES];
    uint16_t actValue = 0;

    status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    			offset, HB2002_REG_SIZE_BYTES, &reg[0]);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	actValue = (int16_t)((int16_t)reg[0] << 8) | (int16_t)reg[1];
	actValue = (actValue & ~mask) | value;

	reg[1] = (uint8_t)actValue;
	reg[0] = (uint8_t)(actValue >> 8);

    status = Register_SPI_BlockWrite(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    			offset, &reg[0] , HB2002_REG_SIZE_BYTES);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}


void HB2002_Init_adc()
{
	lpadc_config_t mLpadcConfigStruct;

	lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
	lpadc_conv_command_config_t mLpadcCommandConfigStruct;

	CLOCK_SetClockDiv(kCLOCK_DivADC0, 1u);
	CLOCK_AttachClk(kFRO12M_to_ADC0);

	/* get default configuration */
	LPADC_GetDefaultConfig(&mLpadcConfigStruct);
	mLpadcConfigStruct.enableAnalogPreliminary = true;
	mLpadcConfigStruct.referenceVoltageSource = DEMO_LPADC_VREF_SOURCE;
	mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;

	LPADC_Init(DEMO_LPADC_BASE, &mLpadcConfigStruct);

	LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);

	LPADC_DoAutoCalibration(DEMO_LPADC_BASE);

	/* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&mLpadcCommandConfigStruct);
	mLpadcCommandConfigStruct.channelNumber = DEMO_LPADC_USER_CHANNEL;
	mLpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
	LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, DEMO_LPADC_USER_CMDID, &mLpadcCommandConfigStruct);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
	mLpadcTriggerConfigStruct.targetCommandId       = DEMO_LPADC_USER_CMDID; /* CMD15 is executed. */
	mLpadcTriggerConfigStruct.enableHardwareTrigger = false;
	LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &mLpadcTriggerConfigStruct); /* Configurate the trigger0. */
	LPADC_EnableInterrupts(DEMO_LPADC_BASE, kLPADC_FIFOWatermarkInterruptEnable);
	EnableIRQ(DEMO_LPADC_IRQn);
}

int32_t HB2002_Read_adc(void)
{
	LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */
	while (!g_LpadcConversionCompletedFlag)
	{
	}

	g_LpadcConversionCompletedFlag = false;

	return g_LpadcResultConfigStruct.convValue;
}

int32_t HB2002_SetPin_PWM(uint8_t channel)
{

	GENERIC_DRIVER_GPIO *gpioDriver = &Driver_GPIO_KSDK;
	PORT_Type *ports[] = PORT_BASE_PTRS;
	PORT_Type port;
	int portNum;
	int pinNum;

	if(channel < 0)
			return SENSOR_ERROR_INVALID_PARAM;

	port_pin_config_t HB2002_PWM_CONFIG = {/* Internal pull-up/down resistor is disabled */
						kPORT_PullDisable,
						/* Low internal pull resistor value is selected. */
						kPORT_LowPullResistor,
						/* Fast slew rate is configured */
						kPORT_FastSlewRate,
						/* Passive input filter is disabled */
						kPORT_PassiveFilterDisable,
						/* Open drain output is disabled */
						kPORT_OpenDrainDisable,
						/* Low drive strength is configured */
						kPORT_LowDriveStrength,
						/* Normal drive strength is configured */
						kPORT_NormalDriveStrength,
						/* Pin is configured as P3_12 */
						kPORT_MuxAlt5,
						/* Digital input enabled */
						kPORT_InputBufferEnable,
						/* Digital input is not inverted */
						kPORT_InputNormal,
						/* Pin Control Register fields [15:0] are not locked */
						kPORT_UnlockRegister
				};


	if(channel)
	{
		portNum = HB2002_IN1.portNumber;
		pinNum = HB2002_IN1.pinNumber;
	}
	else
	{
		portNum = HB2002_IN0.portNumber;
		pinNum = HB2002_IN0.pinNumber;
	}

	/* Pin configuration */
	PORT_SetPinConfig( ports[portNum] , pinNum , &HB2002_PWM_CONFIG);

	/* Pin initialization */
	if(channel)
		gpioDriver->pin_init(&HB2002_IN1, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	else
		gpioDriver->pin_init(&HB2002_IN0, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
}

int32_t HB2002_SetPin_GPIO(uint8_t channel)
{

	GENERIC_DRIVER_GPIO *gpioDriver = &Driver_GPIO_KSDK;
	PORT_Type *ports[] = PORT_BASE_PTRS;
	PORT_Type port;
	int portNum;
	int pinNum;

	if(channel < 0)
		return SENSOR_ERROR_INVALID_PARAM;

	port_pin_config_t HB2002_GPIO_CONFIG = {/* Internal pull-up/down resistor is disabled */
						kPORT_PullDisable,
						/* Low internal pull resistor value is selected. */
						kPORT_LowPullResistor,
						/* Fast slew rate is configured */
						kPORT_FastSlewRate,
						/* Passive input filter is disabled */
						kPORT_PassiveFilterDisable,
						/* Open drain output is disabled */
						kPORT_OpenDrainDisable,
						/* Low drive strength is configured */
						kPORT_LowDriveStrength,
						/* Normal drive strength is configured */
						kPORT_NormalDriveStrength,
						/* Pin is configured GPIO */
						kPORT_MuxAlt0,
						/* Digital input enabled */
						kPORT_InputBufferEnable,
						/* Digital input is not inverted */
						kPORT_InputNormal,
						/* Pin Control Register fields [15:0] are not locked */
						kPORT_UnlockRegister
			};

	if(channel)
	{
		portNum = HB2002_IN1.portNumber;
		pinNum = HB2002_IN1.pinNumber;
	}
	else
	{
		portNum = HB2002_IN0.portNumber;
		pinNum = HB2002_IN0.pinNumber;
	}

	/* Pin configuration */
	PORT_SetPinConfig(ports[portNum], pinNum, &HB2002_GPIO_CONFIG);

	/* Pin initialization */
	if(channel)
		gpioDriver->pin_init(&HB2002_IN1, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	else
		gpioDriver->pin_init(&HB2002_IN0, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

}

int32_t HB2002_InitPins(void)
{

    const port_pin_config_t port3_15_config = {/* Internal pull-up/down resistor is disabled */
                                           kPORT_PullDisable,
                                           /* Low internal pull resistor value is selected. */
                                           kPORT_LowPullResistor,
                                           /* Fast slew rate is configured */
                                           kPORT_FastSlewRate,
                                           /* Passive input filter is disabled */
                                           kPORT_PassiveFilterDisable,
                                           /* Open drain output is disabled */
                                           kPORT_OpenDrainDisable,
                                           /* Low drive strength is configured */
                                           kPORT_LowDriveStrength,
                                           /* Normal drive strength is configured */
                                           kPORT_NormalDriveStrength,
                                           /* Pin is configured as GPIO */
                                           kPORT_MuxAlt0,
                                           /* Digital input enabled */
                                           kPORT_InputBufferEnable,
                                           /* Digital input is not inverted */
                                           kPORT_InputNormal,
                                           /* Pin Control Register fields [15:0] are not locked */
                                           kPORT_UnlockRegister};
        /* PORT3_15 is configured as P3_15 */
    PORT_SetPinConfig(PORT3, 15U, &port3_15_config);

    const port_pin_config_t port3_14_config = {/* Internal pull-up/down resistor is disabled */
                                               kPORT_PullDisable,
                                               /* Low internal pull resistor value is selected. */
                                               kPORT_LowPullResistor,
                                               /* Fast slew rate is configured */
                                               kPORT_FastSlewRate,
                                               /* Passive input filter is disabled */
                                               kPORT_PassiveFilterDisable,
                                               /* Open drain output is disabled */
                                               kPORT_OpenDrainDisable,
                                               /* Low drive strength is configured */
                                               kPORT_LowDriveStrength,
                                               /* Normal drive strength is configured */
                                               kPORT_NormalDriveStrength,
                                               /* Pin is configured as GPIO */
                                               kPORT_MuxAlt0,
                                               /* Digital input enabled */
                                               kPORT_InputBufferEnable,
                                               /* Digital input is not inverted */
                                               kPORT_InputNormal,
                                               /* Pin Control Register fields [15:0] are not locked */
                                               kPORT_UnlockRegister};
            /* PORT3_15 is configured as P3_15 */
        PORT_SetPinConfig(PORT3, 14U, &port3_14_config);

        const port_pin_config_t port2_3_config = {/* Internal pull-up/down resistor is disabled */
                                                      kPORT_PullDisable,
                                                      /* Low internal pull resistor value is selected. */
                                                      kPORT_LowPullResistor,
                                                      /* Fast slew rate is configured */
                                                      kPORT_FastSlewRate,
                                                      /* Passive input filter is disabled */
                                                      kPORT_PassiveFilterDisable,
                                                      /* Open drain output is disabled */
                                                      kPORT_OpenDrainDisable,
                                                      /* Low drive strength is configured */
                                                      kPORT_LowDriveStrength,
                                                      /* Normal drive strength is configured */
                                                      kPORT_NormalDriveStrength,
                                                      /* Pin is configured as GPIO */
                                                      kPORT_MuxAlt0,
                                                      /* Digital input enabled */
                                                      kPORT_InputBufferEnable,
                                                      /* Digital input is not inverted */
                                                      kPORT_InputNormal,
                                                      /* Pin Control Register fields [15:0] are not locked */
                                                      kPORT_UnlockRegister};

           /* PORT3_15 is configured as P3_15 */
		   PORT_SetPinConfig(PORT2, 3U, &port2_3_config);

		   const port_pin_config_t port2_0_pin14_config = {/* Internal pull-up/down resistor is disabled */
		   			kPORT_PullDisable,
		   			/* Low internal pull resistor value is selected. */
		   			kPORT_LowPullResistor,
		   			/* Fast slew rate is configured */
		   			kPORT_FastSlewRate,
		   			/* Passive input filter is disabled */
		   			kPORT_PassiveFilterDisable,
		   			/* Open drain output is disabled */
		   			kPORT_OpenDrainDisable,
		   			/* Low drive strength is configured */
		   			kPORT_LowDriveStrength,
		   			/* Normal drive strength is configured */
		   			kPORT_NormalDriveStrength,
		   			/* Pin is configured as P3_12 */
		   			kPORT_MuxAlt0,
		   			/* Digital input enabled */
		   			kPORT_InputBufferEnable,
		   			/* Digital input is not inverted */
		   			kPORT_InputNormal,
		   			/* Pin Control Register fields [15:0] are not locked */
		   			kPORT_UnlockRegister};
		   	/* PORT2_0 (pin 14) is configured as SF_B */
		   	PORT_SetPinConfig(PORT2, 0U, &port2_0_pin14_config);
}

void HB2002_enbl_reset(void *ppin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 pGPIODriver->clr_pin(ppin);
	 BOARD_DELAY_ms(HB2002_RESET_DELAY_MS);
	 pGPIODriver->set_pin(ppin);
}

void HB2002_dis_reset(void *ppin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 pGPIODriver->set_pin(ppin);
	 BOARD_DELAY_ms(HB2002_RESET_DELAY_MS);
	 pGPIODriver->clr_pin(ppin);
}

void HB2002_Pin_write(void *ppin, uint8_t val)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 if(val)
		 pGPIODriver->set_pin(ppin);
	 else
		 pGPIODriver->clr_pin(ppin);
}

int32_t HB2002_Update_Duty_Cycle(int32_t num, int32_t dutyCycle)
{
	if (num == 0)
		{
			PWM_UpdatePwmDutycycle(BOARD_PWM_BASEADDR, kPWM_Module_0, kPWM_PwmA, kPWM_EdgeAligned, dutyCycle);
		}
		else
		{
			PWM_UpdatePwmDutycycle(BOARD_PWM_BASEADDR, kPWM_Module_1, kPWM_PwmA, kPWM_EdgeAligned, dutyCycle);
		}
		PWM_SetPwmLdok(BOARD_PWM_BASEADDR, kPWM_Control_Module_0 | kPWM_Control_Module_1, true);
}


static void HB2002_PWM_DRV_InitPwm(int32_t num, int freq, int dutyCycle)
{
	uint16_t deadTimeVal;
		pwm_signal_param_t pwmSignal[3];
		uint32_t pwmSourceClockInHz;
		uint32_t pwmFrequencyInHz = freq;
		pwm_submodule_t submod;

		pwmSourceClockInHz = PWM_SRC_CLK_FREQ;

		/* Set deadtime count, we set this to about 650ns */
		deadTimeVal = ((uint64_t)pwmSourceClockInHz * 650) / 1000000000;

		if(num)
			submod = kPWM_Module_1;
		else
			submod = kPWM_Module_0;

		pwmSignal[0].pwmChannel       = kPWM_PwmA;
		pwmSignal[0].level            = kPWM_LowTrue;       /* Should be low true when edge aligned PWM. */

		pwmSignal[0].dutyCyclePercent = dutyCycle;

		pwmSignal[0].faultState       = kPWM_PwmFaultState0;
		pwmSignal[0].pwmchannelenable = true;

		/*********** PWMA_SM0 - phase A, configuration, setup 2 channel as an example ************/
		PWM_SetupPwm(BOARD_PWM_BASEADDR, submod, pwmSignal, 1, kPWM_EdgeAligned, pwmFrequencyInHz,
				pwmSourceClockInHz);
}

int32_t HB2002_Start_Pwm(int32_t num, int32_t freq, int32_t dutyCycle)
{
	pwm_config_t pwmConfig;
	pwm_fault_param_t faultConfig;

	PWM_GetDefaultConfig(&pwmConfig);
	pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
	pwmConfig.enableDebugMode = true;

    /* Initialize submodule 0, make it use same counter clock as submodule 0. */
	if (PWM_Init(BOARD_PWM_BASEADDR, kPWM_Module_0, &pwmConfig) == kStatus_Fail)
	{
		return SENSOR_ERROR_INIT;
	}

	pwmConfig.clockSource           = kPWM_Submodule0Clock;
	pwmConfig.prescale              = kPWM_Prescale_Divide_4;
	pwmConfig.initializationControl = kPWM_Initialize_MasterSync;

	/* Initialize submodule 1 */
	if (PWM_Init(BOARD_PWM_BASEADDR, kPWM_Module_1, &pwmConfig) == kStatus_Fail)
	{
		return SENSOR_ERROR_INIT;
	}

	/*
	 *   config->faultClearingMode = kPWM_Automatic;
	 *   config->faultLevel = false;
	 *   config->enableCombinationalPath = true;
	 *   config->recoverMode = kPWM_NoRecovery;
	 */
	PWM_FaultDefaultConfig(&faultConfig);

#ifdef DEMO_PWM_FAULT_LEVEL
	faultConfig.faultLevel = true;
#endif

	/* Sets up the PWM fault protection */
	PWM_SetupFaults(BOARD_PWM_BASEADDR, kPWM_Fault_0, &faultConfig);
	PWM_SetupFaults(BOARD_PWM_BASEADDR, kPWM_Fault_1, &faultConfig);
	PWM_SetupFaults(BOARD_PWM_BASEADDR, kPWM_Fault_2, &faultConfig);
	PWM_SetupFaults(BOARD_PWM_BASEADDR, kPWM_Fault_3, &faultConfig);

	/* Set PWM fault disable mapping for submodule 0/1/2 */
	PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, kPWM_Module_0, kPWM_PwmA, kPWM_faultchannel_0,
			kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3);
	PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, kPWM_Module_1, kPWM_PwmA, kPWM_faultchannel_0,
			kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3);
	PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, kPWM_Module_2, kPWM_PwmA, kPWM_faultchannel_0,
			kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3);

	/* Call the init function with demo configuration */
	HB2002_PWM_DRV_InitPwm(num, freq, dutyCycle);

	/* Set the load okay bit for all submodules to load registers from their buffer */
	PWM_SetPwmLdok(BOARD_PWM_BASEADDR, kPWM_Control_Module_0 | kPWM_Control_Module_1 , true);
	/* Start the PWM generation from Submodules 0, 1 and 2 */

	PWM_StartTimer(BOARD_PWM_BASEADDR, kPWM_Control_Module_0 |  kPWM_Control_Module_1);
	return SENSOR_ERROR_NONE;

}

int32_t HB2002_Stop_Pwm(int32_t pwm_active_channel)
{

	PWM_Deinit(BOARD_PWM_BASEADDR, kPWM_Module_0);
	PWM_Deinit(BOARD_PWM_BASEADDR, kPWM_Module_1);

	if(pwm_active_channel < 0)
		return SENSOR_ERROR_INVALID_PARAM;

	HB2002_SetPin_GPIO(Pwm_Channel_1);
	HB2002_SetPin_GPIO(Pwm_Channel_2);
	HB2002_Pin_write(&HB2002_IN0, Pwm_Channel_1);
	HB2002_Pin_write(&HB2002_IN1, Pwm_Channel_1);

	pwm_active_channel = -1;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_SPI_Initialize(hb2002_spi_sensorhandle_t *pSensorHandle,
										ARM_DRIVER_SPI *pBus, uint8_t index,
										void *pSlaveSelect)
{
    int32_t status;
    uint8_t reg;
    GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

    /*! Check the input parameters. */
    if ((pSensorHandle == NULL) || (pBus == NULL) || (pSlaveSelect == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Initialize the sensor handle. */
    pSensorHandle->pCommDrv = pBus;
    pSensorHandle->slaveParams.pReadPreprocessFN =  HB2002_SPI_ReadPreprocess;
    pSensorHandle->slaveParams.pWritePreprocessFN = HB2002_SPI_WritePreprocess;

    pSensorHandle->slaveParams.pReadCmdPreprocessFN = HB2002_SPI_ReadCmdPreprocess;

    pSensorHandle->slaveParams.pTargetSlavePinID = pSlaveSelect;
    pSensorHandle->slaveParams.spiCmdLen = HB2002_SPI_CMD_LEN;
    pSensorHandle->slaveParams.ssActiveValue = HB2002_SS_ACTIVE_VALUE;
    pSensorHandle->deviceInfo.deviceInstance = index;
    pSensorHandle->deviceInfo.functionParam = NULL;
    pSensorHandle->deviceInfo.idleFunction = NULL;

    /* Initialize the Slave Select Pin. */
    pGPIODriver->pin_init(pSlaveSelect, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    if (pSensorHandle->slaveParams.ssActiveValue == SPI_SS_ACTIVE_LOW)
    {
        pGPIODriver->set_pin(pSlaveSelect);
    }
    else
    {
        pGPIODriver->clr_pin(pSlaveSelect);
    }

    pGPIODriver->pin_init(&HB2002_ENBL, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    pGPIODriver->pin_init(&HB2002_DIS, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    pGPIODriver->pin_init(&HB2002_FS_B, GPIO_DIRECTION_IN, NULL, NULL, NULL);

    pSensorHandle->isInitialized = true;
    return SENSOR_ERROR_NONE;
}

int32_t HB2002_Output_Disable(void *pDis)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set OE pin High */
	 pGPIODriver->set_pin(pDis);

	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Output_Enable(void *pEnb)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set OE pin High */
	 pGPIODriver->clr_pin(pEnb);

	 return SENSOR_ERROR_NONE;
}


int32_t HB2002_Enable_FS_B(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable FS_B to follow faults */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_FaultStatusMask,
			(uint16_t)(HB2002_STATUS_EN << fs_b),(uint16_t)(HB2002_ONE_BIT_MASK << fs_b));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Clear_Faults(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* clear hb2002 faults */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_Status,
			(uint16_t)(HB2002_STATUS_EN << fs_b),(uint16_t)(HB2002_ONE_BIT_MASK << fs_b));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}


int32_t HB2002_Disable_FS_B(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable FS_B to follow faults */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_FaultStatusMask,
			(uint16_t)(HB2002_STATUS_DIS << fs_b),(uint16_t)(HB2002_ONE_BIT_MASK << fs_b));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_OT(hb2002_spi_sensorhandle_t *pSensorHandle, OverTempSd *overtemp)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to overtemp */
	if ((pSensorHandle == NULL) || (overtemp == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return ot bit */
	*overtemp = hb2002_status.b.ot;

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_TW(hb2002_spi_sensorhandle_t *pSensorHandle, ThermalWarning *thermalwarn)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to thermalwarn */
	if ((pSensorHandle == NULL) || (thermalwarn == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return tw bit */
	*thermalwarn = hb2002_status.b.tw;

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_OC(hb2002_spi_sensorhandle_t *pSensorHandle, OverCurrent *ovrcurrent)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to ovrcurrent*/
	if ((pSensorHandle == NULL) || (ovrcurrent == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return oc bit */
	*ovrcurrent = hb2002_status.b.oc;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_OL(hb2002_spi_sensorhandle_t *pSensorHandle, OpenLoad *openload)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to openload */
	if ((pSensorHandle == NULL) || (openload == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	 status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_READ;
	 }

	 /* return ol bit */
	 *openload = hb2002_status.b.ol;

	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_SC(hb2002_spi_sensorhandle_t *pSensorHandle, ScPwrGnd scpwrgnd, ShortCircuit *shortcircuit)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to shortcircuit */
	if ((pSensorHandle == NULL) || (shortcircuit == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
		 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	switch(scpwrgnd)
	{
		case SCG1:   /* return scg1 bit */
		{
			*shortcircuit = hb2002_status.b.scg1;
			break;
		}
		case SCG2:   /* return scg2 bit */
		{
			*shortcircuit = hb2002_status.b.scg2;
			break;
		}
		case SCP1:   /* return scp1 bit */
		{
			*shortcircuit = hb2002_status.b.scp1;
			break;
		}
		case SCP2:   /* return scp2 bit */
		{
			*shortcircuit = hb2002_status.b.scp2;
			break;
		}
		default:
			break;
	}
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_OV_Vpwr(hb2002_spi_sensorhandle_t *pSensorHandle, OverVoltage *overvoltage)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to overvoltage */
	if ((pSensorHandle == NULL) || (overvoltage == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
		 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return OV bit */
	*overvoltage = hb2002_status.b.ov;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_UV_Vpwr(hb2002_spi_sensorhandle_t *pSensorHandle, UnderVoltage *undervoltage)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to undervoltage */
	if ((pSensorHandle == NULL) || (undervoltage == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
		 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return UV bit */
	*undervoltage = hb2002_status.b.uv;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_CP_UV(hb2002_spi_sensorhandle_t *pSensorHandle, UnderVoltage *undervoltage)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to undervoltage*/
	if ((pSensorHandle == NULL) || (undervoltage == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
		 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return cp_u bit */
	*undervoltage = hb2002_status.b.cp_u;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Check_SPI_FRM(hb2002_spi_sensorhandle_t *pSensorHandle, FrmError *frmerror)
{
	int32_t status;
	HB2002_STATUS hb2002_status;

	/*! Validate for the correct handle and pointer to frmerror */
	if ((pSensorHandle == NULL) || (frmerror == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read HB2002_Status */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
		 HB2002_Status, HB2002_REG_SIZE_BYTES, (uint8_t *)&hb2002_status);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}

	/* return frm bit */
	*frmerror = hb2002_status.b.frm;
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_OV_Protection(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable overvoltage protection in Full Bridge mode */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_FaultStatusMask,
			 	 (uint16_t)(HB2002_OV_PROTECTION_EN << HB2002_DOV_SHIFT), HB2002_DOV_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_OV_Protection(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable overvoltage protection (OV bit is warning only)*/
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_FaultStatusMask,
				 (uint16_t)(HB2002_OV_PROTECTION_DIS << HB2002_DOV_SHIFT), HB2002_DOV_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Set_SPI_VIN1(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Set VIN1 through SPI */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_SPI_SET << HB2002_VIN1_SHIFT), HB2002_VIN1_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Clear_SPI_VIN1(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* clear VIN1 through SPI */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_SPI_CLEAR << HB2002_VIN1_SHIFT), HB2002_VIN1_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Set_SPI_VIN2(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Set VIN2 through SPI */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_SPI_SET << HB2002_VIN2_SHIFT), HB2002_VIN2_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Clear_SPI_VIN2(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* clear VIN2 through SPI */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_SPI_CLEAR << HB2002_VIN2_SHIFT), HB2002_VIN2_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_SPI_Motor_Control(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable motor control through SPI */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_CONFIG_EN << HB2002_INPUT_SHIFT), HB2002_INPUT_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_SPI_Motor_Control(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable motor control through SPI */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_CONFIG_DIS << HB2002_INPUT_SHIFT), HB2002_INPUT_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}


int32_t HB2002_Mode_Control(hb2002_spi_sensorhandle_t *pSensorHandle, ModeControl modecontrol)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Set Input Control mode [ H-bridge or Half-bridge ] */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(modecontrol << HB2002_MODE_SHIFT), HB2002_MODE_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_Output_SPI(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* ENABLE output control when ENBL pin is high and DIS pin is low */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_CONFIG_EN << HB2002_EN_SHIFT), HB2002_EN_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_Output_SPI(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* DISABLE output control and tri-state outputs */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_CONFIG_DIS << HB2002_EN_SHIFT), HB2002_EN_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Set_Slew_Rate(hb2002_spi_sensorhandle_t *pSensorHandle, SlewRate slewrate)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	if( slewrate <= HB2002_MAX_SLEW_RATE )   /* Validate Slew rate value */
	{
		/* set SR 0/1/2 value */
		status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(slewrate << HB2002_SR_SHIFT), HB2002_SR_MASK);
		if (ARM_DRIVER_OK != status)
		{
			return SENSOR_ERROR_WRITE;
		}
	}
	else
		return SENSOR_ERROR_INVALID_PARAM;

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Set_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle, Ilimit ilimit)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	if( ilimit <= HB2002_I_MAX ) /* Validate ilimit value */
	{
		/* set ILIM 1/0 value */
		status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(ilimit << HB2002_ILM_SHIFT), HB2002_ILM_MASK);
		if (ARM_DRIVER_OK != status)
		{
			return SENSOR_ERROR_WRITE;
		}
	}
	else
		return SENSOR_ERROR_INVALID_PARAM;

	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable Active Current Limit mode */
	 status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_CONFIG_EN << HB2002_AL_SHIFT), HB2002_AL_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_SPI_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable Active Current Limit mode */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_CONFIG_DIS << HB2002_AL_SHIFT), HB2002_AL_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Read_DevId(hb2002_spi_sensorhandle_t *pSensorHandle, uint16_t *devid)
{
	int32_t status;
	uint16_t deviceid;

	/*! Validate for the correct handle  and device ID pointer */
	if ((pSensorHandle == NULL) || (devid == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Read Device ID */
	 status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			 	 HB2002_DeviceIdentification, HB2002_REG_SIZE_BYTES, (uint8_t *)&deviceid);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_READ;
	 }

	 deviceid =  ((deviceid << 8) | (deviceid >> 8));    /* update device iD */
	 *devid = (deviceid >> HB2002_DEV_ID_SHIFT);

	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_TM(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable Thermal Management mode */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_CONFIG_EN << HB2002_TM_SHIFT), HB2002_TM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_TM(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable Thermal Management mode */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_CONFIG_DIS << HB2002_TM_SHIFT), HB2002_TM_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_Enable_CL(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Enable Check for open load (in Full Bridge Standby mode) on transition from Standby to Normal mode */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
			 	 (uint16_t)(HB2002_CONFIG_EN << HB2002_CL_SHIFT), HB2002_CL_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}
	return SENSOR_ERROR_NONE;
}

int32_t HB2002_Disable_CL(hb2002_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/* Disable Check for open load on transition from Standby to Normal mode */
	status = HB2002_SPI_BlockedWrite_With_Mask(pSensorHandle, HB2002_ConfigurationControl,
				 (uint16_t)(HB2002_CONFIG_DIS << HB2002_CL_SHIFT), HB2002_CL_MASK);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_WRITE;
	 }
	 return SENSOR_ERROR_NONE;
}

int32_t HB2002_SF_B_Pin_Read (void)
{
	GENERIC_DRIVER_GPIO *gpioDriver = &Driver_GPIO_KSDK;
	return gpioDriver->read_pin(&HB2002_FS_B);   /* return FS_B pin state */
}


uint16_t test_read(hb2002_spi_sensorhandle_t *pSensorHandle, uint8_t offset, uint8_t *temp)
{
	 int32_t status;


	 status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
	     			offset, HB2002_REG_SIZE_BYTES, temp);
	 if (ARM_DRIVER_OK != status)
	 {
		return SENSOR_ERROR_READ;
	 }
}




