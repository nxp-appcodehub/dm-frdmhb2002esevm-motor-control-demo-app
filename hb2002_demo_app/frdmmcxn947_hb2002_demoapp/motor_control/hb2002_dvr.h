/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HB2002_DVR_H_
#define HB2002_DVR_H_

/* Standard C Includes */
#include <stdint.h>

/* ISSDK Includes */
#include "sensor_io_i2c.h"
#include "sensor_io_spi.h"
#include "register_io_i2c.h"
#include "register_io_spi.h"

#include "hb2002.h"

/*--------------------------------
** Enum: FS_B
** @brief Fault Status Bit
** ------------------------------*/
typedef enum FS_B
{
	OT = 0x00,            /* Overtemperature shutdown */
	TW = 0x01,            /* Thermal warning */
	OC = 0x02,            /* Overcurrent - current limit has been activated */
	OL = 0x03,            /* Open load */
	SCG_1 = 0x04,          /* Short-circuit to ground output 1 */
	SCG_2 = 0x05,          /* Short-circuit to ground output 2 */
	SCP_1 = 0x06,          /* Short-circuit to power output 1*/
	SCP_2 = 0x07,          /* Short-circuit to power output 2 */
	OV = 0x08,            /* VPWR overvoltage */
	UV = 0x09,            /* VPWR undervoltage */
	CP_U = 0x0A,          /* Charge pump undervoltage */
	FRM = 0x0B,           /* SPI framing error */
}Fs_B;


/*--------------------------------
** Enum: OVERTEMPSD
** @brief Overtemperature shutdown
** ------------------------------*/
typedef enum OVERTEMPSD
{
	noOverTempSd = 0x00,                 /* over temperature shutdown not occurred */
	overTempSd = 0x01,                   /* over temperature shutdown occurred */
}OverTempSd;

/*--------------------------------
** Enum: THERNALWARNING
** @brief Operating temperature condition
** ------------------------------*/
typedef enum THERMALWARNING
{
	noThermalWarn = 0x00,                 /* Thermal warning not generated */
	thermalWarn = 0x01,                   /* Thermal warning generated */
}ThermalWarning;

/*--------------------------------
** Enum: OPENLOAD
** @brief Open load deducting condition
** ------------------------------*/
typedef enum OPENLOAD
{
	noOpenLoad = 0x00,                 /* No Open load */
	openLoad = 0x01,                   /* Open load */
}OpenLoad;

/*--------------------------------
** Enum: SHORTCIRCUIT
** @brief deducting Short-circuit to power/ground
** ------------------------------*/
typedef enum SHORTCIRCUIT
{
	noShortCircuit = 0x00,                 /* No Short-circuit to power/ground */
	shortCircuit = 0x01,                   /* Short-circuit to power/ground */
}ShortCircuit;

/*--------------------------------
** Enum: SCPWRGND
** @brief Short-circuit to power/ground
** ------------------------------*/
typedef enum SCPWRGND
{
	SCG1 = 0x00,                /* Short-circuit to ground output 1 */
	SCG2 = 0x01,                /* Short-circuit to ground output 2 */
	SCP1 = 0x02,                /* Short-circuit to power output 1 */
	SCP2 = 0x03,                /* Short-circuit to power output 2 */
}ScPwrGnd;

/*--------------------------------
** Enum: OVERCURRENT
** @brief Overcurrent - current limit has been activated
** ------------------------------*/
typedef enum OVERCURRENT
{
	noOverCurrent = 0x00,                 /* current limit has not been activated*/
	overCurrent = 0x01,                   /* current limit has been activated */
}OverCurrent;

/*--------------------------------
** Enum: OVERVOLTAGE
** @brief Operating voltage condition
** ------------------------------*/
typedef enum OVERVOLTAGE
{
	noOverVoltage = 0x00,                 /* Not in over Voltage */
	overVoltage = 0x01,                   /* in over Voltage */
}OverVoltage;

/*--------------------------------
** Enum: UNDERVOLTAGE
** @brief Operating voltage condition
** ------------------------------*/
typedef enum UNDERVOLTAGE
{
	noUnderVoltage = 0x00,                 /* Not in under Voltage */
	underVoltage = 0x01,                   /* in under Voltage */
}UnderVoltage;

/*--------------------------------
** Enum: FRMERROR
** @brief SPI framing error
** ------------------------------*/
typedef enum FRMERROR
{
	noSpiFrmErr = 0x00,                 /* No SPI framing error */
	spiFrmErr = 0x01,                   /* SPI framing error */
}FrmError;

/*--------------------------------
** Enum: MODECONTROL
** @brief half bridge / H bridge condition
** ------------------------------*/
typedef enum MODECONTROL
{
	halfBridge = 0x00,                 /* Half-bridge control mode */
	hBridge = 0x01,                    /* H-bridge control mode */
}ModeControl;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief This defines the sensor specific information for SPI.
 */
typedef struct
{
    registerDeviceInfo_t deviceInfo;      /*!< SPI device context. */
    ARM_DRIVER_SPI *pCommDrv;             /*!< Pointer to the spi driver. */
    bool isInitialized;                   /*!< Whether sensor is intialized or not.*/
    spiSlaveSpecificParams_t slaveParams; /*!< Slave Specific Params.*/
} hb2002_spi_sensorhandle_t;

/*! @def    HB2002_SPI_MAX_MSG_SIZE
 *  @brief  The Maximum size of SPI message. */
#define HB2002_SPI_MAX_MSG_SIZE               (4)

/*! @def    HB2002_REG_SIZE_BYTES
 *  @brief  The size of the Register in bytes. */
#define HB2002_REG_SIZE_BYTES                 (2)

/*! @def    HB2002_SPI_CMD_LEN
 *  @brief  The size of the SPI command for read/write */
#define HB2002_SPI_CMD_LEN                    (0)

/*! @def    HB2002_SS_ACTIVE_VALUE
 *  @brief  Is the Slave Select Pin Active Low or High. */
#define HB2002_SS_ACTIVE_VALUE                SPI_SS_ACTIVE_LOW

/*! @def    HB2002_SPI_WR_CMD
 *  @brief  write command of HB2002 motor control Driver */
#define HB2002_SPI_WR_CMD                     (0x80)

/*! @def    HB2002_SPI_RD_CMD
 *  @brief  Read command of HB2002 motor control Driver */
#define HB2002_SPI_RD_CMD                     (0x00)

/*! @def    HB2002_RST_CMD_ADD
 *  @brief  Read command of HB2002 LED Driver */
//#define HB2002_RST_CMD_ADD                    (0xE000)

/*! @def    HB2002_STATUS_EN
 *  @brief  Enable FS_B to follow faults */
#define HB2002_STATUS_EN                       (1)

/*! @def    HB2002_STATUS_DIS
 *  @brief  Disable FS_B to follow faults */
#define HB2002_STATUS_DIS                      (0)

/*! @def    HB2002_CONFIG_EN
 *  @brief  Enable configuration */
#define HB2002_CONFIG_EN                       (1)

/*! @def    HB2002_CONFIG_DIS
 *  @brief  Disable configuration */
#define HB2002_CONFIG_DIS                      (0)

/*! @def    HB2002_SPI_SET
 *  @brief  SET IN1/IN2 thought SPI  */
#define HB2002_SPI_SET                         (1)

/*! @def    HB2002_SPI_CLEAR
 *  @brief  clear IN1/IN2 thought SPI */
#define HB2002_SPI_CLEAR                       (0)

/*! @def    HB2002_OV_PROTECTION_EN
 *  @brief  Enable overvoltage protection */
#define HB2002_OV_PROTECTION_EN                (0)

/*! @def    HB2002_OV_PROTECTION_DIS
 *  @brief  Disable overvoltage protection */
#define HB2002_OV_PROTECTION_DIS               (1)

/*! @def    HB2002_MAX_SLEW_RATE
 *  @brief  maximum Slew Rate */
#define HB2002_MAX_SLEW_RATE                    (7)

/*! @def    HB2002_I_MAX
 *  @brief  maximum ILIM */
#define HB2002_I_MAX                            (3)

/*! @def    HB2002_PIN_HIGH
 *  @brief  Set GPIO pin high */
#define HB2002_PIN_HIGH                         (1)

/*! @def    HB2002_PIN_LOW
 *  @brief  Set GPIO pin low */
#define HB2002_PIN_LOW                          (0)

/*! @def    HB2002_RESET_DELAY_MS
 *  @brief  reset delay in ms */
#define HB2002_RESET_DELAY_MS                   (1)

/*! @def    Pwm_Channel_1/2
 *  @brief  PWM channel for HB2002 */
#define Pwm_Channel_1                          ((uint8_t)0)
#define Pwm_Channel_2                          ((uint8_t)1)

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*! @brief       stop PWM.
 *  @details     stop PWM for given channel.
 *  @param[in]   pwm_active_channel  		PWM channel zero or one.
 *  @constraints This can be called only after HB2002_Start_Pwm().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Stop_Pwm() returns the status.
 */
int32_t HB2002_Stop_Pwm(int32_t pwm_active_channel);

/*! @brief       configure pins as GPIO.
 *  @details     configure OE, DIS, FS_B and ADC pin as GPIO for HB2002.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_InitPins() returns the status.
 */
int32_t HB2002_InitPins(void);

/*! @brief       Update duty cycle.
 *  @details     Change the duty cycle from last one.
 *  @param[in]   num  		PWM channel zero or one.
 *  @param[in]   dutyCycle  duty cycle[0 to 100 in multiples of 10].
 *  @constraints This can be called only after HB2002_Start_Pwm().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Update_Duty_Cycle() returns the status.
 */
int32_t HB2002_Update_Duty_Cycle(int32_t num, int32_t dutyCycle);

/*! @brief       configure pin for PWM.
 *  @details     configure pin for given PWM channel.
 *  @param[in]   channel  		PWM channel zero or one.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_SetPin_PWM() returns the status.
 */
int32_t HB2002_SetPin_PWM(uint8_t channel);

/*! @brief       Configure and start PWM.
 *  @details     Configure PWM channel with given frequency and duty cycle. Then start PWM.
 *  @param[in]   channel  		PWM channel zero or one.
 *  @param[in]   frequency  	frequency[1500/2000/3000/4000].
 *  @param[in]   dutycycle  	duty cycle[0 to 100 in multiples of 10].
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Start_Pwm() returns the status.
 */
int32_t HB2002_Start_Pwm(int32_t num, int32_t freq, int32_t dutyCycle);

/*! @brief       configure pin as GPIO.
 *  @details     configure pin as GPIO for given PWM channel.
 *  @param[in]   channel  		PWM channel zero or one.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_SetPin_GPIO() returns the status.
 */
int32_t HB2002_SetPin_GPIO(uint8_t channel);

/*! @brief       set pin state.
 *  @details     Set/Clear given pin.
 *  @param[in]   ppin  		pin.
  *  @param[in]  val  		HIGH/LOW.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 */
void HB2002_Pin_write(void *ppin, uint8_t val);

/*! @brief       Initializes the HB2002 Motor control Driver.
 *  @details     Initializes the HB2002 Motor control Driver and its handle.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   pBus  			Pointer to CMSIS API compatible SPI bus object.
 *  @param[in]   index     		Index of the sensor.
 *  @param[in] 	 pSlaveSelect 	Pointer to the slave select pin.
 *  @constraints This should be the first API to be called.
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_SPI_Initialize() returns the status
 */
int32_t HB2002_SPI_Initialize(hb2002_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect);

/*! @brief       Read the device ID.
 *  @details     Read the device ID.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  devid DeviceID.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Read_DevId() returns the status.
 */
int32_t HB2002_Read_DevId(hb2002_spi_sensorhandle_t *pSensorHandle, uint16_t *devid);

/*! @brief       Enable FS_B.
 *  @details     Enable FS_B to follow given faults status. FS_B become active when given fault detected.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   fs_b hb2002 faults.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_FS_B() returns the status.
 */
int32_t HB2002_Enable_FS_B(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b);

/*! @brief       Disable FS_B.
 *  @details     Disable FS_B to follow given faults status.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   fs_b hb2002 faults.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_FS_B() returns the status.
 */
int32_t HB2002_Disable_FS_B(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b);

/*! @brief       clear faults status.
 *  @details     faults status.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   fs_b hb2002 faults.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Clear_Faults() returns the status.
 */
int32_t HB2002_Clear_Faults(hb2002_spi_sensorhandle_t *pSensorHandle, Fs_B fs_b);

/*! @brief       check over temparetue shutdown.
 *  @details     check hb2002 are in over temparetue shutdown or not.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  overtemp       over temparetue shutdown or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_OT() returns the status.
 */
int32_t HB2002_Check_OT(hb2002_spi_sensorhandle_t *pSensorHandle, OverTempSd *overtemp);

/*! @brief       check thermal warning.
 *  @details     check hb2002 have Thermal warning or not.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  thermalwarn    have Thermal warning or not..
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_TW() returns the status.
 */
int32_t HB2002_Check_TW(hb2002_spi_sensorhandle_t *pSensorHandle, ThermalWarning *thermalwarn);

/*! @brief       check Overcurrent.
 *  @details     check current limit has been activated or not.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  ovrcurrent     current limit has been activated or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_OC() returns the status.
 */
int32_t HB2002_Check_OC(hb2002_spi_sensorhandle_t *pSensorHandle, OverCurrent *ovrcurrent);

/*! @brief       check open load.
 *  @details     check  .
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  openload       hb2002 are in open load or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_OL() returns the status.
 */
int32_t HB2002_Check_OL(hb2002_spi_sensorhandle_t *pSensorHandle, OpenLoad *openload);

/*! @brief       check short circuit.
 *  @details     check Short-circuit to ground output 1/2 and to power output 1/2.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  shortcircuit   Short-circuit occurred or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_SC() returns the status.
 */
int32_t HB2002_Check_SC(hb2002_spi_sensorhandle_t *pSensorHandle, ScPwrGnd scpwrgnd, ShortCircuit *shortcircuit);

/*! @brief       check VPWR overvoltage.
 *  @details     check VPWR are above VPWR_OV_HSD(33v to 37v) or not. if VPWR > VPWR_OV_HSD --> overvoltage.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  overvoltage    in overvoltage or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_OV_Vpwr() returns the status.
 */
int32_t HB2002_Check_OV_Vpwr(hb2002_spi_sensorhandle_t *pSensorHandle, OverVoltage *overvoltage);

/*! @brief       check VPWR undervoltage.
 *  @details     check VPWR are below VPWR_FUV(3.55v to 4v) or not. if VPWR < VPWR_FUV --> undervoltage.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  undervoltage    in undervoltage or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_UV_Vpwr() returns the status.
 */
int32_t HB2002_Check_UV_Vpwr(hb2002_spi_sensorhandle_t *pSensorHandle, UnderVoltage *undervoltage);

/*! @brief       check Charge pump undervoltage.
 *  @details     check Charge pump undervoltage or not.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  undervoltage   Charge pump in undervoltage or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_CP_UV() returns the status.
 */
int32_t HB2002_Check_CP_UV(hb2002_spi_sensorhandle_t *pSensorHandle, UnderVoltage *undervoltage);

/*! @brief       check SPI framing error.
 *  @details     check SPI framing error occured or not.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[out]  frmerror       SPI framing error or not.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Check_SPI_FRM() returns the status.
 */
int32_t HB2002_Check_SPI_FRM(hb2002_spi_sensorhandle_t *pSensorHandle, FrmError *frmerror);

/*! @brief       Set VIN1 through SPI.
 *  @details     Set virtual input one through SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Set_SPI_VIN1() returns the status.
 */

int32_t HB2002_Set_SPI_VIN1(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Clear VIN1 through SPI.
 *  @details     Clear virtual input one through SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Clear_SPI_VIN1() returns the status.
 */
int32_t HB2002_Clear_SPI_VIN1(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Set VIN2 through SPI.
 *  @details     Set virtual input two through SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Set_SPI_VIN2() returns the status.
 */
int32_t HB2002_Set_SPI_VIN2(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Clear VIN1 through SPI.
 *  @details     Clear virtual input two through SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Clear_SPI_VIN2() returns the status.
 */
int32_t HB2002_Clear_SPI_VIN2(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable motor control thought SPI.
 *  @details     To set and clear the virtual input 1/2, we need to enable motor control thought SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_SPI_Motor_Control() returns the status.
 */
int32_t HB2002_Enable_SPI_Motor_Control(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable motor control thought SPI.
 *  @details     Disable motor control thought SPI.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_SPI_Motor_Control() returns the status.
 */
int32_t HB2002_Disable_SPI_Motor_Control(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable output.
 *  @details     Enable output control when ENBL pin is high and DIS pin is low.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_Output_SPI() returns the status.
 */
int32_t HB2002_Enable_Output_SPI(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable output.
 *  @details     Disable output control and tri-state outputs.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_Output_SPI() returns the status.
 */
int32_t HB2002_Disable_Output_SPI(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Input Control mode.
 *  @details     Set input Control mode to H-bridge/Half-bridge.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   ModeControl    H-bridge/Half-bridge control.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Mode_Control() returns the status.
 */
int32_t HB2002_Mode_Control(hb2002_spi_sensorhandle_t *pSensorHandle, ModeControl modecontrol);

/*! @brief       Set slew rate.
 *  @details     Set slew rate up to 7.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   slewrate    slew rate.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Set_Slew_Rate() returns the status.
 */
int32_t HB2002_Set_Slew_Rate(hb2002_spi_sensorhandle_t *pSensorHandle, SlewRate slewrate);

/*! @brief       Set ILIM.
 *  @details     Set ILIM up to 3. for zero (4A - 6.8A), one (6A to 8A), two (7.3A to 10.3A ), three (9A to 12.5A).
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   Ilimit    ILIM.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Set_Slew_Rate() returns the status.
 */
int32_t HB2002_Set_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle, Ilimit ilimit);

/*! @brief       Read state of SF_B pin.
 *  @details     Read state of SF_B pin. its low if any faults detects.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   ModeControl    H-bridge/Half-bridge control.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_SF_B_Pin_Read() returns the state of SF_B pin.
 */
int32_t HB2002_SF_B_Pin_Read (void);

/*! @brief       Enable overvoltage protection in Full Bridge mode.
 *  @details     Enable overvoltage protection in Full Bridge mode.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_OV_Protection() returns the status.
 */
int32_t HB2002_Enable_OV_Protection(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable overvoltage protection.
 *  @details     Disable overvoltage protection (OV bit is warning only).
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_OV_Protection() returns the status.
 */
int32_t HB2002_Disable_OV_Protection(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable Active Current Limit mode.
 *  @details     Enable active current limit when overcurrent ILIM threshold has been exceeded.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_AL() returns the status.
 */
int32_t HB2002_Enable_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable Active Current Limit mode.
 *  @details     Disable active current limit. Exceeding overcurrent ILIM threshold set OC flag but does not control outputs.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_AL() returns the status.
 */
int32_t HB2002_Disable_SPI_I_limit(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Read the feedback current.
 *  @details     Read the feedback current by using ADC0 on channel DEMO_LPADC_USER_CHANNEL.
 *  @constraints This can be called any number of times only after HB2002_Init_adc().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Read_adc() returns the ADC value.
 */
int32_t HB2002_Read_adc(void);

/*! @brief       Initialize ADC0.
 *  @details     Initialize ADC0 for channel DEMO_LPADC_USER_CHANNEL.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Init_adc() returns the status.
 */
void HB2002_Init_adc(void);

/*! @brief        Reset HB2002.
 *  @details      Reset using ENBL pin..
 *  @constraints  This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			  Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant    No
 */
void HB2002_enbl_reset(void *ppin);

/*! @brief       Reset HB2002.
 *  @details     Reset using DIS pin.
 *  @constraints Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 */
void HB2002_dis_reset(void *ppin);

/*! @brief       Enable Thermal Management mode.
 *  @details     derate ILIM to ILIM/2 when in OTW state.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_TM() returns the status.
 */
int32_t HB2002_Enable_TM(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable Thermal Management mode.
 *  @details     derate ILIM to ILIM/2 from the beginning.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_TM() returns the status.
 */
int32_t HB2002_Disable_TM(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable Check for open load.
 *  @details     Enable Check for open load on transition from Standby to Normal mode.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Enable_CL() returns the status.
 */
int32_t HB2002_Enable_CL(hb2002_spi_sensorhandle_t *pSensorHandle);

/*! @brief       DIsable Check for open load.
 *  @details     Disable Check for open load on transition from Standby to Normal mode.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *  			 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::HB2002_Disable_CL() returns the status.
 */
int32_t HB2002_Disable_CL(hb2002_spi_sensorhandle_t *pSensorHandle);

uint16_t test_read(hb2002_spi_sensorhandle_t *pSensorHandle, uint8_t offset, uint8_t *temp);

#endif /* HB2002_DVR_H_ */
