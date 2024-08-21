# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## FRDM-HB2002ESEVM Motor Controller driver with demo app running on FRDM-MCXN947 and FRDM-MCXA153

This demo application provides a command line interface which allows the user to easily explore the different functions of the driver to use the Motor controller features like controlling motor with PWM or SPI, checking faults like over-voltage, under-voltage, over-current etc with two input modes H-Bridge and Half-Bridge.

### FRDM-HB2002ESEVM Block Diagram

[<img src="./images/HB2002ESEVM_Block_Diagram.PNG" width="700"/>](HB2002ESEVM_Block_Diagram.PNG)

### Key Features of FRDM-HB2002ESEVM Motor Controller Driver

- Advanced diagnostic reporting via a serial peripheral interface (SPI): charge pump
undervoltage, overvoltage, and undervoltage on VPWR, short to ground and short
to VPWR for each output, open load, temperature warning and overtemperature
shutdown.
- Eight selectable slew rates via the SPI: 0.25 V/μs to more than 16 V/μs for EMI and
thermal performance optimization
- Four selectable current limits via the SPI: 5.4/7.0/8.8/10.7 A covering a wide range of
applications
- Extended high temperature operating range with current foldback while limiting the
current
- Can be operated without SPI with default slew rate of 2.0 V/μs and a 7.0 A current limit
threshold
- Drives inductive loads in a full H-bridge or Half-bridge configuration
- Wide operating range: 5.0 V to 28 V operation

#### Boards: FRDM-MCXA153, FRDM-MCXN947
#### Categories: Motor Control
#### Peripherals: ADC,GPIO, PWM, SPI
#### Toolchains: MCUXpresso IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
- [IoT Sensing SDK (ISSDK) v1.8](https://nxp.com/iot-sensing-sdk) offered as middleware in MCUXpresso SDK for supported platforms
- [MCUXpresso IDE v11.9.0](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)


## 2. Hardware<a name="step2"></a>
- FRDM-MCXN947 and FRDM-MCXA153 MCU board
- [FRDM-HB2002ESEVM Motor Controller Driver](https://www.nxp.com/part/FRDM-HB2002ESEVM) 
- Personal Computer
- Mini/micro C USB cable

## 3. Setup<a name="step3"></a>
### 3.1 Step 1: Download and Install required Software(s)
- Install MCUXpresso IDE 11.9.0
- Download and Install [MCUXpresso SDK v2.14.0 for FRDM-MCXN947](https://mcuxpresso.nxp.com/en/builder?hw=FRDM-MCXN947). Make sure to select ISSDK  middleware while building SDK.
- Download and Install [MCUXpresso SDK v2.14.2 for FRDM-MCXA153](https://mcuxpresso.nxp.com/en/builder?hw=FRDM-MCXA153). Make sure to select ISSDK  middleware while building SDK.
- Install Git v2.39.0 (for cloning and running west commands).
- Install Putty/Teraterm for UART.
 
### 3.2 Step 2: Clone the APP-CODE-HUB/dm-frdmhb2002esevm-motor-control-driver-demo-app
- Clone this repository to get the example projects:
- Change directory to cloned project folder:<br>
    cd *dm-frdmhb2002esevm-motor-control-driver-demo-app*
 
**Note:** If you are using Windows to clone the project, then please configure filename length limit using below command
**git config --system core.longpaths true**
 
### 3.3 Step 3: Build example projects
- Open MCUXpresso IDE and select a directory to create your workspace.
- Install MCXUpresso SDK 2.14.x for FRDM-MCX947, FRDM-MCXA153 (drag and drop SDK zip into "Installed SDK" view) into MCUXpresso IDE.
- Go to "Quickstart Panel" and click on "Import Project(s) from file system",
- Select "Project directory (unpacked)" and browse to the cloned project folder.
- Select example projects that you want to open and run.
- Right click on project and select build to start building the project.

## 4. Test Application Steps<a name="step4"></a>
- User need to check COM port after connecting USB cable between Host PC and Target Board via device manager.

[<img src="./images/device_manager.PNG" width="400"/>](device_manager.png)

- Open PUTTY/Teraterm application installed on your Windows PC with Baudrate 115200 and assigned COM port as mentioned in above step.

[<img src="./images/Putty_Serial_Terminal.PNG" width="400"/>](device_manager.png)

- After right click on project and select "Debug As", Demo application will run in interactive mode. When the demo runs successfully, you can see the logs printed on the terminal.

Note: By default SPI Controller operates in interrupt mode, to switch into EDMA mode Change:

RTE_SPI1_DMA_EN to 1 under board/RTE_Device.h for SPI

## 4.1 Logs Results<a name="step4"></a>

**Main Menu will look like this**

[<img src="./images/LOG1.PNG" width="400"/>](LOG1.png)

**Press #1 to Set PWM**

- Set PWM have two options:
   - Press #1 to  Setup PWM channel from scratch

    [<img src="./images/LOG2.PNG" width="400"/>](LOG2.png)

   - Press #2 to Change duty cycle

    [<img src="./images/LOG3.PNG" width="400"/>](LOG2.png)

**Press #2 to Stop PWM**

- By pressing option 2, user can stop PWM, after stopping PWM, motor stops.

**Press #3 to Get Device ID**

- By pressing option 3, user can get device ID.

    [<img src="./images/LOG4.PNG" width="400"/>](LOG4.png)

**Press #4 for FS_B Control**

- FS_B Control have four options:
   - Press #1 to Enable FS_B (FS_B to become active when fault is active)

    [<img src="./images/LOG5.PNG" width="400"/>](LOG5.png)

    Enable FS_B have thirteen options:

    [<img src="./images/LOG9.PNG" width="400"/>](LOG9.png)

    User can choose any of these available option to FS_B pin follow the choosen faults
    status and option 13 is used to exit from  Enable FS_B menu.

   - Press #2 to Disable FS_B

    [<img src="./images/LOG6.PNG" width="400"/>](LOG6.png)
    
    Disable FS_B have thirteen options:

    [<img src="./images/LOG9.PNG" width="400"/>](LOG9.png)

    User can choose any of these available option to FS_B pin not follow the choosen faults
    status and option 13 is used to exit from Disable FS_B menu.

    - Press #3 to Read FS_B pin

    [<img src="./images/LOG7.PNG" width="400"/>](LOG7.png)

    By pressing option 3, user can read FS_B pin status.

    [<img src="./images/LOG10.PNG" width="400"/>](LOG10.png)

    - Press #4 to Exit from FS_B Control menu

    [<img src="./images/LOG8.PNG" width="400"/>](LOG8.png)

     By pressing option 4, user can exit from FS_B Control menu.

**Press #5 to Check faults status**

- By pressing option 5, user can check available faults.

    [<img src="./images/LOG11.PNG" width="400"/>](LOG11.png)

**Press #6 to Clear faults status**

- Clear faults status have thirteen options:

    [<img src="./images/LOG12.PNG" width="400"/>](LOG12.png)

    User can choose any of these available option to clear choosen faults
    status and option 13 is used to exit from Clear faults status menu.

**Press #7 Over Voltage Protection Control**

- Over Voltage Protection Control have two options:

    - Press #1 to Enable Overvoltage protection

    [<img src="./images/LOG13.PNG" width="400"/>](LOG13.png)

   - Press #2 to Disable Overvoltage protection

    [<img src="./images/LOG14.PNG" width="400"/>](LOG14.png)

    After Disabling overvoltage protection, hb2002 will genarate OV (over voltage) warning only. 

**Press #8 Motor control using SPI**

- Motor control using SPI have seven options:

     [<img src="./images/LOG15.PNG" width="400"/>](LOG15.png)

    - Press #1 to Enable motor control through SPI
         
         By pressing option 1, user can enable motor control through SPI.   

    - Press #2 to Disable Overvoltage protection

         By pressing option 2, user can disable motor control through SPI. 

    - Press #3 to Set virtual Input one (VIN1)

        By pressing option 3, user can set virtual Input one through SPI. 

    - Press #4 to Clear virtual Input one (VIN1)

        By pressing option 4, user can clear virtual Input one through SPI. 
    
    - Press #5 to virtual Input two (VIN2)

        By pressing option 5, user can set virtual Input two through SPI.

    - Press #6 to Clear virtual Input two (VIN2)

        By pressing option 6, user can clear virtual Input two through SPI. 

    - Press #7 to Exit

        By pressing option 7, user can exit from Motor control using SPI menu.

**Press #9 Active Current Limit Control**

- Active Current Limit Control using SPI have three options:

     [<img src="./images/LOG16.PNG" width="400"/>](LOG16.png)

    - Press #1 to Enable Active Current Limit
         
         By pressing option 1, user can enable Active Current LimitI.   

    - Press #2 to Disable Active Current Limit

         By pressing option 2, user can disable Active Current Limit. 

    - Press #3 to Set Active Current Limit

        By pressing option 3, user can set Active Current Limit ( 0 to 3). 

        [<img src="./images/LOG17.PNG" width="400"/>](LOG17.png)

**Press #10 Input mode Control**

- Input mode Control have Two options:

     [<img src="./images/LOG18.PNG" width="400"/>](LOG18.png)

     - Press #1 for H-bridge control mode
         
         By pressing option 1, user can enable H-bridge control mode.   

    - Press #2 for Half-bridge control mode

         By pressing option 2, user can enable  Half-bridge control mode. 

**Press #11 Set SlewRate for SPI**
    
[<img src="./images/LOG19.PNG" width="400"/>](LOG19.png)

By pressing option 11, user can set SlewRate value ( 0 to 7). 

**Press #12 Output Enable Control**
   
- Output Enable Control have four options:

    [<img src="./images/LOG20.PNG" width="400"/>](LOG20.png)

    - Press #1 Enable Output using SPI
         
        By pressing option 1, user can set ENBL pin high using SPI.   

    - Press #2 Disable Output using SPI

         By pressing option 2, user can set ENBL pin low using SPI.

    - Press #3 Enable Output using GPIO
         
         By pressing option 1, user can set ENBL pin high using GPIO.   

    - Press #4 Disable Output using GPIO [Sleep mode]

         By pressing option 2, user can set ENBL pin low using GPIO.

**Press #13 Disable Control**
   
- Disable Control have two options:

    [<img src="./images/LOG21.PNG" width="400"/>](LOG21.png)

    - Press #1 Set Disable pin High [Standby mode]
         
        By pressing option 1, user can set DIS pin high.   

    - Press #2 Set Disable pin Low

         By pressing option 2, user can set DIS pin low.

**Press #14 Read feedback current**
    
[<img src="./images/LOG22.PNG" width="400"/>](LOG22.png)

By pressing option 14, user can read ADC count value for feesback current.

**Press #15 Reset using DIS/ENB**
   
- Reset using DIS/ENB have two options:

    [<img src="./images/LOG23.PNG" width="400"/>](LOG23.png)

    - Press #1 Reset using ENBL pin
         
        By pressing option 1, user can Reset (short-circuit or overtemperature condition) by toggling of ENBL pin.   

    - Press #2 Reset using DIS pin

         By pressing option 2, user can Reset (short-circuit or overtemperature condition) by toggling of DIS pin.

**Press #16 Thermal Management Control**
   
- Thermal Management Control have two options:

    [<img src="./images/LOG24.PNG" width="400"/>](LOG24.png)

    - Press #1 Enable Thermal Management
         
        By pressing option 1, user can Enable Thermal Management (ILIM derate to ILIM/2 on OT warning).   

    - Press #2 Disable Thermal Management

         By pressing option 2, user can Disable Thermal Management (ILIM derate to ILIM/2 from beginning).

**Press #17 Check for open load control**
   
- Check for open load control have two options:

    [<img src="./images/LOG25.PNG" width="400"/>](LOG25.png)

    - Press #1 Enable Check for open load
         
        By pressing option 1, user can Enable Check for open load on transition from Standby to Normal mode.   

    - Press #2 Disable Check for open load

         By pressing option 2, user can Disable Check for open load on transition from Standby to Normal mode.

## 5. Know Limitations

 - we need to made some manual connection on HB2002 board using jumper wire.

    - For FRDMMCXN947

    [<img src="./images/LOG27.PNG" width="400"/>](LOG27.png)

    - For FRDMMCXA153

    [<img src="./images/LOG26.PNG" width="400"/>](LOG26.png)

## 6. FAQs<a name="step5"></a>
*"No FAQs have been identified for this project".*

## 7. Support<a name="step6"></a>

#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-FRDM&ndash;MCXA153-blue)](https://github.com/search?q=org%3Anxp-appcodehub+FRDM-MCXA153+in%3Areadme&type=Repositories) [![Board badge](https://img.shields.io/badge/Board-FRDM&ndash;MCXN947-blue)](https://github.com/search?q=org%3Anxp-appcodehub+FRDM-MCXN947+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-MOTOR%20CONTROL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+ui+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-SPI-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+spi+in%3Areadme&type=Repositories)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+adc+in%3Areadme&type=Repositories)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+gpio+in%3Areadme&type=Repositories)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+pwm+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mcux+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Note**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 8. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release of FRDM-HB2002ESEVM with FRDM-MCXN947 AND MXCA153 on Application Code Hub        | 30 Aug<sup>th</sup> 2024 |

