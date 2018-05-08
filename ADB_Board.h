//*****************************************************************************
// Board.h
//
// configure the device pins for different signals
//
// Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 02/05/2018 at 17:25:59
// by TI PinMux version 4.0.1511 
//
//*****************************************************************************
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP_EXP432P401R are active high. */
#define Board_GPIO_LED_OFF (0)
#define Board_GPIO_LED_ON  (1)

#define Board_initADC               ADC_init
#define Board_initADCBuf            ADCBuf_init
#define Board_initGeneral           MSP_EXP432P401R_initGeneral
#define Board_initGPIO              GPIO_init
#define Board_initI2C               I2C_init
#define Board_initPWM               PWM_init
#define Board_initSDSPI             SDSPI_init
#define Board_initSPI               SPI_init
#define Board_initUART              UART_init
#define Board_initWatchdog          Watchdog_init

/* Board specific I2C addresses */
#define Board_TMP_ADDR              (0x40)
#define Board_RF430CL330_ADDR       (0x28)
#define Board_TPL0401_ADDR          (0x40)

/*!
 *  @def    MSP_EXP432P401R_GPIOName
 *  @brief  Enum of GPIO names on the MSP_EXP432P401R dev board
 */
typedef enum Board_GPIOName {
    FRAM_CS = 0,
    PQ9_EN = 1,
    DEB_STATUS0 = 2,
    DEB_STATUS1 = 3,
    DEB_STATUS2 = 4,
    DEB_STATUS3 = 5,
    BURN0 = 6,
    BURN1 = 7,
    BURN2 = 8,
    BURN3 = 9,
    EXT_WDG = 10,
    Board_GPIOCOUNT
} Board_GPIOName;

/*!
 *  @def    MSP_EXP432P401R_I2CName
 *  @brief  Enum of I2C names on the MSP_EXP432P401R dev board
 */
typedef enum Board_I2CName {
    I2C_MON = 0,
   Board_I2CCOUNT
} Board_I2CName;

/*!
 *  @def    MSP_EXP432P401R_UARTName
 *  @brief  Enum of UART names on the MSP_EXP432P401R dev board
 */
typedef enum Board_UARTName {
    PQ9 = 0,
    DBG = 1,
    Board_UARTCOUNT
} Board_UARTName;
    

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
extern void Board_initGeneral(void);


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
