/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC18F25K80
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"

#define BNO055_Write_Address 0x50
#define BNO055_Read_Address 0x51

void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(uint8_t d);
void I2C_Master_Read(unsigned short a, uint8_t *data);
void BNO055Initialize();

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
    // set CAN bus control register
    CIOCONbits.CANCAP = 1;  // enable CAN capture
    CIOCONbits.CLKSEL = 1;  // CAN clk select
    CIOCONbits.ENDRHI = 1;  // enable drive high (CANTX drives VDD when recessive)
    
    uint8_t BNO055_address;
    BNO055Initialize();
    uCAN_MSG BNO055_data;
    uint8_t linear_accel_x_MSB = 0x05, linear_accel_x_LSB = 0x55, linear_accel_y_MSB = 0x05,
            linear_accel_y_LSB = 0x55, linear_accel_z_MSB = 0x05, linear_accel_z_LSB = 0x55;
    uint8_t *data;
    uint8_t *writeBuffer;
    //
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1)
    {
        // Add your application code
        if(TMR2_GetTransmit){
            /** G sensor */
            //read linear acceleration data for 3 axis
            I2C_Master_Start();
            I2C_Master_Write(BNO055_Write_Address);
            I2C_Master_Write(BNO055_OPR_MODE_ADDR);
            I2C_Master_Write(OPERATION_MODE_ACCONLY);
            I2C_Master_Stop();

            I2C_Master_Start();
            I2C_Master_Write(BNO055_Write_Address);
            I2C_Master_Write(BNO055_ACCEL_DATA_X_LSB_ADDR);
            I2C_Master_Stop();

            I2C_Master_Start();         //Start condition
            I2C_Master_Write(BNO055_Read_Address);     //7 bit address + Read
            I2C_Master_Read(6, data); //Read + Acknowledge
            I2C_Master_Stop();          //Stop condition

            if (data[1] <= 0x0F){ //check if the acceleration data are acceptable
              linear_accel_x_LSB = data[0];
              linear_accel_x_MSB = data[1];
            }
            if (data[3] <= 0x0F){
              linear_accel_y_LSB = data[2];
              linear_accel_y_MSB = data[3];
            }
            if (data[5] <= 0x0F){
              linear_accel_z_LSB = data[4];
              linear_accel_z_MSB = data[5];
            }

            BNO055_data.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
            BNO055_data.frame.id = 0x471;
            BNO055_data.frame.dlc = 6;
            BNO055_data.frame.data0 = linear_accel_z_MSB;
            BNO055_data.frame.data1 = linear_accel_z_LSB;
            BNO055_data.frame.data2 = linear_accel_y_MSB;
            BNO055_data.frame.data3 = linear_accel_y_LSB;
            BNO055_data.frame.data4 = linear_accel_x_MSB;
            BNO055_data.frame.data5 = linear_accel_x_LSB;

            CAN_transmit(&BNO055_data);
            
            TMR2_ClearTransmit();
            INTERRUPT_PeripheralInterruptEnable();
        }
    }
}

void I2C_Master_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
  I2C_Master_Wait();
  SEN = 1;
}

void I2C_Master_RepeatedStart()
{
  I2C_Master_Wait();
  RSEN = 1;
}

void I2C_Master_Stop()
{
  I2C_Master_Wait();
  PEN = 1;
}

void I2C_Master_Write(uint8_t d)
{
  I2C_Master_Wait();
  SSPBUF = d;
}

void I2C_Master_Read(unsigned short a, uint8_t *data)
{
  unsigned short temp;
  uint8_t i = 0;
  while (i < a){
    I2C_Master_Wait();
    RCEN = 1;
    I2C_Master_Wait();
    data[i] = SSPBUF;
    i++;
    I2C_Master_Wait();
    ACKDT = (i < a)?0:1;
    ACKEN = 1;
  }
}

void BNO055Initialize()
{
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_OPR_MODE_ADDR);
    I2C_Master_Write(OPERATION_MODE_CONFIG);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_SYS_TRIGGER_ADDR);
    I2C_Master_Write(0x20);
    I2C_Master_Stop();
    
    __delay_ms(20);
    
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_PWR_MODE_ADDR);
    I2C_Master_Write(POWER_MODE_NORMAL);
    I2C_Master_Stop();
    
    __delay_ms(20);
    
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_PAGE_ID_ADDR);
    I2C_Master_Write(0x0);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_UNIT_SEL_ADDR);
    I2C_Master_Write(0x1);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(BNO055_Write_Address);
    I2C_Master_Write(BNO055_SYS_TRIGGER_ADDR);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();
    
    __delay_ms(20);
}
/**
 End of File
*/
