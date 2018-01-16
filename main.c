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

#define BNO055_MAX_RETRY 50

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
    BNO055_address = BNO055_ADDRESS_B;
    BNO055_Initialize();
    uCAN_MSG BNO055_data;
    uint8_t linear_accel_x_MSB = 0x55, linear_accel_x_LSB = 0x55, linear_accel_y_MSB = 0x55,
            linear_accel_y_LSB = 0x55, linear_accel_z_MSB = 0x55, linear_accel_z_LSB = 0x55;
    uint8_t *data, *buffer;
    data[0] = 0x0;
    data[1] = 0x0;
    data[2] = 0x0;
    data[3] = 0x0;
    data[4] = 0x0;
    data[5] = 0x0;
    uint8_t *writeBuffer;
    I2C_MESSAGE_STATUS flag;
    uint16_t timeOut = 0, counter = 0;
    bool fail = false, complete = false, overflow = false;
//    RC0 = 1;
//    RC1 = 1;
//    RC2 = 1;
    
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
        /** G sensor */
        while (flag != I2C_MESSAGE_FAIL && timeOut < BNO055_MAX_RETRY) {
            while (I2C_MasterQueueIsFull() == true);
            writeBuffer[0] = BNO055_SYS_TRIGGER_ADDR;
            writeBuffer[1] = 0x20;
            flag = I2C_MESSAGE_PENDING;
            I2C_MasterWrite ( writeBuffer, 1, BNO055_address, &flag );
            while ( flag == I2C_MESSAGE_PENDING );
            if (flag == I2C_MESSAGE_COMPLETE)   timeOut = BNO055_MAX_RETRY;
            timeOut ++;
        }
        buffer[0] = data[0];
        timeOut = 0;
        
        while (flag != I2C_MESSAGE_FAIL && timeOut < BNO055_MAX_RETRY) {
            while (I2C_MasterQueueIsFull() == true);
            writeBuffer[0] = BNO055_CHIP_ID_ADDR;
            flag = I2C_MESSAGE_PENDING;
            I2C_MasterWrite ( writeBuffer, 1, BNO055_address, &flag );
            while ( flag == I2C_MESSAGE_PENDING );
            if (flag == I2C_MESSAGE_COMPLETE)   timeOut = BNO055_MAX_RETRY;
            timeOut ++;
        }
        buffer[1] = data[0];
        if (flag == I2C_MESSAGE_COMPLETE) {
            flag = I2C_MESSAGE_PENDING;
            I2C_MasterRead ( data, 1, BNO055_address, &flag );
            while ( flag == I2C_MESSAGE_PENDING );
        }
        timeOut = 0;
        
        //set to 9 degrees of freedom mode
        while (flag != I2C_MESSAGE_FAIL && timeOut < BNO055_MAX_RETRY) {
            while ( I2C_MasterQueueIsEmpty() == false );
            writeBuffer[0] = BNO055_OPR_MODE_ADDR;
            writeBuffer[1] = OPERATION_MODE_NDOF;
            I2C_MasterWrite ( writeBuffer, 2, BNO055_address, &flag );
            while (flag == I2C_MESSAGE_PENDING);
            if (flag == I2C_MESSAGE_COMPLETE) {
                timeOut = BNO055_MAX_RETRY;
            }
            timeOut++;
        }
        timeOut = 0;
        buffer[2] = data[0];
        buffer[3] = data[1];
        buffer[4] = data[2];
        buffer[5] = data[3];
        buffer[6] = data[4];
        buffer[7] = data[5];
        
        //read linear acceleration data for 3 axis
        while (flag != I2C_MESSAGE_FAIL && timeOut < BNO055_MAX_RETRY){
            while ( I2C_MasterQueueIsEmpty() == false );
            writeBuffer[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
            flag = I2C_MESSAGE_PENDING;
            I2C_MasterWrite (writeBuffer, 1, BNO055_address, &flag);
            while (flag == I2C_MESSAGE_PENDING);
            if (flag == I2C_MESSAGE_COMPLETE)   timeOut = BNO055_MAX_RETRY;
            timeOut++;
        }
        timeOut = 0;
        
        while (I2C_MasterQueueIsEmpty() == false);
        flag = I2C_MESSAGE_PENDING;
        I2C_MasterRead (data, 6, BNO055_address, &flag);
        while (flag == I2C_MESSAGE_PENDING);
        
        linear_accel_x_LSB = data[0];
        linear_accel_x_MSB = data[1];
        linear_accel_y_LSB = data[2];
        linear_accel_y_MSB = data[3];
        linear_accel_z_LSB = data[4];
        linear_accel_z_MSB = data[5];
        
        if (counter > 100)       RC0 = 1;    else    RC0 = 0;
        if (complete)   RC1 = 1;    else    RC1 = 0;
        if (data[0] != buffer[2] && data[1] != buffer[3] && data[2] != buffer[4]
                && data[3] != buffer[5] && data[4] != buffer[6] && data[5] != buffer[7]) {
            RC2 = 1;
        }else    RC2 = 0;
        fail = false;
        complete = false;
        overflow = false;
        
        BNO055_data.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
        BNO055_data.frame.id = 0x131;
        BNO055_data.frame.dlc = 6;
        BNO055_data.frame.data0 = linear_accel_x_MSB;
        BNO055_data.frame.data1 = linear_accel_x_LSB;
        BNO055_data.frame.data2 = linear_accel_y_MSB;
        BNO055_data.frame.data3 = linear_accel_y_LSB;
        BNO055_data.frame.data4 = linear_accel_z_MSB;
        BNO055_data.frame.data5 = linear_accel_z_LSB;
        
        CAN_transmit(&BNO055_data);
        counter++;
    }
}
/**
 End of File
*/