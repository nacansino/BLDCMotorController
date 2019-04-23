/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_I2C_H
#define	XC_I2C_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

void i2c_start();
void i2c_restart();
void i2c_stop();
void i2c_write(unsigned char byte);
void i2c_idle();
unsigned int i2c_ackstatus();
void i2c_notack();
void i2c_ack();
unsigned char i2c_getc();

unsigned int i2c_putc(unsigned char data);  // 8-bit ASCII
unsigned int i2c_puts(char *data);
unsigned int i2c_putint(int data);  // 16-bit integer
unsigned int i2c_putl(long data);   // 32-bit integer
unsigned int i2c_putf(double data);  // float


void i2c_start(){               // generates an I2C start condition
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN);     // wait for start condition
}

void i2c_restart(){             // generates an I2C restart condition
    I2C1CONbits.RSEN = 1;
    while(I2C1CONbits.RSEN);    // wait for restart condition
}

void i2c_stop(){                // generates an I2C stop condition
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);     // wait for stop condition
}

void i2c_write(unsigned char byte){
    I2C1TRN = byte;
    while(I2C1STATbits.TBF);
}

void i2c_idle(){
    while(I2C1STATbits.TRSTAT); // wait for bus idle
}

unsigned int i2c_ackstatus(){
    return (!I2C1STATbits.ACKSTAT);
}

void i2c_notack(){
    I2C1CONbits.ACKDT = 1;      // set for NotACK
    I2C1CONbits.ACKEN = 1;
    while(I2C1CONbits.ACKEN);   // wait for ACK to complete
    I2C1CONbits.ACKDT = 0;      // set for NotACK
}

void i2c_ack(){
    I2C1CONbits.ACKDT = 0;      // set for ACK
    I2C1CONbits.ACKEN = 1;
    while(I2C1CONbits.ACKEN);   // wait for ACK to complete
}

unsigned char i2c_getc(){
    I2C1CONbits.RCEN = 1;       // enable master receive
    while(!I2C1STATbits.RBF);   // wait for receive buffer to be full
    return I2C1RCV;             // return data in buffer
}

unsigned int i2c_putc(unsigned char data){
    unsigned int error;
    i2c_idle();
    i2c_start();
    i2c_write(0x44);        // address + write
    i2c_idle();
    error = i2c_ackstatus();
    i2c_write(data);
    i2c_idle();
    error = i2c_ackstatus();
    i2c_stop();
    return error;
}

unsigned int i2c_puts(char *data){
    unsigned int error;
    i2c_idle();
    i2c_start();
    i2c_write(0x44);        // address + write
    i2c_idle();
    error = i2c_ackstatus();

    while(*data){
        i2c_write(*data++);
        i2c_idle();
        error = i2c_ackstatus();
    }
    i2c_stop();
    return error;
}

unsigned int i2c_putint(int data){
    char str[6];
    sprintf(str, "%d", data);
    return i2c_puts(str);
}

unsigned int i2c_putl(long data){
    char str[11];
    sprintf(str, "%ld", data);
    return i2c_puts(str);
}

unsigned int i2c_putf(double data){
    char str[20];
    sprintf(str, "%.2f", data);
    return i2c_puts(str);
}    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

