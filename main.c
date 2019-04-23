/*
 * File:   main.c
 * Author: ricky
 *
 * Created on December 2, 2015, 11:05 PM
 */

#include "xc.h"
#include <dsp.h>
#include <p33ep512mc806.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "variables.h"
#include "function.h"
#include "init.h"
#include "i2c.h"
#include "isr.h"

int main(void) {
    //initialize procedures
    init_oscillator();
    init_ports();
    init_Comparator();
    init_OCPfault();
    init_coefficients();
    init_dsp();
    init_PWM_EdgeAligned(); //6step control (use center aligned for FOC)
    init_IsensePWM();
    init_ADC();
    init_Hall();
    init_i2c();
    init_Timer1();
    init_isr();
    
    ISR_enable();
    INTCON2bits.GIE = 1;
    
    DoStartupBlink();
    
    PWM_EnableModule(1);//enable pwm module to start ADC Reading (PWM triggering)
    
    SampleWait();
    
    /*high pedal lockout*/
    while(HPLockout);
    HPLockout=2;    //lift HPLockout status
    i2c_puts("HPLockout Released\n");
    
    PWM_DISABLE=0;
    
    STATUSLED=0;   //status bit as output; this light signifies enabled PWM at the output; 0 means on

    ApplyGateSignal();  //kickstart gate signal
    
    //LATAbits.LATA7=1;
    //MC_SINCOS_T values;
    //int16_t angle;
    //angle=0x2aab;
    //int temp;
//            LATAbits.LATA6=1;    
    //temp=MC_CalculateSineCosine_InlineC_Ram(angle,&values);

    while(1){
        //LATAbits.LATA7=PORTDbits.RD6;
        
        //LATAbits.LATA7^=1;
    }
    return (EXIT_SUCCESS);
}
