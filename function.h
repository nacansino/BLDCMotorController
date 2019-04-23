#ifndef XC_FUNCTION_H
#define	XC_FUNCTION_H

#include <xc.h> // include processor files - each processor file is guarded.  

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

void SampleWait(void);
void ApplyGateSignal(void);
void PWM_setDuty_CenterAligned(long double,long double,long double);
void PWM_setDuty_EdgeAligned(long double);
void ISR_enable(void);
void ISR_disable(void);
void PWM_EnableModule(int);


void SampleWait(void)
{
	long i;
	for (i = 0;i < 512000;i++)
		;
	return;
}

void TurnOverDeadTime(void)
{//approximately 5% of period
	long i;
	for (i = 0;i < 152;i++)
		;
	return;
}

/*Apply gate signal code
 * This code applies a gate signal whenever it needs to change
 * 1. Due to a met high throttle threshold (switch in the LOW signals stage)
 * 2. Due to commutation
 *  */
void ApplyGateSignal(){
    int hallstate = (HallA << 2) | (HallB << 1) | HallC;
    //(low side high)
    if(pedalstate==0){//start-up or idle
        if(throttle>HIGHTHROTTLE_THRESHOLD){//start-up
            //apply commutation signals to enter motoring mode
            IOCON1bits.OVRENL = !((maskcodes_FW[hallstate-1] & 0b10000) >> 4);
            IOCON2bits.OVRENL = !((maskcodes_FW[hallstate-1] & 0b100) >> 2);
            IOCON4bits.OVRENL = !(maskcodes_FW[hallstate-1] & 0b1);
            //Override to states that needs to be shut down (commutation process)
            IOCON1bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b100000) >> 5);
            IOCON2bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b1000) >> 3);
            IOCON4bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b10) >> 1);
            pedalstate=1;   //change pedal state to motoring mode
        }else{//idle
            //apply low to all
            IOCON1bits.OVRENL = 1;
            IOCON2bits.OVRENL = 1;
            IOCON4bits.OVRENL = 1;
            IOCON1bits.OVRENH=1;
            IOCON2bits.OVRENH=1;
            IOCON4bits.OVRENH=1;
        }
    }else{//pedalstate==1, continue motoring or turn off
        if(throttle>HIGHTHROTTLE_THRESHOLD_LTP){//continue motoring
            //apply commutation signals to continue motoring mode
            IOCON1bits.OVRENL = !((maskcodes_FW[hallstate-1] & 0b10000) >> 4);
            IOCON2bits.OVRENL = !((maskcodes_FW[hallstate-1] & 0b100) >> 2);
            IOCON4bits.OVRENL = !(maskcodes_FW[hallstate-1] & 0b1);
            //Override to states that needs to be shut down (commutation process)
            IOCON1bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b100000) >> 5);
            IOCON2bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b1000) >> 3);
            IOCON4bits.OVRENH=!((maskcodes_FW[hallstate-1] & 0b10) >> 1);
        }else{//turn-off, apply to all
            pedalstate=0;   //change pedal state to stop
            IOCON1bits.OVRENL = 1;
            IOCON2bits.OVRENL = 1;
            IOCON4bits.OVRENL = 1;
            IOCON1bits.OVRENH=1;
            IOCON2bits.OVRENH=1;
            IOCON4bits.OVRENH=1;
        }
    }
}

void DoStartupBlink(){
    STATUSLED=0;   //turn off status led; 1 means off
    SampleWait();
    STATUSLED=1;   //turn off status led; 1 means off
    SampleWait();
    STATUSLED=0;   //turn off status led; 1 means off
    SampleWait();
    STATUSLED=1;   //turn off status led; 1 means off
}

void PWM_setDuty_CenterAligned(long double dutyA,long double dutyB,long double dutyC){
    /* For Center-aligned Mode
     * Set Duty Cycles using PDCx
    PDCx = (Fosc / (Fpwm*PWM input clock prescaler)) * Desired Duty Cycle 
    */
    PDC1=(int)(dutyA*(FOSC/(FPWM*2*100)));
    PDC2=(int)(dutyB*(FOSC/(FPWM*2*100)));
    PDC4=(int)(dutyC*(FOSC/(FPWM*2*100))); 
}

void PWM_setDuty_EdgeAligned(long double duty){
    /* For Edge-Aligned mode, MASTER DUTY (6-step control)
     Set Duty Cycle using formula:
        MDC/PDCx = (Fosc / (Fpwm*PWM input clock prescaler)) * Desired Duty Cycle 
    */
    MDC=(int)(duty*PERIOD_CTS/100);
}



void ISR_enable(void)
{
/* Enable level 1-7 interrupts */
/* No restoring of previous CPU IPL state performed here */
INTCON2bits.GIE = 1;
return;
}

void ISR_disable(void)
{
/* Disable level 1-7 interrupts */
/* No saving of current CPU IPL setting performed here */
INTCON2bits.GIE = 0;
return;
}

void PWM_EnableModule(int enable){
     /* Enable PWM Module */
    if (enable){PTCON = 0x8000;} else {PTCON = 0x0000;}
}    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

