#ifndef XC_INIT_H
#define	XC_INIT_H

#include <xc.h> // include processor files - each processor file is guarded.  
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

void init_ports(){
    
    zerospeed_flag=0;   //initialize zero speed flag
    HPLockout = 1;
    pedalstate = 0;
    
    PID_IA_err_acc = 0;
    OCP_flag = 0;   //initialize the OCP flag
    
    //initialize PWM isolator to disable signals to gate driver 
    TRISEbits.TRISE5=0; //initialize disable bit as output
    PWM_DISABLE=1;
    
    //adc interrupt indicator bit
    TRISBbits.TRISB15 = 0;  //configure as output
    LATBbits.LATB15 = 0;   //initialize to low
    
    TRISDbits.TRISD10 = 1; //configure OVP pin as input
    TRISDbits.TRISD9 = 1;  //configure UVP pin as input
    
    /*configure status bit as output*/
    TRISDbits.TRISD2=0;
    return;
}

void init_OCPfault(){
/*
    __builtin_write_OSCCONL(OSCCON & (~(1<<6))); // Unlock Register
    RPINR12bits.FLT1R = 1;// Assign FLT1 to Pin C1OUT
    __builtin_write_OSCCONL(OSCCON | (1<<6));// Lock Registers
*/
    
}

void init_Hall(){
    /*configure pins for HALL*/
    /*initialize input capture for hall signals*/
    TRISFbits.TRISF6=1; //Configure Hall A as Input
    TRISFbits.TRISF2=1; //Configure Hall B as Input
    TRISFbits.TRISF3=1; //Configure Hall C as Input
    
    RPINR7bits.IC1R = 102;        // HallA at RP102
    RPINR7bits.IC2R = 98;        // HallB at RP98
    RPINR10bits.IC7R = 99;       // HallC at RP99

    IC1CON1bits.ICTSEL = 0b100;       // select Timer1
    IC2CON1bits.ICTSEL = 0b100;
    IC7CON1bits.ICTSEL = 0b100;

    IC1CON2bits.SYNCSEL = 0b01011;       // select Timer1
    IC2CON2bits.SYNCSEL = 0b01011;
    IC7CON2bits.SYNCSEL = 0b01011;
    
    IC1CON1bits.ICI = 0;         // interrupt on every capture event
    IC2CON1bits.ICI = 0;
    IC7CON1bits.ICI = 0;

    IC1CON1bits.ICM = 1;         // capture mode, every edge
    IC2CON1bits.ICM = 1;
    IC7CON1bits.ICM = 1;

    IPC0bits.IC1IP = PRIORITY_HALL;       // interrupt priority
    IPC1bits.IC2IP = PRIORITY_HALL;
    IPC5bits.IC7IP = PRIORITY_HALL;
	
    IFS0bits.IC1IF = 0;                 // clear interrupt flag
    IFS0bits.IC2IF = 0;
    IFS1bits.IC7IF = 0;
	
    IEC0bits.IC1IE = 1;                 // enable IC interrupt
    IEC0bits.IC2IE = 1;
    IEC1bits.IC7IE = 1;
    
    /*CN Interrupt config bits
    IPC4bits.CNIP=PRIORITY_HALL; //Set CNinterrupt priority
    IEC1bits.CNIE=1;        // Enable CN interrupt
    IFS1bits.CNIF=0;      // Reset CN interrupt
    */
    return;
}

void init_Timer1(){
    /*this timer is used to measure the speed of the motor*/
    int i;
    for(i=0;i<SPEED_FILTER_SAMPLE;i++){
        speed_hyst[i]=0;
    }
    tmr1_ctr=0;                     //initialize timer counter (inline postscaler)
    tmr1_acc=0;                     //initialize accumulator timer
    init_time=0;                    //initialize init_time to zero
    T1CONbits.TON = 0;              // Disable Timer
    T1CONbits.TCS = 0;              // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;            // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11;        // Select 1:256 Prescaler
    TMR1 = 0x00;                    // Clear timer register
    PR1 = 46875;                   // Load the period value (16-bit max) (200ms)
    IPC0bits.T1IP = PRIORITY_TIMER; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;              // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;              // enable Timer1 interrupt
    T1CONbits.TON = 1;              // Start Timer
    
}

void init_isr(){
    /* Interrupt nesting enabled here */
    INTCON1bits.NSTDIS = 0;
    
    //CNENAbits.CNIEA15 = 1;
    
    /* Set Change Notice interrupt priority to user-defined rank */
    IPC4bits.CNIP = PRIORITY_OVPUVP;
    /* Enable CN interrupts */
    IEC1bits.CNIE = 1;
    IFS1bits.CNIF = 0;
    
    
    return;
}
    
// Fcy = Fosc/2 = (1/2)*Fin*(M/(N1 * N2);
// N1 = PLLPRE + 2
// N2 = 2 x (PLLPOST + 1)
// M = PLLDIV + 2
void init_oscillator(){
//    OSCTUNbits.TUN = 0b010010;
    PLLFBD = 63;                        // M = 65
    CLKDIVbits.PLLPOST = 0;             // N2 = 2
    CLKDIVbits.PLLPRE = 0;              // N1 = 3

    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);      // select new clock
    __builtin_write_OSCCONL(OSCCON | 0x01);      // request clock switching
    
    while(OSCCONbits.COSC != 0b001);    // wait for the clock to switch
    while(OSCCONbits.LOCK != 1);        // wait for the PLL to lock
    return;
}

//Use PWM 1,2,4
//for center-aligned PWM
void init_PWM_CenterAligned(){
    /* Set Independent Time Bases, Center-Aligned mode and
     Independent Duty Cycles */
    PWMCON1 = PWMCON2 = PWMCON4 = 0x0204; //bit 9 (ITB), 2 (CAM) asserted
     
    /*configure PWM_EN pin as output*/
    TRISEbits.TRISE5=0;
    
    /* Set Dead Time Values */
    /* DTRx Registers are ignored in Center-Aligned Mode */
    /*Calculation:
     ALTDTR/DTR = Fosc * Desired Dead Time / PWM input clock prescaler
     */
    DTR1 = DTR2 = DTR4 = 0;
    ALTDTR1 = ALTDTR2 = ALTDTR4 = 25;
    /* Set PWM Mode to Complementary */
    IOCON1 = IOCON2 = IOCON4 = 0xC000;
    
    /* Configure Faults */
    FCLCON1 = FCLCON2 = FCLCON4 = 0x0003;
    /* 1:1 Prescaler */
    PTCON2 = 0x0000;
    
    /* Set PWM Periods on PHASEx Registers (Center-aligned mode)
    Aim for 32KHz, 0x09c3=2499 in decimal
    PTPER,PHASEx,SPHASEx = FOSC/ (Fpwm * PWM Input Clock Prescaler * 2) 
    */
    PHASE1 = PHASE2 = PHASE4 = PERIOD_CTS/2;    
   
    TRIG1 = FOSC/(FPWM*2); /* Point at which the ADC module is to be
    triggered by primary PWM */
    TRGCON1bits.TRGDIV = 0; /* Trigger output divider set to trigger ADC on
    every trigger match event */
    TRGCON1bits.TRGSTRT = 4; /* First ADC trigger event occurs after four
    trigger match events */
    PWMCON1bits.TRGIEN = 1; /* Trigger event generates an interrupt request */
    while (PWMCON1bits.TRGSTAT == 1);  /* Wait for ADC interrupt status change */
    return;
}

//Use PWM 1,2,4
//for edge-aligned PWM; 6stepcontrol application
void init_PWM_EdgeAligned(){
    /* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
    PWMCON1 = PWMCON2 = PWMCON4 = 0x0000;
    
    /* Set PWM Period on Primary Time Base 
     PTPER=FOSC/(FPWM* PWM Input Clock Prescaler)
     */
    PTPER = PERIOD_CTS/1;
    /* Set Phase Shift */
    PHASE1 = SPHASE1 = PHASE2 = SPHASE2 = PHASE4 = SPHASE4 = 0; //no phase shift
    
    /* Set Dead Time Values */
    DTR1 = DTR2 = DTR4 = 25;
    ALTDTR1 = ALTDTR2 = ALTDTR4 = 0;

    /* Set ownership (pwm ownership), Output Pin Polarity (active-high), Pin Mode (independent)*/
    IOCON1bits.PENH=1;
    IOCON1bits.PENL=1;  //PWM control
    IOCON1bits.POLH=0;
    IOCON1bits.POLL=0;
    IOCON2bits.PENH=1;
    IOCON2bits.PENL=1;  //PWM CONTROL
    IOCON2bits.POLH=0;
    IOCON2bits.POLL=0;
    IOCON4bits.PENH=1;
    IOCON4bits.PENL=1;  //PWM CONTROL
    IOCON4bits.POLH=0;
    IOCON4bits.POLL=0;
    IOCON1bits.PMOD=0b11;
    IOCON2bits.PMOD=0b11;
    IOCON4bits.PMOD=0b11;
    //set trisbits of pwm
    /*configure override for PWM low signals as output (6-STEP CONTROL)*/
    //TRISEbits.TRISE0=0;
    //TRISEbits.TRISE2=0;
    //TRISEbits.TRISE6=0;
    /*Initialize as low*/
    //LA=0;
    //LB=0;
    //LC=0;
        
    /*Set PWM of LOW switches to 100% (good as normally high)*/
    SDC1 = SDC2 = SDC4 = 2*PERIOD_CTS;
        
    /* Configure Fault  */
    FCLCON1bits.IFLTMOD = 0; /* CLDAT bits control PWMxH and FLTDAT bits control PWMxL */
    FCLCON1bits.CLSRC = 8;  /* Current-limit input source is Analog Comparator 1 */
    FCLCON1bits.FLTSRC = 9; /* Fault input source is Analog Comparator 2 */
    FCLCON1bits.CLPOL = 1;  /* Current-limit source is active-low */
    FCLCON1bits.FLTPOL = 1; /* Fault Input source is active-low */
    FCLCON1bits.CLMOD = 1; /* Enable current-limit function */
    FCLCON1bits.FLTMOD = 1; /* Enable Cycle-by-Cycle Fault mode */
    IOCON1bits.FLTDAT = 0;  /* PWMxH and PWMxL are driven inactive on occurrence of fault */
    IOCON1bits.CLDAT = 0; /* PWMxH and PWMxL are driven inactive on occurrence of current-limit */
    
    FCLCON2bits.IFLTMOD = 0; /* CLDAT bits control PWMxH and FLTDAT bits control PWMxL */
    FCLCON2bits.CLSRC = 8;  /* Current-limit input source is Analog Comparator 1 */
    FCLCON2bits.FLTSRC = 9; /* Fault input source is Analog Comparator 2 */
    FCLCON2bits.CLPOL = 1;  /* Current-limit source is active-low */
    FCLCON2bits.FLTPOL = 1; /* Fault Input source is active-low */
    FCLCON2bits.CLMOD = 1; /* Enable current-limit function */
    FCLCON2bits.FLTMOD = 1; /* Enable Cycle-by-Cycle Fault mode */
    IOCON2bits.FLTDAT = 0;  /* PWMxH and PWMxL are driven inactive on occurrence of fault */
    IOCON2bits.CLDAT = 0; /* PWMxH and PWMxL are driven inactive on occurrence of current-limit */
    
    FCLCON4bits.IFLTMOD = 0; /* CLDAT bits control PWMxH and FLTDAT bits control PWMxL */
    FCLCON4bits.CLSRC = 8;  /* Current-limit input source is Analog Comparator 1 */
    FCLCON4bits.FLTSRC = 9; /* Fault input source is Analog Comparator 2 */
    FCLCON4bits.CLPOL = 1;  /* Current-limit source is active-low */
    FCLCON4bits.FLTPOL = 1; /* Fault Input source is active-low */
    FCLCON4bits.CLMOD = 1; /* Enable current-limit function */
    FCLCON4bits.FLTMOD = 1; /* Enable Cycle-by-Cycle Fault mode */
    IOCON4bits.FLTDAT = 0;  /* PWMxH and PWMxL are driven inactive on occurrence of fault */
    IOCON4bits.CLDAT = 0; /* PWMxH and PWMxL are driven inactive on occurrence of current-limit */
        
    
    /* 1:1 Prescaler */
    PTCON2 = 0x0000;

    /*set override value of HPWM to low (since the ON PHASE contains the PWM only, others are off)*/
    IOCON1bits.OVRDAT1=0;
    IOCON2bits.OVRDAT1=0;
    IOCON4bits.OVRDAT1=0;
    /*set override value of LPWM to low (since the ON PHASE contains the PWM only, others are off)*/
    IOCON1bits.OVRDAT0=0;
    IOCON2bits.OVRDAT0=0;
    IOCON4bits.OVRDAT0=0;
    
    /*enable override of HPWM to low for now; disable override when PWM is enabled*/
    IOCON1bits.OVRENH=1;
    IOCON2bits.OVRENH=1;
    IOCON4bits.OVRENH=1;
    /*enable override of LPWM to low for now; disable override when PWM is enabled*/
    IOCON1bits.OVRENL=1;
    IOCON2bits.OVRENL=1;
    IOCON4bits.OVRENL=1;
    /*set override of PWM to on by default*/
    
    TRIG1 = FOSC/(FPWM*2);
    TRIG2 = FOSC/(FPWM*2);
    TRIG4 = FOSC/(FPWM*2);/* Point at which the ADC module is to be
    triggered by primary PWM */
    TRGCON1bits.TRGDIV = 0;
    TRGCON2bits.TRGDIV = 0;
    TRGCON4bits.TRGDIV = 0; /* Trigger output divider set to trigger ADC on 
    every trigger match event */
    TRGCON1bits.TRGSTRT = 1;
    TRGCON2bits.TRGSTRT = 1;
    TRGCON4bits.TRGSTRT = 1; /* First ADC trigger event occurs after four
    trigger match events */
    PWMCON1bits.TRGIEN = 1; /* Trigger event generates an interrupt request */
    while (PWMCON1bits.TRGSTAT == 1);  /* Wait for ADC interrupt status change */
    PWMCON2bits.TRGIEN = 1;
    while (PWMCON2bits.TRGSTAT == 1);
    PWMCON3bits.TRGIEN = 1;
    while (PWMCON4bits.TRGSTAT == 1);
    return;
}

//for edge-aligned PWM; 6stepcontrol application
void init_IsensePWM(){
    /* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
    PWMCON3 = 0x0000;
    /* Set PWM Period on Primary Time Base */
    PTPER = PERIOD_CTS;
    
    /* Set PWM Mode to Independent, GPIO control on high side signal */
    IOCON3bits.PENH=0;
    IOCON3bits.PENL=1;
    IOCON3bits.PMOD=0b11;
    
    /* Set Phase Shift */
    SPHASE3 = 0; //no phase shift
    
    return;
}

//for peak current limiting functionality
void init_Comparator(){
 
    //Comparator output is present on the CxOUT pin
    CM1CONbits.COE = 1;
    CM2CONbits.COE = 1;
    
    //Comparator output is not inverted
    CM1CONbits.CPOL = 0;
    CM2CONbits.CPOL = 0;
       
    //Trigger/event/interrupt is generated on any change of the comparator output (while CEVT = 0)
    CM1CONbits.EVPOL = 0b11;
    CM2CONbits.EVPOL = 0b11;
    /*
    //VIN+ input connects to CxIN1+ pin
    CM1CONbits.CREF = 0;
    */
    
    //VIN+ input connects to internal CVREFIN voltage (that is connected to pin VREF+)
    CM1CONbits.CREF = 1;
    CM2CONbits.CREF = 1;
    
    //Comparator Channel Select bits
    CM1CONbits.CCH = 0b00;  //assign CMP1 inverting input to C1IN2- 
    CM2CONbits.CCH = 0b01;  //assign CMP2 inverting input to C2IN1-
    
    //Comparator Voltage Reference Source Selection bit
    CVRCONbits.CVRSS = 0b0; //Comparator voltage reference source, CVRSRC = AVDD ? AVSS; This pin is ignored (check circuit) when VREFSEL=1

    //Set trip point at 50A ~ Vsense near 2.5V (provision, trip point is set by an external source)
    //CVRCONbits.VREFSEL = 0b0;    //CVREFIN is generated by the resistor network
    CVRCONbits.VREFSEL = 0b1;   //CVREFIN is VREF+
    CVRCONbits.CVREN = 0b0;    //Comparator voltage reference circuit powered off
    CVRCONbits.CVROE = 0b0;    //Voltage level is disconnected from CVREF pin
    CVRCONbits.CVRR = 0b0;  //CVRSRC/32 step-size
    CVRCONbits.CVR = 0b111; //CVrefin = 3.3((8+2^(1+1+1))/32)
    
    //output of comparator
    RPOR14bits.RP120R=0b011000;
    RPOR13bits.RP118R=0b011001;

    //Comparator event bit
    CM1CONbits.CEVT = 0; // clear comparator event bit
    CM2CONbits.CEVT = 0; // clear comparator event bit
    
    //configure interrupt bits
    IPC4bits.CMIP=PRIORITY_OCP;
    IEC1bits.CMIE=1;    //enable combined comparator interrupt
    
    //Comparator Enable Pin
    CM1CONbits.CON = 1;  //Comparator is enabled 
    CM2CONbits.CON = 1;  //Comparator is enabled
    
}

void init_i2c(){
    /*set pins as input*/
    TRISGbits.TRISG2 = 1;
    TRISGbits.TRISG3 = 1;

//    I2C1BRG = 0x188;        // Fcy=60Mhz, Fscl=100kHz
    /*Baud Rate Generator
     I2CxBRG=((1/FSCL-delay)xFCY/2)-2
     */
    I2C1BRG = 0x20;         // Fcy=60MHz, Fscl=1MHz
    I2C1ADD = 0x00;
    I2C1CON = 0x1200;
    I2C1RCV = 0x0000;
    I2C1TRN = 0x0000;
    I2C1CONbits.I2CEN = 1;  // enable the I2C(1) module. configures SDA1 and SCL1 as serial port pins.
    return;
}

void init_ADC(){
    /*Set analog inputs*/
    ANSELBbits.ANSB1 = 1;
    ANSELBbits.ANSB2 = 1;         
    ANSELBbits.ANSB10 = 1;
    ANSELBbits.ANSB11 = 1;
    
    /*configure pins for ADC*/
    TRISBbits.TRISB10 = 1;  //Configure AN10 as input
    TRISBbits.TRISB0 = 1;   //Configure AN0 as input
    TRISBbits.TRISB1 = 1;   //Configure AN1 as input
    TRISBbits.TRISB2 = 1;   //Configure AN2 as input
    
    AD1CON1bits.AD12B = 0;      //10-bit, 4 channel mode
    AD1CON2bits.VCFG2 = 1;      //VREFH=AVDD, VREFL=AVSS
    AD1CON3bits.ADCS = 2;       //TAD=3*Tcy (FAD=Fcy/3)
    AD1CON3bits.ADRC=0;         //Clock derived from system clock
    
    AD1CHS0bits.CH0NA = 0;      //CH0 (-) input is VREFL, for sample A
    AD1CHS0bits.CH0NB = 0;      //CH0 (-) input is VREFL, for sample B (Unused)
    AD1CHS0bits.CH0SA = 10;     //CH0 (+) input is AN10, for sample A
    AD1CHS0bits.CH0SB = 0;      //CH0 (+) input is AN0, for sample B (Unused)
    
    AD1CHS123bits.CH123NA = 0;  //CH1, CH2, CH3 (-) input is VREFL, for sample A
    AD1CHS123bits.CH123NB = 0;  //CH1, CH2, CH3 (-) input is VREFL, for sample B (Unused)
    AD1CHS123bits.CH123SA = 0;  //CH1 (+) input is AN0, CH2 (+) input is AN1, CH3 (+) input is AN2
    AD1CHS123bits.CH123SB = 0;  //CH1 (+) input is AN3, CH2 (+) input is AN4, CH3 (+) input is AN5 (Unused)
    
    AD1CON2bits.CHPS = 0b10;    //Converts CH0, CH1, CH2 and CH3
    
    AD1CON1bits.SIMSAM = 1;     //Samples CH0, CH1, CH2, CH3 simultaneously
    AD1CON2bits.CSCNA = 0;      //Do not scan inputs
    AD1CON2bits.ALTS = 0;       //Always uses channel input selects for Sample A
    AD1CON1bits.ASAM = 1;       //Sampling begins immediately after the last conversion; SAMP bit is auto-set

    AD1CON1bits.SSRCG = 1;      
    AD1CON1bits.SSRC = 1;       //PWM Generator 1 primary trigger compare ends sampling and starts conversion
    
    AD1CON2bits.SMPI = 0;       // Generates interrupt after completion of every sample/conversion operation
    AD1CON2bits.BUFM = 0;       // Buffer Fill Mode Select bit (Always starts filling the buffer from the Start address)
    AD1CON1bits.FORM = 0b00;    //Unsigned Integer

    IEC0bits.AD1IE = 1;     // Enable A/D interrupt
    IPC3bits.AD1IP = 5;     // Set Priority
    IFS0bits.AD1IF = 0;     // Clear the A/D interrupt flag bit
    AD1CON1bits.ADON = 1;         // turn on ADC
    return;
}    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

