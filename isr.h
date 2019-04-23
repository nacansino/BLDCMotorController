#ifndef XC_ISR_H
#define	XC_ISR_H

#include <xc.h> 

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

void __attribute__((__interrupt__, no_auto_psv)) _AD1Interrupt(void)
{
    int adc_result[4];
    int hallstate = (HallA << 2) | (HallB << 1) | HallC;
    
//adc trigger indicator on
    LATBbits.LATB15 = 1;
    
    adc_result[0]=0;
    adc_result[1]=0;
    adc_result[2]=0;
    adc_result[3]=0;    
    adc_result[0]=ADC1BUF0;
    adc_result[1]=ADC1BUF1;
    adc_result[2]=ADC1BUF2;
    adc_result[3]=ADC1BUF3;
    /*
	i2c_puts("ADC done!");
    i2c_putint(adc_result[0]);
    i2c_puts(" ");
    i2c_putint(adc_result[1]);
    i2c_puts(" ");
    i2c_putint(adc_result[2]);
    i2c_puts(" ");
    i2c_putint(adc_result[3]);
	i2c_puts("\n");    

    i2c_puts("adc buf results: ");
    i2c_putint(adc_result[0]);
	i2c_puts("\n");
    */
    
    /*Obtain current here
     Imeas=(I_COUNTERGAIN)*|ADCBUF-I_BITS_HALF|/I_BITS_HALF
     */
    if(adc_result[2]<I_BITS_HALF){
        Ib=I_BITS_HALF-adc_result[2];
        //Ib=(((long)I_BITS_HALF-adc_result[2])*I_COUNTERGAIN) >> 9;
    }else{
        Ib=adc_result[2]-I_BITS_HALF;
        //Ib=((long)(adc_result[2]-I_BITS_HALF)*I_COUNTERGAIN) >> 9;
    }
    if(adc_result[3]<I_BITS_HALF){
        Ic=I_BITS_HALF-adc_result[3];
        //Ic=(((long)I_BITS_HALF-adc_result[3])*I_COUNTERGAIN) >> 9;
    }else{
        Ic=adc_result[3]-I_BITS_HALF;        
        //Ic=((long)(adc_result[3]-I_BITS_HALF)*I_COUNTERGAIN) >> 9;
    }
    //i2c_putint(hallstate);i2c_puts(" ");i2c_putint(Ib);i2c_puts("  ");i2c_putint(Ic);i2c_puts("\n");

    //SDC3 = (int)(((long)Ib)*PERIOD_CTS/512);    //adjust PWM duty cycle
    
    /*choose only one appropriate sample
     * IA is the instantaneous armature current
     */
    if((hallstate==1)||(hallstate==2)||(hallstate==5)){
        //may current sa Ib
        IA=Ib;  //set armature current to measurement in Ib
    }else if ((hallstate==3)||(hallstate==4)||(hallstate==6)){
        //may current sa Ic
        IA=Ic;
    }
    
    //Ib=adc_result[2];
    //Ic=adc_result[3];
    //Adjust duty here
    //duty=(long)(adc_result[0]*100)/1024;
    //i2c_puts("nasa isr ako");
    
    /*duty calculator*/
    //duty prescaler
    if (throttle_prescale_ctr==THROTTLE_SAMPLE_PRESCALER){
        //reset and compute duty
        throttle_prescale_ctr=0;

    /*throttle correction parameters:
     * low:adcvalue: 51
     * high: adcvalue: 312    
     * throttle correction duty
     * 
    */
    
    //throttle=(((long)(adc_result[0]-51))<<ADC_MAP_BITS)/261;//old throttle
    throttle=(long)(adc_result[0]);//new throttle
    
    //throttle=(((long)(adc_result[0]))*100)>>10;   //adjusted to meet available throttle        
    }
    throttle_prescale_ctr++;
    
    /*high pedal lockout routine
     *This happens only at startup (HPLockout!=2)
     */
    if(HPLockout!=2){//check if HPLockout has not been lifted before
        HPLockout=1;
        if(throttle<HIGHTHROTTLE_THRESHOLD){HPLockout=0;}
    }
    
    /*Torque control Loop
     * do PID here (10-bit based)
     */
    //rescale measured armature current to 10-bit based
    /*
    IA=(IA<<ADC_MAP_BITS)/MAX_CURRENT_CTS;  //IA will be equal to ADC_MAP if sensed armature current is equal to max reference set (due to scaling)
    PID_IA_err[0] = throttle - IA; //calculate error
    PID_IA_err_acc += PID_IA_err[0];   //accumulate error (for I term)

    if(PID_IA_err_acc>PID_IA_MAX_ERROR){
        PID_IA_err_acc = PID_IA_MAX_ERROR;
    }else if(PID_IA_err_acc<PID_IA_MIN_ERROR){
        PID_IA_err_acc = PID_IA_MIN_ERROR;
    }
     */
    //    result = __builtin_clr();   // result = 0
     
    
    /*speed control*/
    //duty_old=duty;  //save old duty
    //duty_ref=(throttle*100)>>ADC_MAP_BITS;  //unmap; rescale to 100
    if(throttle<HIGHTHROTTLE_THRESHOLD_LTP){
        duty_ref=0;//reset reference
    }else{
        duty_ref=MAX_DUTY;	//pump duty cycle to maximum; let peak current limiting do its job
    }
    /*
    i2c_puts("duty:");i2c_putl(duty);
    i2c_puts(";ref:");i2c_putl(duty_ref);i2c_puts("\n");*/
    /*apply duty ramp filter*/
    if(duty_ref>duty){
        duty=duty+0.005;
    }else if(duty_ref<duty){
        duty=duty-0.01 ;
    }else{
        duty=duty_ref;
    }

    //duty=(duty*100)>>ADC_MAP_BITS;  //unmap; rescale to 100
    
    if(duty>MAX_DUTY){
        duty=MAX_DUTY;
    }else if (duty<MIN_DUTY){
        duty=MIN_DUTY;
    }
    
    //apply PWM
    PDC1 = PDC2 = PDC4 = (int)(duty*PERIOD_CTS /100);
    
    //i2c_puts("pedal:");i2c_putint(highpedal);i2c_puts("\n");
    
    //set duty
    if(OCP_flag){
        PDC1 = PDC2 = PDC4 = 0;
    }else{
        PDC1 = PDC2 = PDC4 = (int)(duty*PERIOD_CTS/100);
    }
    //after duty, apply gate signals
    if(HPLockout==2){
        ApplyGateSignal();
    }
    //i2c_putl(duty);
    //i2c_puts("\n");                                                                             

    //adjust trigger location for the next trigger; get average
    //trigger time in a period: t_trigger = Ton + (Toff/2)
    //in counts, Ton : MDC; PERIOD : PERIOD_CTS
    TRIG1 = MDC + (PERIOD_CTS-MDC)/2; // Point at which the ADC module is to be triggered by primary PWM 
    //PWMCON1bits.TRGIEN=0;//clear trgstat big
    //PWMCON1bits.TRGIEN=1;
    //while (PWMCON1bits.TRGSTAT == 1);  // Wait for ADC interrupt status change 
    
    //Measure current here

//adc trigger indicator off
    LATBbits.LATB15 = 0;
    
	IFS0bits.AD1IF = 0;              /* Clear ADC Interrupt Flag */
	//AD1STATbits.P0RDY = 0;           /* Clear the ADSTAT bits */

}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    tmr1_acc++; //accumulate timer level
    //print every 1s
    if(tmr1_ctr==5){
        tmr1_ctr=0;
    i2c_puts("adcbuffer:");i2c_putint(ADC1BUF0);i2c_puts(";");    
    i2c_puts("Throttle: ");i2c_putl(duty);i2c_puts("%; ");
    i2c_puts("Speed:");i2c_putl(speed);i2c_puts(" rpm; ");    
    i2c_puts("Ib:");i2c_putint(Ib);i2c_puts(" A; ");
    i2c_puts("Ic:");i2c_putint(Ic);i2c_puts(" A; ");
    i2c_puts("Arm I:");i2c_putint(IA);i2c_puts(" clicks\n");
    /*nothing to do here*/
    }else{tmr1_ctr++;}
    
    /*zero speed detector. this is triggered when one full cycle of timer interrupt occur
     without any input capture activity
    */
    if(zerospeed_flag==1){
        curr_time = 0;
        speed = 0;
    }else{
        zerospeed_flag = 1;
    }
    
    
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    curr_time = IC1BUF;
    /*do commutate here
     hallstate variable is the (index+1) of the mask codes (maskcodes_FW)
    */
    ApplyGateSignal();

    IFS0bits.IC1IF = 0; // Reset respective interrupt flag

    /*calculate speed*/
    zerospeed_flag=0;   //indicate that hall activity has occurred
    if(curr_time>init_time){
        hall_period = curr_time-init_time;
    }else{
        hall_period = (PR1-init_time)+curr_time;
    }
    init_time=curr_time;
    /*
    i2c_puts("-hall_period:\"");
    i2c_putint(hall_period);
    i2c_puts("\"\n");
    
    i2c_puts("-hallstate:");
    i2c_putint(hallstate);
    i2c_puts("\n");
    */
    speed=SPEED_CALC_CONSTANT/(long)hall_period;
    /*speed filter, running average (queue)*/
    speed_runningsum-=speed_hyst[speed_index];//subtract oldest sample
    speed_hyst[speed_index]=speed;  //update oldest sample
    speed_runningsum+=speed_hyst[speed_index];
    speed_index++;  //increment
    if(speed_index==SPEED_FILTER_SAMPLE){speed_index=0;}
    speed=speed_runningsum >> SPEED_FILTER_SAMPLE_BITS;    //get the average    
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{    
    /*do commutate here
     hallstate variable is the (index+1) of the mask codes (maskcodes_FW)
    */
    ApplyGateSignal();
    
    IFS0bits.IC2IF = 0; // Reset respective interrupt flag

}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
    ApplyGateSignal();
    
    IFS1bits.IC7IF = 0; // Reset respective interrupt flag
    
}

void __attribute__ ((__interrupt__, no_auto_psv)) _CM1Interrupt(void)
{
    if(OCP_EN){
        OCP_flag=1;//place OCP by default
        if((CM1CONbits.COUT)&&(CM2CONbits.COUT)){ //if both are HIGH, then no OCP (rising edge)
            OCP_flag=0;
        }
    }
    
    //close comparator isr
    CM1CONbits.CEVT = 0; // clear comparator event bit
    CM2CONbits.CEVT = 0; // clear comparator event bit
    IFS1bits.CMIF = 0; // Reset respective interrupt flag
}

void __attribute__((__interrupt__,no_auto_psv)) _CNInterrupt(void)
{
/*change notification dedicated for hall signal*/
/* Insert ISR Code Here*/
//    LATAbits.LATA7=1;
    /* Clear CN interrupt */
/*read PORT bits to determine what caused the change*/
    
        /*calculate hall period*/
    curr_time = TMR1;
    /*
    i2c_puts("currtime: ");i2c_putint(curr_time);
    i2c_puts(" init_time: "); 
    i2c_putint(init_time);
    i2c_puts("\n");
    */
    if(curr_time>init_time){
        hall_period = curr_time-init_time;
    }else{
        hall_period = (0xFFFF-init_time)+curr_time;
    }
    init_time=curr_time;
    /*Calculate Speed.
     speed=deltaTheta/deltat = (deltaTheta/halltrigger)/(deltat/halltrigger)
     >  (deltaTheta/halltrigger) is the theta per hall trigger
        computed as: (360deg/mechrev /(5 elecrev/mechrev (pole set) * 6 halltrigger/elecrev))    
     */
    //speed=((FCY*60)/(MOTOR_HALLTRIGGERPD*MOTOR_NUMPOLES*(long)hall_period*TMR_PRESCALER));    //rpm
    speed=97656/(long)hall_period;
    /*speed filter, running average (queue)*/
    speed_runningsum-=speed_hyst[speed_index];//subtract oldest sample
    speed_hyst[speed_index]=speed;  //update oldest sample
    speed_runningsum+=speed_hyst[speed_index];
    speed_index++;  //increment
    if(speed_index==SPEED_FILTER_SAMPLE){speed_index=0;}
    speed=speed_runningsum;    //get the average
    IFS1bits.CNIF = 0;
}

void init_dsp(){            // initialize dsp engine
    CORCONbits.US = 0;      // DSP engine multiplication is signed
    CORCONbits.IF = 1;      // Integer mode
}

void init_coefficients(){
    kp = (int)(Kp * (1<<Q));
    kd = (int)(Kd * (1<<Q));
    ki = (int)(Ki * (1<<Q));
}    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

