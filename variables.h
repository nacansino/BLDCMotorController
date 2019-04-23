#ifndef XC_VARIABLES_H
#define	XC_VARIABLES_H

#include <xc.h> // include processor files - each processor file is guarded.  

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

//Oscillator properties (configure here)
#define FOSC    (120000000ULL)
#define FCY     (FOSC/2)
#define FPWM    (20000ULL)
#define PERIOD_CTS  FOSC/FPWM

/*Status LED*/    
#define STATUSLED  LATDbits.LATD2    
    
/*Flag Pins*/    
#define OVP     PORTDbits.RD10
#define UVP     PORTDbits.RD9

#define OTP     PORTDbits.RB13

/*physical properties*/
#define MOTOR_NUMPOLES  24   //number of motor pole set (5 set of ABCA'B'C')
#define MOTOR_HALLTRIGGERPD 2   //number of hall triggers per electrical cycle

/*Isense constants*/
#define I_OFFSET    1.65
#define I_NUMBITS_HALF 9
#define I_NUMBITS_FULL 10
#define I_BITS_HALF 512
#define I_BITS_FULL 1024
#define I_COUNTERGAIN  88    //gain needed to be applied to recover the information for I. Calculated as (I_PN*I_OFFSET/(HALLGAIN*DIFFAMPGAIN))
    
/*Closed Loop Tool */
#define Q       8       // 1.08 format
#define Kp      2
#define Ki      0
#define Kd      0
#define PID_IA_MAX_ERROR   10000
#define PID_IA_MIN_ERROR   -10000    
    
/*interrupt priority*/
#define PRIORITY_HALL   5
#define PRIORITY_OTP    4
#define PRIORITY_OCP    2
#define PRIORITY_OVPUVP 3
#define PRIORITY_ADC    7
#define PRIORITY_TIMER  6
    
/*Limits*/    
#define MAX_DUTY    98
#define MIN_DUTY    0
#define DUTY_RAMP_TIME   3   //how long in seconds is the ramp time
#define TMR_PRESCALER   256
#define SPEED_FILTER_SAMPLE    32    
#define SPEED_FILTER_SAMPLE_BITS    5
#define SPEED_CALC_CONSTANT 292968.75   //60 [s/min]*(Fcy/PRESCALER) [counts/sec] / ((48 [counts/rev]))
#define DUTY_FILTER_SAMPLE    32    
#define DUTY_FILTER_SAMPLE_BITS    5
#define DUTY_SAMPLE_PRESCALER    16    
#define THROTTLE_SAMPLE_PRESCALER    16    
#define HIGHTHROTTLE_THRESHOLD  100  //OUT OF 1024; UPPER TRIGGER POINT
#define HIGHTHROTTLE_THRESHOLD_LTP  50  //OUT OF 1024; LOWER TRIGGER POINT
#define ADC_MAP 1024  //default range of signals from ADC
#define ADC_MAP_BITS    10   
#define MAX_CURRENT_CTS 350 //ADC count (0-511) to set max current reference of throttle
    
//OCP enable pin
#define OCP_EN  0
    
// pwm disable pin. If 0, pwm enabled. If 1, pwm disabled.
#define PWM_DISABLE LATEbits.LATE5

//Hall state
#define HallA   PORTFbits.RF6
#define HallB   PORTFbits.RF2
#define HallC   PORTFbits.RF3

//Gate LOW signals (6stepcontrol)
#define LA  LATEbits.LATE0
#define LB  LATEbits.LATE2
#define LC  LATEbits.LATE6

//current sense variables
long Ia, Ib, Ic, IA;
int OCP_flag;
    
//PID Parameters
int kp, ki, kd;
int PID_IA_err[0];
int PID_IA_err_acc;

register int result asm("A");

/*Speed calculator variable*/    
unsigned int init_time, curr_time,tmr1_ctr,tmr1_acc,speed_index;        
int zerospeed_flag;
long speed,hall_period,speed_hyst[SPEED_FILTER_SAMPLE],speed_runningsum=0;
int HPLockout; //0 for low pedal, 1 for high pedal, 2 for lifted HPLO
int pedalstate; //0 for low pedal, 1 for high pedal

/*Throttle variables*/
int throttle_index=0,throttle_prescale_ctr=0;
long throttle=0;

/*Duty Cycle variables*/
long duty_ramp_ctr=0;
float duty=0, duty_ref=0;  

int maskcodes_FW[6]={6,24,18,33,36,9}; 
/*Sensor table (+ dir)
    A   B   C   AH  AL  BH  BL  CH  CL  value   
    0   0   1   0   0   0   1   1   0   6
    0   1   0   0   1   1   0   0   0   24
    0   1   1   0   1   0   0   1   0   18
    1   0   0   1   0   0   0   0   1   33 
    1   0   1   1   0   0   1   0   0   36
    1   1   0   0   0   1   0   0   1   9

*/
int maskcodes_BW[6]={9,36,33,18,24,6}; 
/*Sensor table (+ dir)
    A   B   C   AH  AL  BH  BL  CH  CL  value   
    0   0   1   0   0   1   0   0   1   9
    0   1   0   1   0   0   1   0   0   36
    0   1   1   1   0   0   0   0   1   33
    1   0   0   0   1   0   0   1   0   18 
    1   0   1   0   1   1   0   0   0   24
    1   1   0   0   0   0   1   1   0   6

*/    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

