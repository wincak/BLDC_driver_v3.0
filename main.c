/*
 * File:   main.c
 * Author: D.W.
 *
 * Created: 22.4.2015
 * Project: BLDC driver v3.0
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>         /* sprintf() */
#include <float.h>

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include <plib/delays.h>
#include <plib/pcpwm.h>

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// Flags
volatile unsigned char flags_status;    // status flag word
bit flag_stop = 0 ;                         // stop motor
bit flag_ADC_data_rdy = 0;                  // ADC conversion complete
bit flag_SPI_data_rdy = 0;                  // new SPI data recieved
bit flag_velocity_rdy = 0;                      // velocity meas. ready
#ifdef DEBUG_STATUS
bit flag_debug_status = 0;
#endif

// Conditions
struct_status status = {0,0,0,0,0};

// Requests
unsigned int req_current = 0;
unsigned char req_motor_mode = 0;
int req_velocity = 0;

// SPI
unsigned char RX_tab[RX_tab_size];    // SPI data receive tab
unsigned char TX_tab[TX_tab_size];    // SPI data send tab

// Velocity calc
extern unsigned int rot_change_count_buffer;

// PCPWM
unsigned int dutycycle = 0;         // PWM dutycycle
extern unsigned int PCPWM_Mot_Period;    // Can be 1/4 of dutycycle!! Weird prescaler

// ADC
unsigned char ADC_tab[ADC_tab_size];       // A/D result tab
float LM335_ADC_8bittoV;        // for 8bit A/D result->voltage translation
float Thermistor_ADC_8bittoV;
float BATT_ADC_8bittoV;

// PCPWM
/* Motoring Mode */
extern unsigned char PCPWM_Mot_Config0;
extern unsigned char PCPWM_Mot_Config1;
extern unsigned char PCPWM_Mot_Config2;
extern unsigned char PCPWM_Mot_Config3;
extern unsigned int PCPWM_Mot_Period;
extern unsigned int PCPWM_Mot_Sptime;
/* Generator mode */
extern unsigned char PCPWM_Gen_Config0;
extern unsigned char PCPWM_Gen_Config1;
extern unsigned char PCPWM_Gen_Config2;
extern unsigned char PCPWM_Gen_Config3;
extern unsigned int PCPWM_Gen_Period;
extern unsigned int PCPWM_Gen_Sptime;

// PID
SPid PID_status = {0,0,10000,0,10,6,5};

// Commutation table (for 3.0 hardware version)
unsigned char pos1 = 0b00000110;      // PWM7 & PWM0 active
unsigned char pos2 = 0b00100100;      // PWM7 & PWM4 active
unsigned char pos3 = 0b00100001;      // PWM4 & PWM1 active
unsigned char pos4 = 0b00001001;      // PWM2 & PWM1 active
unsigned char pos5 = 0b00011000;      // PWM5 & PWM2 active
unsigned char pos6 = 0b00010010;      // PWM5 & PWM0 active


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    unsigned char ret = 0;

    
    /* Initialize I/O and Peripherals for application */
    InitApp();

    /* Calculate constants */
    //TabGen();     // required too much memory...

    /* Instant start: non-controlled mode */
#ifdef INSTANT_START
    req_motor_mode = mode_regen_CCW;
    //req_motor_mode = mode_motor_CW;
    //req_velocity = 33;
#endif

    do    // main program loop
    {
        if(!flags_error & !flag_stop){   // Everything OK?
            // No error
            if((req_motor_mode == motor_mode) /*&& (motor_mode)*/){
                asm("nop");     // everything ok..
            }
            else{
                switch (req_motor_mode){    // change motor mode
                    case mode_motor_CW: motor_init(CW); break;
                    case mode_motor_CCW: motor_init(CCW); break;
                    case mode_regen: check_direction(); break;
                    // Regen rotation manual set for debug ONLY!
                    case mode_regen_CW: regen_init(CW); break;
                    case mode_regen_CCW: regen_init(CCW); break;
                    case mode_free_run: free_run_init(); break;
                    default: motor_halt(); break;
                }
            }
        }


        if(flag_ADC_data_rdy){  // new A/D data?
            calc_ADC_data();

            
            // Check condition limits
            ret = limits_check();
            if(ret) motor_halt;
            
        }

        if(flag_SPI_data_rdy){  // new SPI data?
            SPI_request_update();
        }

        if(flag_velocity_rdy){ // new velocity measurement data ready?
            status.velocity = calc_velocity(rot_change_count_buffer);
        }

#ifdef DEBUG_STATUS     // UART status debugging messages
        if(flag_debug_status){
            flag_debug_status = 0;
            debug_status();
        }
#endif

    }while(1);

}

/* Stop motor on error and set error conditions */
void motor_halt(){
    OVDCOND = 0;
    OVDCONS = 0;
    PID_TIMER_ON = 0;
    set_dutycycle(0);
    flag_stop = 1;
    LED_RED = 1;

    Closepcpwm();

    // PID reset
    PID_status.dState = 0;
    PID_status.iState = 0;
}

/* Set dutycycle value for all PWM channels */
void set_dutycycle(unsigned int dtc){
    Setdc0pcpwm(dtc);
    Setdc1pcpwm(dtc);
    Setdc2pcpwm(dtc);
    Setdc3pcpwm(dtc);
    dutycycle = dtc;
}

/* Switch motor to motoring mode */
void motor_init(unsigned char direction){
    unsigned char USART_CW_msg[] = "CW  \r";
    unsigned char USART_CCW_msg[] = "CCW \r";

    // Configure & enable PCPWM Module
    LATB = 0;
    OVDCONS = 0;
    Openpcpwm(PCPWM_Mot_Config0, PCPWM_Mot_Config1, PCPWM_Mot_Config2,
            PCPWM_Mot_Config3, PCPWM_Mot_Period, PCPWM_Mot_Sptime);


#ifdef CHARGE_BOOTSTRAPS    /* TODO Bootstrap charging procedure */
    /* Charging bootstraps */
    INTCONbits.GIEH = 0;    // disable interrupts to stop commutation routines
                            // and avoid shortcuts

#endif
    // Motoring mode needs to find out rotor position to turn on the motor,
    // see commutation_asm.c for more info
    // Note: how about flipping the IC interrupt flag manually here?
    if (direction == CW) { // init CW commutation
        putsUSART((char *) USART_CW_msg);

        // change motor mode bits
        clrbit(flags_status, 2);
        clrbit(flags_status, 1);
        setbit(flags_status, 0);

        if (!HALL_A & !HALL_B & HALL_C) OVDCOND = pos1;
        else if (!HALL_A & HALL_B & HALL_C) OVDCOND = pos2;
        else if (!HALL_A & HALL_B & !HALL_C) OVDCOND = pos3;
        else if (HALL_A & HALL_B & !HALL_C) OVDCOND = pos4;
        else if (HALL_A & !HALL_B & !HALL_C) OVDCOND = pos5;
        else if (HALL_A & !HALL_B & HALL_C) OVDCOND = pos6;
        else motor_halt(); // error
    
    } else if (direction == CCW) { // init CCW commutation
        putsUSART((char *) USART_CCW_msg);

        // change motor mode bits
        clrbit(flags_status, 2);
        setbit(flags_status, 1);
        clrbit(flags_status, 0);

        if (!HALL_A & !HALL_B & HALL_C) OVDCOND = pos4;
        else if (!HALL_A & HALL_B & HALL_C) OVDCOND = pos5;
        else if (!HALL_A & HALL_B & !HALL_C) OVDCOND = pos6;
        else if (HALL_A & HALL_B & !HALL_C) OVDCOND = pos1;
        else if (HALL_A & !HALL_B & !HALL_C) OVDCOND = pos2;
        else if (HALL_A & !HALL_B & HALL_C) OVDCOND = pos3;
        else motor_halt(); // error
    } else motor_halt();

    set_dutycycle(DTC_MIN);
    PID_TIMER_ON = 1;
}

/* Switch motor to regenerative braking mode */
void regen_init(unsigned char direction) {
    //unsigned char USART_CW_msg[] = "GEN  \r";
    /* TODO Regen braking initialization*/
    Closepcpwm();
    OVDCOND = 0;
    OVDCONS = 0;
    LATB = 0;
    TRISBbits.RB0 = 0;
    TRISBbits.RB2 = 0;
    TRISBbits.RB5 = 0;

    if (direction == CW) {
        // change motor mode bits
        setbit(flags_status, 2);
        clrbit(flags_status, 1);
        setbit(flags_status, 0);

        //putsUSART((char *) USART_CW_msg);
    } else if (direction == CCW) {
        // change motor mode bits
        setbit(flags_status, 2);
        setbit(flags_status, 1);
        clrbit(flags_status, 0);
        
    } else motor_halt();

    Openpcpwm(PCPWM_Gen_Config0, PCPWM_Gen_Config1, PCPWM_Gen_Config2,
            PCPWM_Gen_Config3, PCPWM_Gen_Period, PCPWM_Gen_Sptime);
    set_dutycycle(2);

}

// This is NOT WORKING now, direction must be set manually.
// quite tricky to find it out automatically....
/* Before regenerative braking starts, rotation direction must be known */
void check_direction(void){
    // WARN! This routine will be quite messy if the motor gets locked...
    unsigned char position_buffer;

    while(status.velocity < 5);     // waiting for rotation

    while ((PORTA & HALL_MASK) != 0b00010000); // wait for position 1

    while((PORTA & HALL_MASK) == 0b00010000); // wait for next position

    position_buffer = PORTA & HALL_MASK;   // read second value

    if(position_buffer == 0b00011000)      // is position 2 the next?
        regen_init(CW);
    else if(position_buffer == 0b00010100) // how about position 6?
        regen_init(CCW);
    else motor_halt();  // must be an error

    return;
}

/* Switch motor to free run mode */
void free_run_init(){
    unsigned char USART_free_run_msg[] = "FREE\r";

    // Power off the motor
    OVDCOND = 0;
    PID_TIMER_ON = 0;
    set_dutycycle(0);

    Closepcpwm();
    LATB = 0;

    // Change motor mode bits
    clrbit(flags_status,2);
    clrbit(flags_status,1);
    clrbit(flags_status,0);

    putsUSART((char *)USART_free_run_msg);

}

/* Decode received SPI data and set request variables */
void SPI_request_update (void){
    unsigned char buff = 0;

    /* Load Current request value */
    // TODO: calculate req_current 8bit-16bit
    req_current = RX_tab[RX_CURRENT_REQ];

    /* Load motor mode request */
    // TODO: add regen braking
    buff = RX_tab[RX_MOTOR_MODE];
    switch(buff){   // set appropriate motor mode bits
        case SPI_free_run : {   // free run requested
            req_motor_mode = mode_free_run; break;
        }
        case SPI_CW: {          // motor CW requested
            req_motor_mode = mode_motor_CW; break;
        }
        case SPI_CCW : {         // motor CCW requested
            req_motor_mode = mode_motor_CCW; break;
        }
        default : {             // unknown command
            req_motor_mode = mode_free_run; motor_halt(); break;
        }
    }
}

/* Receive new SPI data and store to RX_tab */
unsigned char Receive_SPI_data(unsigned char length){
    getsSPI(RX_tab,length);

    return(0);
}

/* Transmit data from TX_tab over SPI */
unsigned char Transmit_SPI_data(unsigned char length){
    TX_tab[length] = '\0';    // inserting null terminator at tab's end
                              // (will not be sent)
    putsSPI(TX_tab);

    return(0);
}

/* Read data from A/D tab and calculate real values */
void calc_ADC_data (void){
    unsigned char ADC_buffer[ADC_tab_size];
    unsigned int current_buffer;
    unsigned int current_current;
    // bit current_current_direction;
    char Motor_temp;
    char Transistor_temp;
    unsigned char Batt_voltage;
    int count;

    /* Load data to buffer tab */
    for(count=ADC_tab_size; count>0; count--){
        ADC_buffer[count] = ADC_tab[count];
    }

    /* Calculate motor current */
    // load data
    current_buffer = (ADC_buffer[ADC_H_CURRENT])<<2 | (ADC_buffer[ADC_L_CURRENT]>>6);
    // remove offset (bear in mind voltage loss along copper trace)
    current_buffer = current_buffer - HALL_U_OFFSET;
    // average for one period
    current_current = ((current_buffer*dutycycle)/(4*PCPWM_Mot_Period));
    // save into global status register
    status.current = current_current;

    /* Calculate Motor temperature */
    /* TODO: Find out motor thermistor coefficients */
    /* https://learn.adafruit.com/thermistor/using-a-thermistor */
    Motor_temp = ADC_buffer[ADC_H_MOTOR_TEMP];
    Motor_temp = (char)(Motor_temp*Thermistor_ADC_8bittoV);
    status.motor_temp = Motor_temp;

#ifdef DEBUG_MOTOR_TEMP
    char USART_m_temp_msg[12];
    sprintf(USART_m_temp_msg,"M_TEMP:%d\n\r",status.motor_temp);
    putsUSART((char *)USART_m_temp_msg);
#endif

    /* Calculate Transistor temperature */
    Transistor_temp = ADC_buffer[ADC_H_TRANSISTOR_TEMP];
    Transistor_temp = (char)(Transistor_temp*LM335_ADC_8bittoV)-KELVIN_OFFSET;
    status.transistor_temp = Transistor_temp;

#ifdef DEBUG_TRANSISTOR_TEMP
    char USART_t_temp_msg[10];
    sprintf(USART_t_temp_msg,"T_TEMP:%d\n\r",status.transistor_temp);
    putsUSART((char *)USART_t_temp_msg);
#endif

    /* Calculate Battery voltage */
    /* TODO Calculate battery voltage */
    Batt_voltage = ADC_buffer[ADC_H_BATT_VOLTAGE];
    Batt_voltage = (char)(Batt_voltage*BATT_ADC_8bittoV);
    status.batt_voltage = Batt_voltage;

#ifdef DEBUG_BATT_VOLTAGE
    char USART_batt_V_msg[10];
    sprintf(USART_batt_V_msg,"BATT:%d\n\r",status.batt_voltage);
    putsUSART((char *)USART_batt_V_msg);
#endif

    /* Update TX_tab data */
    TX_tab[TX_H_CURRENT] = (current_current>>8) & 0x0003;
    TX_tab[TX_L_CURRENT] = current_current & 0x00FF;

    TX_tab[TX_TRANSISTOR_TEMP] = Transistor_temp;
    TX_tab[TX_MOTOR_TEMP] = Motor_temp;
    TX_tab[TX_BATT_VOLTAGE] = Batt_voltage;

}

/* Check if system condition is within operation limits */
char limits_check(void){
    if(status.motor_temp > M_TEMP_MAX) setbit(flags_status,FLT_T);
    else clrbit(flags_status,FLT_T);

    if(status.transistor_temp > T_TEMP_MAX) setbit(flags_status,FLT_T);
    else clrbit(flags_status,FLT_T);

    if((status.batt_voltage < V_BATT_MIN) || (status.batt_voltage >V_BATT_MAX))
        setbit(flags_status,FLT_U);
    else clrbit(flags_status,FLT_U);
    // SPI timeout is managed by Timer0 interrupt and overcurrent by PID

    if(flags_error) return(1);
    else return 0;
}

/* Calculate rotation speed from number of rot. hall sensor transitions */
int calc_velocity(unsigned int transition_count){
    // For TMR0 interrupt every second
    int velocity;

    velocity = transition_count/6;  // RPS,Six sensor transitions/revolution

#ifdef DEBUG_VELOCITY
    char USART_dbg_msg[15];
    sprintf(USART_dbg_msg, "RPS:%d\n\r", velocity);
    putsUSART((char *) USART_dbg_msg);
#endif

    flag_velocity_rdy = 0;
    return velocity;
}

short long UpdatePID(SPid * pid, int error, int measure)
{
  short long pTerm, dTerm, iTerm;

  IO_EXT_PORT = 1;

  pTerm = pid->pGain * error;   // calculate the proportional term

// calculate the integral state with appropriate limiting
  pid->iState += error;

  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin) pid->iState = pid->iMin;

  iTerm = pid->iGain * pid->iState;  // calculate the integral term

  dTerm = pid->dGain * (pid->dState - measure);

  pid->dState = measure;

  IO_EXT_PORT = 0;
  return (pTerm + dTerm + iTerm)/10;
}

#ifdef DEBUG_STATUS
void debug_status(void){
        char USART_dbg_msg[15];
        sprintf(USART_dbg_msg,"\nREQ:%d\n\r",req_current);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"STA:%d\n\r",status.current);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"DTC:%d\n\r",dutycycle);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"M_TEMP:%d C\n\r",status.motor_temp);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"T_TEMP:%d C\n\r",status.transistor_temp);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"BATT:%d dV\n\r",status.batt_voltage);
        putsUSART((char *)USART_dbg_msg);

        sprintf(USART_dbg_msg,"Veloc:%d rpm\n\r",60*status.velocity);
        putsUSART((char *)USART_dbg_msg);

        if(flags_error){
            char USART_dbg_msg[]="OpCond exceeded!\n\r";
            putsUSART((char *)USART_dbg_msg);
        }

        return;
}

#endif
