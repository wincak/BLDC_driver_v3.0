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
#include <stdlib.h>        /* abs() */
#include <stdio.h>         /* sprintf() */
#include <float.h>

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include <plib/delays.h>

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// Flags
volatile unsigned char flags_status;    // status flag word
bit flag_stop = 0 ;                         // stop motor
bit flag_ADC_data_rdy = 0;                  // ADC conversion complete
bit flag_SPI_data_rdy = 0;                  // new SPI data recieved
// bit flag_veloc_rdy;                      // velocity meas. ready (not used)

// Conditions
struct {
    unsigned int current;
    unsigned char motor_temp;
    unsigned char transistor_temp;
    unsigned char batt_voltage;
} status = {0,0,0,0};

// Requests
unsigned int req_current = 0;
unsigned char req_motor_mode = 0;

// SPI
unsigned char RX_tab[RX_tab_size];    // SPI data receive tab
unsigned char TX_tab[TX_tab_size];    // SPI data send tab

// Timer 0
unsigned char TMR0Config = 0;

// PCPWM
unsigned int dutycycle = 0;         // PWM dutycycle
extern unsigned int PCPWMPeriod;    // Can be 1/4 of dutycycle!! Weird prescaler

// ADC
unsigned char ADC_tab[ADC_tab_size];       // A/D result tab
float LM335_ADC_8bittoV;        // for 8bit A/D result->voltage translation
float Thermistor_ADC_8bittoV;
float BATT_ADC_8bittoV;


// PID
unsigned char prescaler = 100;


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
    /* Initialize I/O and Peripherals for application */
    InitApp();

    /* Calculate constants */
    //TabGen();     // required too much memory...

    do    // main program loop
    {
        if(!flags_error & !flag_stop){   // Everything OK?
            // No error
            if(req_motor_mode == motor_mode){
                // update dutycycle
            }
            else{
                switch (req_motor_mode){    // change motor mode
                    case mode_motor_CW: motor_init(CW); break;
                    case mode_motor_CCW: motor_init(CCW); break;
                    case mode_regen_CW: regen_init(CW); break;
                    case mode_regen_CCW: regen_init(CCW); break;
                    case mode_free_run: free_run_init(); break;
                    default: motor_halt(); break;
                }
            }
        }


        if(flag_ADC_data_rdy){  // new A/D data?
            calc_ADC_data();
            if(!prescaler){     // run PID every 100 cycles
                PID();
                prescaler = 100;
            }else prescaler--;

        }

        if(flag_SPI_data_rdy){  // new SPI data?
            SPI_request_update();
        }

    }while(1);

}

// stop motor on error
void motor_halt(){
    OVDCOND = 0;
    set_dutycycle(0);
    flag_stop = 1;
    LED_RED = 1;
}

// Set dutycycle value for all PWM channels
void set_dutycycle(unsigned int dtc){
    Setdc0pcpwm(dtc);
    Setdc1pcpwm(dtc);
    Setdc2pcpwm(dtc);
    Setdc3pcpwm(dtc);
    dutycycle = dtc;
}

void motor_init(unsigned char direction){
    unsigned char USART_CW_msg[] = "CW  \r";
    unsigned char USART_CCW_msg[] = "CCW \r";


#ifdef CHARGE_BOOTSTRAPS    /* TODO Bootstrap charging procedure */
    /* Charging bootstraps */
    INTCONbits.GIEH = 0;    // disable interrupts to stop commutation routines
                            // and avoid shortcuts

#endif
    if(direction == CW){    // init CW commutation
        putsUSART((char *)USART_CW_msg);

        // change motor mode bits
        clrbit(flags_status,2);
        clrbit(flags_status,1);
        setbit(flags_status,0);

        OVDCONS = 0;
        if(!HALL_A & !HALL_B & HALL_C) OVDCOND = pos1;
        else if(!HALL_A & HALL_B & HALL_C) OVDCOND = pos2;
        else if(!HALL_A & HALL_B & !HALL_C) OVDCOND = pos3;
        else if(HALL_A & HALL_B & !HALL_C) OVDCOND = pos4;
        else if(HALL_A & !HALL_B & !HALL_C) OVDCOND = pos5;
        else if(HALL_A & !HALL_B & HALL_C) OVDCOND = pos6;
        else motor_halt();   // error
    }
    else if(direction == CCW){  // init CCW commutation
        putsUSART((char *)USART_CCW_msg);

        // change motor mode bits
        clrbit(flags_status,2);
        setbit(flags_status,1);
        clrbit(flags_status,0);

        OVDCONS = 0;
        if(!HALL_A & !HALL_B & HALL_C) OVDCOND = pos4;
        else if(!HALL_A & HALL_B & HALL_C) OVDCOND = pos5;
        else if(!HALL_A & HALL_B & !HALL_C) OVDCOND = pos6;
        else if(HALL_A & HALL_B & !HALL_C) OVDCOND = pos1;
        else if(HALL_A & !HALL_B & !HALL_C) OVDCOND = pos2;
        else if(HALL_A & !HALL_B & HALL_C) OVDCOND = pos3;
        else motor_halt();// error
    }
    else motor_halt();

    set_dutycycle(DTC_min);
}

void regen_init(unsigned char direction){
    /* TODO Regen braking initialization*/
    asm("nop");
}

void free_run_init(){
    unsigned char USART_free_run_msg[] = "FREE\r";

    // Power off the motor
    OVDCOND = 0;
    set_dutycycle(0);

    // Change motor mode bits
    clrbit(flags_status,2);
    clrbit(flags_status,1);
    clrbit(flags_status,0);

    putsUSART((char *)USART_free_run_msg);

}

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

unsigned char Receive_SPI_data(unsigned char length){
    getsSPI(RX_tab,length);

    return(0);
}

unsigned char Transmit_SPI_data(unsigned char length){
    TX_tab[length] = '\0';    // inserting null terminator at tab's end
                              // (will not be sent)
    putsSPI(TX_tab);

    return(0);
}

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
    current_current = (((current_buffer*dutycycle)/(4*PCPWMPeriod))
            +HALL_U_OFFSET);
    // save into global status register
    status.current = current_current;

    /* Calculate Motor temperature */
    /* TODO: Find out motor thermistor coefficients */
    Motor_temp = ADC_buffer[ADC_H_MOTOR_TEMP];
    Motor_temp = (char)(Motor_temp*Thermistor_ADC_8bittoV);
    status.motor_temp = Motor_temp;
#ifdef MOTOR_TEMP_DEBUG
    char USART_m_temp_msg[12];
    sprintf(USART_m_temp_msg,"T_TEMP:%d\n\r",status.motor_temp);
    putsUSART((char *)USART_m_temp_msg);
#endif

    /* Calculate Transistor temperature */
    Transistor_temp = ADC_buffer[ADC_H_TRANSISTOR_TEMP];
    Transistor_temp = (char)(Transistor_temp*LM335_ADC_8bittoV)-KELVIN_OFFSET;
    status.transistor_temp = Transistor_temp;
#ifdef TRANSISTOR_TEMP_DEBUG
    char USART_t_temp_msg[10];
    sprintf(USART_t_temp_msg,"T_TEMP:%d\n\r",status.transistor_temp);
    putsUSART((char *)USART_t_temp_msg);
#endif
    IO_EXT_PORT = 1;

    /* Calculate Battery voltage */
    /* TODO Calculate battery voltage */
    Batt_voltage = ADC_buffer[ADC_H_BATT_VOLTAGE];
    Batt_voltage = (char)(Batt_voltage*BATT_ADC_8bittoV);
    status.batt_voltage = Batt_voltage;
    IO_EXT_PORT = 0;

#ifdef BATT_VOLTAGE_DEBUG
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

void PID(void){
    /* TODO PID regulator */
    
    // Begin Makeshift linear regulator
    if(abs(req_current-status.current) >= 5){ // error big enough?
        if(status.current < req_current){ // increase current
            if(dutycycle < DTC_max)
                set_dutycycle(dutycycle + DTC_step);
        }
        else if (status.current > req_current){  // decrease current
            if(dutycycle > DTC_min)
                set_dutycycle(dutycycle - DTC_step);
        }
        else set_dutycycle(0);    // something went wrong, abort
#ifdef PID_DEBUG
        char USART_dtc_msg[10];

        sprintf(USART_dtc_msg,"REQ:%d\n\r",req_current);
        putsUSART((char *)USART_dtc_msg);

        sprintf(USART_dtc_msg,"STA:%d\n\r",status.current);
        putsUSART((char *)USART_dtc_msg);

        sprintf(USART_dtc_msg,"DTC:%d\n\r",dutycycle);
        putsUSART((char *)USART_dtc_msg);
#endif

    }


}