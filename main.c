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

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// Flags
volatile unsigned char flags_status;    // status flag word
bit flag_stop;                          // stop motor
bit flag_ADC_data_rdy;                  // ADC conversion complete
bit flag_SPI_data_rdy;                  // new SPI data recieved
// bit flag_veloc_rdy;                     // velocity meas. ready (not used)

// Requests
unsigned char req_current = 0;
unsigned char req_motor_mode = 0;

// SPI
unsigned char RX_tab[RX_tab_size];    // SPI data receive tab
unsigned char TX_tab[TX_tab_size];    // SPI data send tab

unsigned char sync_mode;
unsigned char bus_mode;
unsigned char smp_phase;

// Timer 0
unsigned char TMR0Config = 0;

// PCPWM
unsigned int dutycycle = 0;         // PWM dutycycle
extern unsigned int PCPWMPeriod;    // Can be 1/4 of dutycycle!! Weird prescaler

// ADC
unsigned char ADC_tab[ADC_tab_size];       // A/D result tab

// Conditions
// try not to use global variables, but function call parameters
// unsigned int current_current = 0;   // measured motor current


// PID


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


    while(1)    // main program loop
    {
        if(!flags_error & !flag_stop){   // Everything OK?
            // No error
            if(req_motor_mode == motor_mode){
                // update dutycycle
                asm("nop");
            }
            else{
                switch (req_motor_mode){    // change motor mode
                    case mode_motor_CW: motor_init(CW); break;
                    case mode_motor_CCW: motor_init(CCW); break;
                    case mode_regen_CW: regen_init(CW); break;
                    case mode_regen_CCW: regen_init(CCW); break;
                    case mode_free_run: /* TODO */ break;
                    default: motor_halt(); break;
                }
            }
        }


        if(flag_ADC_data_rdy){  // new A/D data?
            calc_ADC_data();
            PID();
            TX_tab_update;
        }

        if(flag_SPI_data_rdy){  // new SPI data?
            request_update();
        }


    }

}

// stop motor on error
void motor_halt(){
    OVDCOND = 0;
    set_dutycycle(0);
    flag_stop = 1;
    LED_RED = 1;
}

// Set dutycycle value for all PWM channels
void set_dutycycle(unsigned char dtc){
    Setdc0pcpwm(dtc);
    Setdc1pcpwm(dtc);
    Setdc2pcpwm(dtc);
    Setdc3pcpwm(dtc);
}

void motor_init(unsigned char direction){

#ifdef CHARGE_BOOTSTRAPS    /* TODO */
    /* Charging bootstraps */
    INTCONbits.GIEH = 0;    // disable interrupts to stop commutation routines
                            // and avoid shortcuts

#endif
    if(direction == CW){    // init CW commutation
        OVDCONS = 0;
        if(!HALL_A & !HALL_B & HALL_C) OVDCOND = pos1;
        else if(!HALL_A & HALL_B & HALL_C) OVDCOND = pos2;
        else if(!HALL_A & HALL_B & !HALL_C) OVDCOND = pos3;
        else if(HALL_A & HALL_B & !HALL_C) OVDCOND = pos4;
        else if(HALL_A & !HALL_B & !HALL_C) OVDCOND = pos5;
        else if(HALL_A & !HALL_B & HALL_C) OVDCOND = pos6;
        else OVDCOND = 0;   // error
    }
    else if(direction == CCW){  // init CCW commutation
        OVDCONS = 0;
        if(!HALL_A & !HALL_B & HALL_C) OVDCOND = pos4;
        else if(!HALL_A & HALL_B & HALL_C) OVDCOND = pos5;
        else if(!HALL_A & HALL_B & !HALL_C) OVDCOND = pos6;
        else if(HALL_A & HALL_B & !HALL_C) OVDCOND = pos1;
        else if(HALL_A & !HALL_B & !HALL_C) OVDCOND = pos2;
        else if(HALL_A & !HALL_B & HALL_C) OVDCOND = pos3;
        else OVDCOND = 0;// error
    }
    else motor_halt();
}

void SPI_request_update (void){
    unsigned char buff = 0;

    /* Load Current request value */
    req_current = RX_tab[0];

    /* Load motor mode request */
    // TODO: add regen braking
    buff = RX_tab[1];
    switch(buff){   // set appropriate motor mode bits
        case SPI_free_run : {   // free run requested
            clrbit(flags_status,2);
            clrbit(flags_status,1);
            clrbit(flags_status,0);
            break;
        }
        case SPI_CW: {          // motor CW requested
            clrbit(flags_status,2);
            clrbit(flags_status,1);
            setbit(flags_status,0);
            break;
        }
        case SPI_CCW : {         // motor CCW requested
            clrbit(flags_status,2);
            setbit(flags_status,1);
            clrbit(flags_status,0);
            break;
        }
        default : {             // unknown command
            clrbit(flags_status,2);
            clrbit(flags_status,1);
            clrbit(flags_status,0);
        }
    }
}

void calc_ADC_data (void){
    unsigned char ADC_buffer[ADC_tab_size];
    unsigned int current_buffer;
    unsigned int current_current;
    // bit current_current_direction;
    unsigned char Motor_temp;
    unsigned char Transistor_temp;
    unsigned char Batt_voltage;
    int count;

    /* Load data to buffer tab */
    for(count=ADC_tab_size; count>0; count--){
        ADC_buffer[count] = ADC_tab[count];
    }

    /* Calculate motor current */
    // load data
    current_buffer = (ADC_buffer[H_CURRENT])<<2 | (ADC_buffer[L_CURRENT]>>6);
    // remove offset (bear in mind voltage loss along copper trace)
    current_buffer = current_buffer - HALL_U_OFFSET;
    // average for one period
    current_current = (((current_buffer*dutycycle)/(4*PCPWMPeriod))
            +HALL_U_OFFSET);

    /* Calculate Motor temperature */
    /* TODO */
    Motor_temp = ADC_buffer[H_MOTOR_TEMP];

    /* Calculate Transistor temperature */
    /* TODO */
    Transistor_temp = ADC_buffer[H_TRANSISTOR_TEMP];

    /* Calculate Battery voltage */
    /* TODO */
    Batt_voltage = ADC_buffer[H_BATT_VOLTAGE];



}