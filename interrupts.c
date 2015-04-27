/*
 * File:   interrupts.c
 * Author: D.W.
 *
 * Created: 21.4.2015
 * Project: BLDC driver v3.0
 */

/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#include "user.h"
#include "system.h"

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#include <stdio.h>         /* sprintf() */

#endif

/******************************************************************************/
/*Variables                                                                   */
/******************************************************************************/
// Flags
extern unsigned char flags_status;
extern bit flag_stop;
extern bit flag_ADC_data_rdy;
#ifdef DEBUG_STATUS
extern bit flag_debug_status;
#endif

// Status
extern struct_status status;
extern unsigned int dutycycle;

// ADC
extern unsigned char ADC_tab[8];    // ADC result tab
unsigned char hbuf, lbuf;           // fast ASM decode variables

// Timeout
unsigned int BF_timeout;

// Requests
extern unsigned int req_current;
extern unsigned char req_motor_mode;


/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */
void interrupt int_high()
{

    if(PIR3bits.IC1IF || PIR3bits.IC2QEIF || PIR3bits.IC3DRIF){
        PIR3bits.IC1IF = 0; PIR3bits.IC2QEIF = 0; PIR3bits.IC3DRIF = 0;

        switch(motor_mode){     // Motor mode check
            case mode_free_run: break;                   // motor off
            case mode_motor_CW: commutate_mot(); break;  // motoring
            case mode_motor_CCW: commutate_mot(); break;
            case mode_regen_CW: commutate_gen(); break;  // generating
            case mode_regen_CCW: commutate_gen(); break;
            default: motor_halt(); break;                // error, stop
        }
    }
    else if(PIR1bits.ADIF && PIE1bits.ADIE){
        PIR1bits.ADIF = 0;

        asm("movff  ADRESL, _lbuf");    // Current-sense (low byte)
        asm("movff  ADRESH, _hbuf");    // (high byte)
        ADC_tab[0] = hbuf; ADC_tab[1] = lbuf;

        asm("movff  ADRESL, _lbuf");    // Motor-temp
        asm("movff  ADRESH, _hbuf");
        ADC_tab[2] = hbuf; ADC_tab[3] = lbuf;

        asm("movff  ADRESL, _lbuf");    // Transistor-temp
        asm("movff  ADRESH, _hbuf");
        ADC_tab[4] = hbuf; ADC_tab[5] = lbuf;

        asm("movff  ADRESL, _lbuf");    // Batt-voltage
        asm("movff  ADRESH, _hbuf");
        ADC_tab[6] = hbuf; ADC_tab[7] = lbuf;

        flag_ADC_data_rdy = 1;  // Conversion complete flag
    }
    else if(INTCONbits.TMR0IF && INTCONbits.TMR0IE){
        INTCONbits.TMR0IF = 0;

#ifndef UART_CONTROL
        setbit(flags_status,comm_error);  // comm_error flag set
#endif
#ifdef DEBUG_STATUS
        flag_debug_status = 1;
#endif
        LED_GREEN = !LED_GREEN;         // Green LED blink
    }

}

/* Low-priority service */
void interrupt low_priority int_low()
{
    if(INTCON3bits.INT2IF && INTCON3bits.INT2IE){
        INTCON3bits.INT2IF = 0;

        LED_RED = 1;

        // TODO change SPI receive interrupt routine

        BF_timeout = 50000;   // Aby se necekalo na naplneni bufferu donekonecna
        do{                 // timeout nastaven od oka... 50 us
            if(BF_timeout<=0) break;
            else BF_timeout--;
        }while ( !SSPSTATbits.BF );  // cekam na naplneni bufferu

        // Exchange data
        Receive_SPI_data(RX_tab_size);     // Read SPI data
        Transmit_SPI_data(TX_tab_size);    // Write SPI data

        // SPI Communication status okay
        WriteTimer0(0x0000);                // SPI timeout timer clear
        clrbit(flags_status,comm_error);    // comm_err flag clear
        LED_GREEN = 1;

        LED_RED = 0;
    }
#ifdef UART_CONTROL
    else if(PIR1bits.RCIF && PIE1bits.RCIE){
        char UART_RX_buff;

        LED_GREEN = 1;

        // Read the receive register to clear interrupt flag
        while(PIR1bits.RCIF){
            UART_RX_buff = ReadUSART();

            switch(UART_RX_buff){   // set appropriate motor mode bits
                case ' ' : {   // free run requested
                    req_motor_mode = mode_free_run; break;
                }
                case 'w': {          // motor CW requested
                    req_motor_mode = mode_motor_CW; break;
                }
                case 's' : {         // motor CCW requested
                    req_motor_mode = mode_motor_CCW; break;
                }
                case '+' : {
                    if(req_current <= INT_MAX-REQ_I_STEP)
                        req_current += REQ_I_STEP; break;
                }
                case '-' : {
                    if(req_current >= REQ_I_STEP)
                        req_current -= REQ_I_STEP; break;
                }
            }
        }

        LED_GREEN = 0;

    }
#endif
}