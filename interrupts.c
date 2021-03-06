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
#include <stdlib.h>        /* abs() */

#endif

/******************************************************************************/
/*Variables                                                                   */
/******************************************************************************/
// Flags
extern unsigned char flags_status;
extern bit flag_stop;
extern bit flag_ADC_data_rdy;
extern bit flag_velocity_rdy;
#ifdef DEBUG_STATUS
extern bit flag_debug_status;
#endif

// Status
extern struct_status status;
extern unsigned int dutycycle;

// ADC
extern unsigned char ADC_tab[8];    // ADC result tab
unsigned char hbuf, lbuf;           // fast ASM decode variables

// Timing
unsigned int BF_timeout;
unsigned int rot_change_count = 0;      // hall sensor transition counter
unsigned int rot_change_count_buffer = 0; // buffer for storing counter values
unsigned char SPI_timeout_prescale;

// Requests
extern unsigned int req_current;
extern unsigned char req_motor_mode;
extern int req_velocity;

// PID without PhD
extern SPid PID_status;
unsigned int drive = 0;
unsigned char PID_prescaler;

// Rotor position decode
unsigned char hall_pos_curr;
extern unsigned char hall_pos1;
extern unsigned char hall_pos2;
extern unsigned char hall_pos3;
extern unsigned char hall_pos4;
extern unsigned char hall_pos5;
extern unsigned char hall_pos6;
extern unsigned char rotor_pos;
extern unsigned char rotor_pos_prev;

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */
void interrupt int_high()
{
    // All IC input changes are recognized as one event
    // Note: is it more efficient to differentiate them?
    if(PIR3bits.IC1IF || PIR3bits.IC2QEIF || PIR3bits.IC3DRIF){
        PIR3bits.IC1IF = 0; PIR3bits.IC2QEIF = 0; PIR3bits.IC3DRIF = 0;

        switch(motor_mode){     // Motor mode check
            case mode_free_run: break;                   // motor off
            case mode_motor_CW: commutate_mot(); break;  // motoring
            case mode_motor_CCW: commutate_mot(); break;
            case mode_regen: commutate_gen(); break;  // generating
            default: motor_halt(); break;                // error, stop
        }

        // Check rotor position
        hall_pos_curr = PORTA & HALL_MASK;
        rotor_pos_prev = rotor_pos;
        if(hall_pos_curr == hall_pos1) rotor_pos = 1;
        else if(hall_pos_curr == hall_pos2) rotor_pos = 2;
        else if(hall_pos_curr == hall_pos3) rotor_pos = 3;
        else if(hall_pos_curr == hall_pos4) rotor_pos = 4;
        else if(hall_pos_curr == hall_pos5) rotor_pos = 5;
        else if(hall_pos_curr == hall_pos6) rotor_pos = 6;
        else rotor_pos = 0;

        rot_change_count++;

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
        WriteTimer0(TMR0_1s);

#ifndef SPI_TIMEOUT_DISABLE
        setbit(flags_status,comm_error);  // comm_error flag set
#endif
#ifdef DEBUG_STATUS
        flag_debug_status = 1;
#endif
        LED_GREEN = !LED_GREEN;         // Green LED blink

        // Store value for velocity calculation and reset the counter
        rot_change_count_buffer = rot_change_count;
        rot_change_count = 0;
        flag_velocity_rdy = 1;
    }

}

/* Low-priority service */
void interrupt low_priority int_low() {
    // SPI data exchange
    if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) {
        INTCON3bits.INT2IF = 0;
        PIR1bits.SSPIF = 0;
        unsigned char address;
        unsigned int timeout;

        LED_RED = !LED_RED;
        SSPCONbits.SSPEN = 1;   // turn on SPI
        SSPBUF=0;

        timeout = 5000; // wait
        while (!SSPSTATbits.BF && timeout > 0){    // wait for buffer to be full (potential lockup)
            timeout--;
        }
        address = SSPBUF;

        if (address == SPI_ADDRESS) {
            // Exchange data
            SSPBUF = 0; // clear address from buffer
            Receive_SPI_data(RX_tab_size); // Read SPI data
            Transmit_SPI_data(TX_tab_size); // Write SPI data

        }
        SSPBUF=0;
        SSPCONbits.SSPEN = 0;   // turn off SPI
        LED_RED = !LED_RED;
                
        // SPI Communication status okay
        WriteTimer0(0x0000); // SPI timeout timer clear
        clrbit(flags_status, comm_error); // comm_err flag clear
        LED_GREEN = 1;

    }
    // PID routine
    else if(PIR1bits.TMR1IF && PIE1bits.TMR1IE){
        PIR1bits.TMR1IF = 0;
        WriteTimer1(TMR1_50ms);


#if 0
        // PID velocity regulator

        if (PID_prescaler == 0) {
            PID_prescaler = 5;

            // Begin PID without PhD
            drive = (unsigned int) UpdatePID(&PID_status,  \
                req_velocity - status.velocity, status.velocity);
            if (drive <= DTC_MAX) set_dutycycle(drive);
            else set_dutycycle(DTC_MAX);
#ifdef DEBUG_PID
            char USART_dbg_msg[15];
            sprintf(USART_dbg_msg, "\nPID:%d\n\r", drive);
            putsUSART((char *) USART_dbg_msg);
#endif
        }
        PID_prescaler--;
#endif
        
        // Begin Makeshift linear current regulator
        if(abs(req_current - status.current) >= 5){ // error big enough?
            if(status.current < req_current){ // increase current
                if(dutycycle < DTC_MAX)
                    set_dutycycle(dutycycle + DTC_STEP);
            }
            else if (status.current > req_current){  // decrease current
                if(dutycycle > DTC_MIN)
                    set_dutycycle(dutycycle - DTC_STEP);
            }
            else set_dutycycle(0);    // something went wrong, abort

//
//#ifdef DEBUG_PID
//            char USART_dtc_msg[10];
//
//            sprintf(USART_dtc_msg,"REQ:%d\n\r",req_current);
//            putsUSART((char *)USART_dtc_msg);
//
//            sprintf(USART_dtc_msg,"STA:%d\n\r",status.current);
//            putsUSART((char *)USART_dtc_msg);
//
//            sprintf(USART_dtc_msg,"DTC:%d\n\r",dutycycle);
//            putsUSART((char *)USART_dtc_msg);
//#endif
        }

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
                case '1' : {
                    set_dutycycle(dutycycle +1); break;
                }
                case '2' : {
                    set_dutycycle(dutycycle -1); break;
                }
            }
        }

        LED_GREEN = 0;

    }
#endif

}
