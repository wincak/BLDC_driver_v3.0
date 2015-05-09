/*
 * File:   comm.c
 * Author: D.W.
 *
 * Created: 9.5.2015
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

#include <stdio.h>         /* sprintf() */

#endif

/******************************************************************************/
/*Variables                                                                   */
/******************************************************************************/

// Requests
extern unsigned int req_current;
extern unsigned char req_motor_mode;
extern int req_velocity;



/******************************************************************************/
/* Communication routines                                                     */
/******************************************************************************/

/**********     SPI     *******************************************************/

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
    //unsigned char tmp;
    getsSPI(RX_tab,length);

    //tmp = SSPBUF;

    return(0);
}

/* Transmit data from TX_tab over SPI */
unsigned char Transmit_SPI_data(unsigned char length){
    unsigned char count, tmp;
    TX_tab[length] = '\0';    // inserting null terminator at tab's end
                              // (will not be sent)
    FLT_EXT_PORT=1;
    //putsSPI(TX_tab);
    for(count=0;count<8;count++){
        SSPBUF=TX_tab[count];
        while(!SSPSTATbits.BF  && !SLAVE_SELECT );
    }

    tmp = SSPBUF;   // clear buffer
    FLT_EXT_PORT=0;

    return(0);
}


/**********     UART    *******************************************************/

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