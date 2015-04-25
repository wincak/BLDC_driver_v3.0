/*
 * File:   user.c
 * Author: D.W.
 *
 * Created: 21.4.2015
 * Project: BLDC driver v3.0
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

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

#endif

#include <plib/pcpwm.h>
#include <plib/spi.h>
#include <plib/timers.h>
#include "user.h"

/******************************************************************************/
/* Variables                                                                  */
/******************************************************************************/

extern unsigned char flags_status;

// SPI
//unsigned char SPI_Send[3] = "OK\0";
unsigned char SPI_Recv[25];

unsigned char sync_mode = 0;
unsigned char bus_mode = 0;
unsigned char smp_phase = 0;
unsigned char w = 0;

// Timer 0
unsigned char TMR0Config = 0;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void InitApp(void)
{
    LED_GREEN = 0;
    LED_RED = 0;

    /* Port initialization */
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;

    /* Disable PWM outputs */
    OVDCOND = 0;
    set_dutycycle(0);

    /* Set "online" flag */
    setbit(flags_status,online);


    /***********************   SPI   ******************************************/
    TRISDbits.RD1 = 0;      // serial data out

    CloseSPI();
    SSPBUF = 0x00;

    // SSP Interrupt config
    PIR1bits.SSPIF = 0;     // SSP flag clear
    IPR1bits.SSPIP = 0;     // SSP interrupt low priority
    PIE1bits.SSPIE = 1;     // SSP interrupt enable

    //***Configure SPI SLAVE module *****
    sync_mode = SLV_SSON;    // use Slave Select
    bus_mode = MODE_00;       // send data on rising edge
    smp_phase = SMPMID;
    OpenSPI(sync_mode,bus_mode,smp_phase );


    //**********************   INT2  ******************************************/
    // External interrupt for SPI Slave Select line monitoring
    // Falling Edge detection
    TRISCbits.RC5 = 1;          // INT2 -> input

    INTCON2bits.INTEDG2 = 0;    // interrupt on falling edge
    INTCON3bits.INT2IP = 0;     // INT2 low priority
    INTCON3bits.INT2IF = 0;     // INT2 interrupt flag clear
    INTCON3bits.INT2IE = 1;     // INT2 enable

    /***********************  INPUT CAPTURE  **********************************/

    // Noise filter (3xCLK)
    DFLTCONbits.FLTCK = 0b000;  // clock divider off
    DFLTCONbits.FLT1EN = 1;     // IC1 filter enable
    DFLTCONbits.FLT2EN = 1;     // IC2 filter enable
    DFLTCONbits.FLT3EN = 1;     // IC3 filter enable

    // IC1
    CAP1CONbits.CAP1M = 0b1000; // IC1 capture on every change
    PIR3bits.IC1IF = 0;         // flag reset
    IPR3bits.IC1IP = 1;         // high priority

    //IC2
    CAP2CONbits.CAP2M = 0b1000; // IC2 capture on every change
    PIR3bits.IC2QEIF = 0;       // flag reset
    IPR3bits.IC2QEIP = 1;       // high priority

    //IC3
    CAP3CONbits.CAP3M = 0b1000; // IC3 capture on every change
    PIR3bits.IC3DRIF = 0;       // flag reset
    IPR3bits.IC3DRIP = 1;       // high priority

    // IC enable
    PIE3bits.IC1IE = 1;     // IC1 interrupt enable
    PIE3bits.IC2QEIE = 1;   // IC2 interrupt enable
    PIE3bits.IC3DRIE = 1;   // IC3 interrupt enable

    /********************    ADC     ******************************************/
    // PIC18LF4431: ADC version 7 (see XC8 documentation)
    ADCON0bits.ACONV = 0;   // single shot conversion

    ADCON1bits.VCFG = 0b00; // AVdd, AVss reference

    ADCON2bits.ADFM = 0;        // left justified
    ADCON2bits.ACQT = 0b0000;   // no acquisition delay
    ADCON2bits.ADCS = 0b001;   //  A/D conversion clock

    ADCON3bits.ADRS = 0b000;    // ignored (Interrupt on every word in buffer)
    ADCON3bits.SSRC = 0b10000;  // PCPWM base trigger
                                // see PCPWM init for timing advance and scaler

    ADCON0bits.ACSCH = 1;   // Multi-Channel mode
    ADCON0bits.ACMOD = 11;  // Simultaneous conversion sequence GA+GB, GC+GD

    ADCHSbits.SASEL = 10;   // Group A - Current-sense (AN8)
    ADCHSbits.SBSEL = 00;   // Group B - Motor-temp (AN1)
    ADCHSbits.SCSEL = 01;   // Group C - Transistor-temp (AN6)
    ADCHSbits.SDSEL = 01;   // Group D - Battery-voltage (AN7)
    ADCON1bits.FIFOEN = 1;  // FIFO enabled

    PIE1bits.ADIE = 1;      // ADC interrupt: 1-enabled; 0-disabled
    PIR1bits.ADIF = 0;      // interrupt flag clear
    IPR1bits.ADIP = 1;      // ADC interrupt priority: 0-low

    ADCON0bits.ADON = 1;    // ADC On

    /********************   TIMER 0  ******************************************/
    // Connection timeout timer
    // check time to overflow
    TMR0Config = TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32;
    OpenTimer0(TMR0Config);
    INTCON2bits.TMR0IP = 1;     // high priority interrupt

    /*******************  INTERRUPTS  *****************************************/

    // Low Voltage Detect
    PIE2bits.LVDIE = 0;     // LVD interrupt disable

    RCONbits.IPEN = 1;      // enable interrupt levels

    /* Enable interrupts */
    INTCONbits.GIEH = 1;    // zapinam high level interrupty
    INTCONbits.GIEL = 1;    // zapinam low level interrupty

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

