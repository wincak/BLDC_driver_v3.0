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

#include "user.h"
#include "system.h"

#include <plib/pcpwm.h>
#include <plib/spi.h>
#include <plib/timers.h>
#ifdef UART_CONTROL
    #include <plib/usart.h>
#endif


/******************************************************************************/
/* Variables                                                                  */
/******************************************************************************/

// Flags
extern unsigned char flags_status;

// SPI
unsigned char sync_mode = 0;
unsigned char bus_mode = 0;
unsigned char smp_phase = 0;

#ifdef UART_CONTROL
unsigned char UARTConfig = 0;
unsigned char spbrg = 0;
unsigned char baudconfig = 0;
#endif

// PCPWM
unsigned char PCPWMConfig0 = 0;
unsigned char PCPWMConfig1 = 0;
unsigned char PCPWMConfig2 = 0;
unsigned char PCPWMConfig3 = 0;
unsigned int PCPWMPeriod = 0;
unsigned int PCPWMSptime = 0;

// Timer 0
unsigned char TMR0Config = 0;

// Conversion data and tables
extern float LM335_ADC_8bittoV;
extern float Thermistor_ADC_8bittoV;
extern float BATT_ADC_8bittoV;
//unsigned int LUT_I_req[CHAR_MAX+1];     // Table for current request translation
                                        // 8bit request -> 16 bit internal use

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

    /********************  VSTUPY/VYSTUPY  ************************************/
    TRISCbits.RC3 = 0;    // LED2 output (RED)
    TRISDbits.RD0 = 0;    // LED1 output (GREEN)

    ANSEL0 = 0b11100010;    // analog input settings

#ifdef DEBUG_PINS
    // debugging outputs
    IO_EXT_TRIS = 0;    
    FLT_EXT_TRIS = 0;
#endif

#ifndef UART_CONTROL
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
#endif

#ifdef UART_CONTROL
    /***********************   UART  ******************************************/
    unsigned char UART_start_msg[] = "UART ONLINE\n";

    TRISCbits.RC5 = 1;  // this pin is connected with TX pin in hardware
    TRISCbits.RC6 = 1;  // USART TX pin

    CloseUSART();

    TXSTAbits.TXEN = 1;     // UART Transmitter enable
    TXSTAbits.BRGH = 1;     // High baud rate
    RCSTAbits.SPEN = 1;     // Serial port enable
    RCSTAbits.CREN = 1;     // UART Receiver enable
    BAUDCONbits.BRG16 = 1;  // 16-bit baud rate generator

    SPBRG = BAUD_115200;     // Baud rate divider.


    // Set interrupts once againt, this time for real
    PIE1bits.TXIE = 0;  // USART TX interrupt disable
    PIE1bits.RCIE = 1;  // USART RX interrupt enable
    IPR1bits.RCIP = 0;  // USART RX interrupt low priority

    // test
    TRISCbits.RC6 = 1;  // USART TX pin

    // Startup message
    while(BusyUSART());
    putsUSART((char *)UART_start_msg);
    WriteUSART('\r');

#endif
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
    ADCON0bits.ACMOD = 0b11;  // Simultaneous conversion sequence GA+GB, GC+GD

    ADCHSbits.SASEL = 0b10;   // Group A - Current-sense (AN8)
    ADCHSbits.SBSEL = 0b00;   // Group B - Motor-temp (AN1)
    ADCHSbits.SCSEL = 0b01;   // Group C - Transistor-temp (AN6)
    ADCHSbits.SDSEL = 0b01;   // Group D - Battery-voltage (AN7)
    ADCON1bits.FIFOEN = 1;  // FIFO enabled

    PIE1bits.ADIE = 1;      // ADC interrupt: 1-enabled; 0-disabled
    PIR1bits.ADIF = 0;      // interrupt flag clear
    IPR1bits.ADIP = 1;      // ADC interrupt priority: 1-high

    ADCON0bits.ADON = 1;    // ADC On

    /********************   TIMER 0  ******************************************/
    // Connection timeout timer
    // check time to overflow
#ifdef USE_PLL
    TMR0Config = TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_128;
#else
    TMR0Config = TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32;
#endif
    OpenTimer0(TMR0Config);
    INTCON2bits.TMR0IP = 1;     // high priority interrupt

    /***********************  PCPWM  ******************************************/
    // Comments:
    // check PTCON0 postscale-reason why pwm timer counts by adding 4?
    // PWMCON1 - spec event trigger postscale (for ADC)
    // PCPWM init
    PCPWMConfig0 = PWM_IO_ALL & PWM_0AND1_INDPEN & PWM_2AND3_INDPEN
            & PWM_4AND5_INDPEN & PWM_6AND7_INDPEN;
    PCPWMConfig1 = PW_SEVT_POS_1_16 & PW_SEVT_DIR_UP & PW_OP_SYNC;
    PCPWMConfig2 = PT_POS_1_16 & PT_PRS_1_1 & PT_MOD_CNT_UPDN;
    PCPWMConfig3 = PT_ENABLE & PT_CNT_UP;   // warn: this is correct,
                                            // comment in plib is wrong
    PCPWMPeriod = PWM_PERIOD;

    PCPWMSptime = 0x01; // 0x01 means trigger for A/D conversion right in
                        // the middle of PWM pulse

    set_dutycycle(0);

    Openpcpwm(PCPWMConfig0, PCPWMConfig1, PCPWMConfig2, PCPWMConfig3,
            PCPWMPeriod, PCPWMSptime);

    TRISBbits.RB2 = 1;  // channel 7 & 8 pins
    TRISBbits.RB3 = 1;  // set as output

    PIE3bits.PTIE = 0;  // PWM time base interrupt enable/disable
    PIR3bits.PTIF = 0;  // PWM time base interrupt clear

    /*******************  INTERRUPTS  *****************************************/

    // Low Voltage Detect
    PIE2bits.LVDIE = 0;     // LVD interrupt disable

    RCONbits.IPEN = 1;      // enable interrupt levels

    /* Enable interrupts */
    INTCONbits.GIEH = 1;    // zapinam high level interrupty
    INTCONbits.GIEL = 1;    // zapinam low level interrupty

    /*******************  CALCULATION OF CONSTANTS*****************************/

    // A/D result calculation
    LM335_ADC_8bittoV = VCC/CHAR_MAX*100;   // 10mV/K
    Thermistor_ADC_8bittoV = VCC/CHAR_MAX;  // TODO: find out temp. coefficient
    BATT_ADC_8bittoV = (10*VCC/CHAR_MAX)/BATT_V_DIVIDER; // output as Ux10^-1 V

}

/* TabGen
 *
 * Calculates table of constants to translate legacy 8-bit SPI current requests
 * to 16-bit value used internally
 * WARNING: Uses like 65% of data memory!
 */
/*
void TabGen(void){
    int pos;

    LED_GREEN = 1;
    for(pos=CHAR_MAX; pos>=0; pos--){
        LUT_I_req[pos] = (pos/CHAR_MAX)*INT_MAX;
    }

    LED_GREEN = 0;

    return;
}
*/
