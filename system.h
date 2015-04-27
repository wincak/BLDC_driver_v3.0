/*
 * File:   system.h
 * Author: D.W.
 *
 * Created: 22.4.2015
 * Project: BLDC driver v3.0
 */

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* Driver hardware version */
//#define V1_1  // unsupported
//#define V2_0  // unsupported
#define V3_0

/* Microcontroller MIPs (FCY) */
#define _XTAL_FREQ      10000000L   // external crystal frequency
#define USE_PLL                     // is PLL switched on?

#ifdef USE_PLL
    #define SYS_FREQ        _XTAL_FREQ*4
#else
    #define SYS_FREQ    _XTAL_FREQ
#endif
#define FCY             SYS_FREQ/4

/* Data type limits */
// useless? yeah, but better to see where that number came from
#define CHAR_MAX    255
#define INT_MAX     1023

/* ADC condition definitions */
// Microcontroller input voltage
//#define VCC     3.3   // fs <= 10MHz only
#define VCC     4.7       // Use SPI voltage level shift!
#define HALL_U_OFFSET   511     // Current Hall Sensor Offset
#define KELVIN_OFFSET   273     // 0°C ~ 273K
#define BATT_V_DIVIDER  (3.9/(12+3.9))    // Battery voltage sense divider

/* Hall rotation sensor mask */
#define HALL_MASK   0b00011100

/* Bootstrap charging enable */
//#define CHARGE_BOOTSTRAPS

/* UART control */
#define UART_CONTROL
#ifdef USE_PLL
    #define BAUD_115200     86
#else
    #define BAUD_115200     21
#endif

/* PWM definitions */
#ifdef USE_PLL
    // Period PWM: 40MHz, 0x00FF, UPDN => 9.8 kHz
    #define PWM_PERIOD  0x01FE  // default PWM period
    #define DTC_min     200     // change!
    #define DTC_max     1500    // change!
    #define DTC_step    100     // change! (for USART debug)
#else
    // Period PWM: 10MHz, 0x00FF, UPDN => 4.7 kHz
    #define PWM_PERIOD  0x00FF  // default PWM period
    #define DTC_min     100     // change!
    #define DTC_max     750     // change!
    #define DTC_step    50      // change! (for USART debug)
#endif


/* Current limit */
#define I_MAX   5   // Max current in Amps

/* Status LED definitions */
#define LED_GREEN   LATDbits.LD0
#define LED_RED     LATCbits.LC3

/* Motor rotation hall sensors */
#define HALL_A      PORTAbits.RA2
#define HALL_B      PORTAbits.RA3
#define HALL_C      PORTAbits.RA4

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

