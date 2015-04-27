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
//#define USE_PLL                     // is PLL switched on?

#ifdef USE_PLL
    #define SYS_FREQ        _XTAL_FREQ*4
#else
    #define SYS_FREQ    _XTAL_FREQ
#endif
#define FCY             SYS_FREQ/4

/* ADC condition definitions */
// Microcontroller input voltage
//#define VCC     3.3   // fs <= 10MHz only
#define VCC     5       // Use SPI voltage level shift!
// Current Hall Sensor Offset
#define HALL_U_OFFSET   511

/* Hall rotation sensor mask */
#define HALL_MASK   0b00011100

/* Bootstrap charging enable */
//#define CHARGE_BOOTSTRAPS

/* UART control */
#define UART_CONTROL
#ifdef PLL
    #define BAUD_115200     86
#else
    #define BAUD_115200     21
#endif

/* Current limit */
#define I_MAX   5   // Max current in Amps

/* Data type limits */
// useless? yeah, but better to see where that number came from
#define CHAR_MAX    255
#define INT_MAX     1023


/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

