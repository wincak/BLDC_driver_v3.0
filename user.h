/*
 * File:   user.h
 * Author: D.W.
 *
 * Created: 21.4.2015
 * Project: BLDC driver v3.0
 */

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

// bitwise operation definition
// see http://www.microchip.com/forums/m688196-p2.aspx
// x - register/variable; n - bit number
#define clrbit(x,n)  ((x) &= ~(1UL<<(n)))
#define setbit(x,n)  ((x) |= 1UL<<(n))
#define flipbit(x,n) ((x) ^= 1UL<<(n))
#define testbit(x,n) (!!((x)&1UL<<(n)))     // check optimalization

// FLAGS_STATUS
// TODO: Getting sick of this rigid code. Change to bit variables? Struct?
// definition of bits in flags_status byte
/*
 * | MSB |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | LSB |
 * |-----|-------|-------|-------|-------|-------|-------|-------|-------|-----|
 * | bit |ONLINE | FLT_I | FLT_T | FLT_U |COMM_ER|      Motor mode       |     |
 * |---------------------------------------------------------------------------|
 * ONLINE:      1 = ONLINE/OK, 0 = OFFLINE
 * FLT_I:       0 = OK, 1 = Overcurrent condition
 * FLT_T:       0 = OK, 1 = Max. temp exceeded
 * FLT_U:       0 = OK, 1 = Battery voltage error
 * COMM_ER:     0 = OK, 1 = Communication error
 * Motor mode:  000 = Motor off
 *              001 = Motor clockwise
 *              010 = Motor counter-clockwise
 *              101 = Generator clockwise
 *              110 = Generator counter-clockwise
 *              111 = Dynamic brake (not implemented)
 */
#define online      7
#define FLT_I       6
#define FLT_T       5
#define FLT_U       4
#define comm_error  3
#define motor_mode  (flags_status & 0b00000111)
#define flags_error (flags_status & 0b01111000)

// Status structure
typedef struct {
    unsigned int current;
    unsigned char motor_temp;
    unsigned char transistor_temp;
    unsigned char batt_voltage;
    int velocity;
} struct_status;

// Motor mode definitions
#define mode_free_run   0b000
#define mode_motor_CW   0b001
#define mode_motor_CCW  0b010
#define mode_regen_CW   0b101
#define mode_regen_CCW  0b110
#define mode_brake      0b111
#define CW  1
#define CCW 2
#define mode_regen      0b100   // must check_direction first

// EXT pins definition and control
#define IO_EXT_TRIS     TRISDbits.RD5
#define IO_EXT_PORT     PORTDbits.RD5
#define FLT_EXT_TRIS    TRISDbits.RD4
#define FLT_EXT_PORT    PORTDbits.RD4

// Debugging outputs
//#define DEBUG_STATUS    // UART system status messages
#define DEBUG_PINS      // turn on debugging output pins
//#define DEBUG_PID     // UART PID messages
//#define DEBUG_TRANSISTOR_TEMP     // UART trans. temperature messages
//#define DEBUG_MOTOR_TEMP          // UART motor temperature messages
//#define DEBUG_BATT_VOLTAGE        // UART battery voltage messages
//#define DEBUG_VELOCITY
#define INSTANT_START

// SPI
#define TX_tab_size     9   // 8 data + 1 null terminator
#define TX_DTC              0
#define TX_CURR_REQ         1
#define TX_H_CURRENT        2
#define TX_L_CURRENT        3
#define TX_TRANSISTOR_TEMP  4
#define TX_MOTOR_TEMP       5
#define TX_BATT_VOLTAGE     6
#define TX_STATUS_BYTE      7

#define RX_tab_size     2   // 2 data
#define RX_CURRENT_REQ      0
#define RX_MOTOR_MODE       1

#define SPI_free_run    0
#define SPI_CW          0x08
#define SPI_CCW         0x01
#define SPI_regen       0x11    // change this!
#define SPI_brake       0x12    // this too!

// ADC
#define ADC_tab_size            8
#define ADC_H_CURRENT           0
#define ADC_L_CURRENT           1
#define ADC_H_MOTOR_TEMP        2
#define ADC_H_TRANSISTOR_TEMP   4
#define ADC_H_BATT_VOLTAGE      6

// UART
#define REQ_I_STEP  10      // I-current control
#define REQ_I_MAX   100
#define REQ_I_MIN   0
#define REQ_V_STEP  100     // V-Velocity control
#define REQ_V_MAX   5000

// PID without PhD
typedef struct
{
  short long dState;      	// Last position input
  short long iState;      	// Integrator state
  short long iMax, iMin;  	// Maximum and minimum allowable integrator state

  int   pGain,    	// proportional gain
        iGain,    	// integral gain
        dGain;     	// derivative gain
} SPid;

/******************************************************************************/
/* User Variables initialization                                              */
/******************************************************************************/

extern unsigned char RX_tab[RX_tab_size];   /* defined in main.c */
/* RX data format
 * 0: [xxxx xxxx] - Current request
 * 1: [xxxx xxxx] - Req_motor_mode
 */

extern unsigned char TX_tab[TX_tab_size];   /* defined in main.c */
/* TX data format
 * 0: [xxxx xxxx] - Dutycycle (0-100 ~ 0%-100%)
 * 1: [xxxx xxxx] - Current_req
 * 2: [---- --xx] - H-Current (right justified!)
 * 3: [xxxx xxxx] - L-Current
 * 4: [xxxx xxxx] - Transistor_temp
 * 5: [xxxx xxxx] - Motor_temp
 * 6: [xxxx xxxx] - Batt_voltage
 * 7: [xxxx xxxx] - Status_byte
 * 8: [---- ----] - Null terminator "\0"
 */

extern unsigned char ADC_tab[8];    /* defined in main.c */
/* A/D conversion result tab format
 * 0: [xxxx xxxx] - H-Current
 * 1: [xx-- ----] - L-Current
 * 2: [xxxx xxxx] - H-Motor temp.
 * 3: [xx-- ----] - L-Motor temp (discarded)
 * 4: [xxxx xxxx] - H-Transistor temp.
 * 5: [xx-- ----] - L-Transistor temp. (discarded)
 * 6: [xxxx xxxx] - H-Batt voltage
 * 7: [xx-- ----] - L-Batt voltage (discarded)
 */

/******************  COMMUTATION***********************************************/
/* Commutation tab
 *
 * Forward motion (TODO: CW/CCW?) tab
 * POS | 1 | 2 | 3 | 4 | 5 | 6 |
 * ----|-----------------------|
 *  A  | 0 | 0 | 0 | 1 | 1 | 1 |
 *  B  | 0 | 1 | 1 | 1 | 0 | 0 |
 *  C  | 1 | 1 | 0 | 0 | 0 | 1 |
 * ----|=======================|
 * A-H | 1 | 0 | 0 | 0 | 0 | 1 |
 * A-L | 0 | 0 | 1 | 1 | 0 | 0 |
 * B-H | 0 | 0 | 0 | 1 | 1 | 0 |
 * B-L | 1 | 1 | 0 | 0 | 0 | 0 |
 * C-H | 0 | 1 | 1 | 0 | 0 | 0 |
 * C-L | 0 | 0 | 0 | 0 | 1 | 1 |
 * Reverse: switch L -> H, H -> L
 */
// OVDCOND commutation table
extern unsigned char pos1;      // PWM7 & PWM0 active
extern unsigned char pos2;      // PWM7 & PWM4 active
extern unsigned char pos3;      // PWM4 & PWM1 active
extern unsigned char pos4;      // PWM2 & PWM1 active
extern unsigned char pos5;      // PWM5 & PWM2 active
extern unsigned char pos6;      // PWM5 & PWM0 active


/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

void InitApp(void);         /* I/O and Peripheral Initialization */
void TabGen(void);          /* Calculate constants */

// SPI
unsigned char Receive_SPI_data(unsigned char length);
unsigned char Transmit_SPI_data(unsigned char length);
void SPI_request_update (void);

// ADC
void calc_ADC_data (void);

// Condition check
char limits_check(void);
void check_direction(void);

// Dutycycle
void set_dutycycle (unsigned int dtc);

// Velocity
int calc_velocity(unsigned int transition_count);

// PID
//int PIDmain(int error);
short long UpdatePID(SPid * pid, int error, int position);

// Commutation asm routines
void commutate_mot();   // TODO
void commutate_gen();   // TODO

void motor_init(unsigned char direction);
void regen_init(unsigned char direction);
void free_run_init();
void motor_halt();  // stop motor on error

// SPI
unsigned char ReadSPI_mod(void);

// Debug
#ifdef DEBUG_STATUS
void debug_status(void);
#endif
