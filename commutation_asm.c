/*
 * File:   commutation_asm.c
 * Author: D.W.
 *
 * Created: 25.4.2015
 * Project: BLDC driver v3.0
 *
 * Comment: Commutation routines for OVDCOND register setting
 *          These routines cannot be called from interrupt AND anywhere else
 *              because of the compiler label redefinition error!
 *          Possible "unexpected token" errors may be caused by MPLAB X's faulty
 *              interpretation of #asm directives
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>
#include "system.h"     // HALL_MASK definition

/******************************************************************************/
/* Variables                                                                  */
/******************************************************************************/

// Tab of possible rotation hall sensor combinations
unsigned char hall_cur = 0;             // current combination
unsigned char hall_pos1 = 0b00010000;
unsigned char hall_pos2 = 0b00011000;
unsigned char hall_pos3 = 0b00001000;
unsigned char hall_pos4 = 0b00001100;
unsigned char hall_pos5 = 0b00000100;
unsigned char hall_pos6 = 0b00010100;

// Generator switch tab
/* Low side switching combinations */
// Be careful about RB4/PWM5 and RB5/PWM4
unsigned char set_LA = 0b00000001;
unsigned char set_LB = 0b00000100;
unsigned char set_LC = 0b00010000;
unsigned char set_HA = 0b00000010;
unsigned char set_HB = 0b00001000;
unsigned char set_HC = 0b00100000;

extern unsigned char flags_status;
extern unsigned char hall_cur;
extern unsigned char meas_rot_dir;


/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void commutate_mot(void){
    hall_cur = PORTA & HALL_MASK;

#asm

mot_cw:
    btfsc   _flags_status,1  ; Motor mode xx1 ?
        bra mot_ccw         ; reverse

rot1_cw:
    movf    _hall_pos1,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_1
rot2_cw:
    movf    _hall_pos2,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_2
rot3_cw:
    movf    _hall_pos3,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_3
rot4_cw:
    movf    _hall_pos4,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_4
rot5_cw:
    movf    _hall_pos5,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_5
rot6_cw:
    movf    _hall_pos6,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_6
    movlw   0x00     ; error,
    movwf   OVDCOND  ; disable PWM outputs
    movwf   OVDCONS
    return

mot_ccw:

rot1_ccw:
    movf    _hall_pos1,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_4
rot2_ccw:
    movf    _hall_pos2,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_5
rot3_ccw:
    movf    _hall_pos3,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_6
rot4_ccw:
    movf    _hall_pos4,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_1
rot5_ccw:
    movf    _hall_pos5,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_2
rot6_ccw:
    movf    _hall_pos6,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    pos_3

    movlw   0x00        ; error
    movwf   OVDCOND     ; disable PWM outputs
    movwf   OVDCONS
    return
;---------------------------------------------
pos_1:
    movff   _pos1,OVDCOND
    return
pos_2:
    movff   _pos2,OVDCOND
    return
pos_3:
    movff   _pos3,OVDCOND
    return
pos_4:
    movff   _pos4,OVDCOND
    return
pos_5:
    movff   _pos5,OVDCOND
    return
pos_6:
    movff   _pos6,OVDCOND
    return
#endasm

}

void commutate_gen(void) {
    // First try: Simple turning of low side transistors to reduce voltage
    // drop on builtin transistor diodes.

    hall_cur = PORTA & HALL_MASK;

#asm

gen_cw:
btfsc   _meas_rot_dir,1  ; Rotation direction xx1 ?
    bra gen_ccw         ; reverse

gen1:
    movf    _hall_pos1,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_1
gen2:
    movf    _hall_pos2,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_2
gen3:
    movf    _hall_pos3,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_3
gen4:
    movf    _hall_pos4,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_4
gen5:
    movf    _hall_pos5,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_5
gen6:
    movf    _hall_pos6,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_6

    movlw   0x00        ; error
    movwf   LATB        ; disable outputs
    return

gen_ccw:

gen1_ccw:
    movf    _hall_pos1,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_4
gen2_ccw:
    movf    _hall_pos2,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_5
gen3_ccw:
    movf    _hall_pos3,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_6
gen4_ccw:
    movf    _hall_pos4,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_1
gen5_ccw:
    movf    _hall_pos5,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_2
gen6_ccw:
    movf    _hall_pos6,w
    subwf   _hall_cur,w
    btfsc   STATUS,2
        goto    gen_pos_3

movlw   0x00        ; error
movwf   LATB     ; disable PWM outputs
return

;---------------------------------------------
gen_pos_1:
    movff   _set_LB,OVDCONS
    movff   _set_LA,OVDCOND
    return
gen_pos_2:
    movff   _set_LB,OVDCONS
    movff   _set_LC,OVDCOND
    return
gen_pos_3:
    movff   _set_LA,OVDCONS
    movff   _set_LC,OVDCOND
    return
gen_pos_4:
    movff   _set_LA,OVDCONS
    movff   _set_LB,OVDCOND
    return
gen_pos_5:
    movff   _set_LC,OVDCONS
    movff   _set_LB,OVDCOND
    return
gen_pos_6:
    movff   _set_LC,OVDCONS
    movff   _set_LA,OVDCOND
    return
#endasm
}