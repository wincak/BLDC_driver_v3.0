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

extern unsigned char flags_status;
extern unsigned char hall_cur;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void commutate_mot(void){
    hall_cur = PORTA & HALL_MASK;

#asm

com_cw:
    btfsc   _flags_status,1  ; Motor mode xx1 ?
        bra com_rev         ; reverse

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

com_rev:

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

void commutate_gen(void){
    // TODO Regen braking commutation routine
    asm("nop");
}