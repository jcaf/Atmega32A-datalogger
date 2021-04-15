/*
 * pinGetLevel.h
 *
 *  Created on: Dec 3, 2020
 *      Author: jcaf
 */

#ifndef PINGETLEVEL_PINGETLEVEL_H_
#define PINGETLEVEL_PINGETLEVEL_H_

#define PINGETLEVEL_NUMMAX 2//1 //# of pines to check


struct _pinGetLevel
{
  	int8_t counterDebounce;
    struct _pinGetLevel_bf
    {
		unsigned sm0:1;
		unsigned level:1;
		unsigned level_last:1;
		unsigned changed:1;
		unsigned __a:4;
  	}bf;

    //
    PTRFX_retUINT8_T readPinLevel;
};

extern struct _pinGetLevel pinGetLevel[PINGETLEVEL_NUMMAX];

//
#define PINGETLEVEL_INITwCHANGED
void pinGetLevel_init();//by default always changed-flag = 1 at begin
//
void pinGetLevel_job(void);

#define pinGetLevel_hasChanged(i)pinGetLevel[i].bf.changed
#define pinGetLevel_setChange(i) do{pinGetLevel[i].bf.changed = 1;}while(0)
#define pinGetLevel_clearChange(i) do{pinGetLevel[i].bf.changed = 0;}while(0)
#define pinGetLevel_level(i) pinGetLevel[i].bf.level


//REMAPING DEFINITIOS PORTW/R
#include "../main.h"

//remapping Switch 0
#define PORTWxGETLEVEL_0 	PORTWxTACTSW_P1
#define PORTRxGETLEVEL_0 	PORTRxTACTSW_P1
#define CONFIGIOxGETLEVEL_0 	CONFIGIOxTACTSW_P1
#define PINxGETLEVEL_0		PINxTACTSW_P1
//
#define PGLEVEL_LYOUT_P1 0

//remapping Switch 1
#define PORTWxGETLEVEL_1 	PORTWxTACTSW_P2
#define PORTRxGETLEVEL_1 	PORTRxTACTSW_P2
#define CONFIGIOxGETLEVEL_1 	CONFIGIOxTACTSW_P2
#define PINxGETLEVEL_1		PINxTACTSW_P2
//
#define PGLEVEL_LYOUT_P2 1


#endif /* PINGETLEVEL_PINGETLEVEL_H_ */
