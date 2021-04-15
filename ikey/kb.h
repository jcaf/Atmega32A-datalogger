#ifndef KB_H_
#define KB_H_

    #include "../types.h"

    void key_initialization(void);
    void kb_job(void);
    void kb_flush(void);

    //void kb_change_keyDo(const PTRFX_retVOID const *keyDo);
    //void kb_change_keyDo(PTRFX_retVOID *keyDo);
    //void kb_change_keyDo_pgm(PTRFX_retVOID const *  keyDo);

    void kb_processKeyRead_D(void);


    /*
        PORTWxKBCOL_1 PORTWxKBCOL_2 PORTWxKBCOL_3 PORTWxKBCOL_4
    PORTWxKBFIL_1 	1  2  3  4
    PORTWxKBFIL_2 	5  6  7  8
    PORTWxKBFIL_3 	9  10 11 12
    PORTWxKBFIL_4 	13 14 15 16
     */

    #define PORTWxKBFIL_1 		PORTD
    #define PORTRxKBFIL_1 		PIND
    #define CONFIGIOxKBFIL_1 	DDRD
    #define PINxKBFIL_1 		2

    #define PORTWxKBFIL_2 		PORTB
    #define PORTRxKBFIL_2 		PINB
    #define CONFIGIOxKBFIL_2 	DDRB
    #define PINxKBFIL_2 		4

//    #define PORTWxKBFIL_3 		PORTA
//    #define PORTRxKBFIL_3 		PINA
//    #define CONFIGIOxKBFIL_3 	DDRA
//    #define PINxKBFIL_3 		5
//
//    #define PORTWxKBFIL_4 		PORTA
//    #define PORTRxKBFIL_4 		PINA
//    #define CONFIGIOxKBFIL_4 	DDRA
//    #define PINxKBFIL_4 		4

    #define PORTWxKBCOL_1 		PORTB
    #define PORTRxKBCOL_1 		PINB
    #define CONFIGIOxKBCOL_1 	DDRB
    #define PINxKBCOL_1 		0

    #define PORTWxKBCOL_2 		PORTB
    #define PORTRxKBCOL_2 		PINB
    #define CONFIGIOxKBCOL_2 	DDRB
    #define PINxKBCOL_2		    1

    #define PORTWxKBCOL_3 		PORTB
    #define PORTRxKBCOL_3 		PINB
    #define CONFIGIOxKBCOL_3 	DDRB
    #define PINxKBCOL_3 		2

    #define PORTWxKBCOL_4 		PORTB
    #define PORTRxKBCOL_4 		PINB
    #define CONFIGIOxKBCOL_4 	DDRB
    #define PINxKBCOL_4 		3

    //////////////////////////////////////
    //kb layout
    #include "ikey.h"

    #define KB_LYOUT_KEY_X1     IKEY_POS1
    #define KB_LYOUT_KEY_A      IKEY_POS2
    #define KB_LYOUT_KEY_B      IKEY_POS3
    #define KB_LYOUT_KEY_C      IKEY_POS4

    #define KB_LYOUT_KEY_X2     IKEY_POS5
    #define KB_LYOUT_KEY_X3     IKEY_POS6
    #define KB_LYOUT_KEY_X4     IKEY_POS7
    #define KB_LYOUT_KEY_X5	    IKEY_POS8

//    #define KB_LYOUT_KEY_7      IKEY_POS9
//    #define KB_LYOUT_KEY_8      IKEY_POS10
//    #define KB_LYOUT_KEY_9      IKEY_POS11
//    #define KB_LYOUT_KEY_2ND    IKEY_POS12
//
//    #define KB_LYOUT_KEY_CLEAR  IKEY_POS13
//    #define KB_LYOUT_KEY_0      IKEY_POS14
//    #define KB_LYOUT_KEY_MENU   IKEY_POS15
//    #define KB_LYOUT_KEY_ENTER  IKEY_POS16
#endif
