/*
 * kb_setup.c
 *
 *  Created on: Dec 3, 2022
 *      Author: jcaf
 */


/*
 * main.c
 *
 *  Created on: Sep 28, 2021
 *      Author: jcaf
 */
#include "system.h"
#include "types.h"
#include "kb_setup.h"
#include "ikb/ikb.h"


//#define iKPAD
#define iKEY

#ifdef iKPAD

	#define KB_NUM_KEYS 16  //Total Number of keys

	/* physical pinout KEYPAD4x4
	PORTWxKBCOL_1 PORTWxKBCOL_2 PORTWxKBCOL_3 PORTWxKBCOL_4
	PORTWxKBFIL_1 	1  2  3  4
	PORTWxKBFIL_2 	5  6  7  8
	PORTWxKBFIL_3 	9  10 11 12
	PORTWxKBFIL_4 	13 14 15 16
	 */
	#define KeyPad4x4_readkey_setupTime() _delay_us(2)

	#define PORTWxKBFIL_0 		PORTD
	#define PORTRxKBFIL_0 		PIND
	#define CONFIGIOxKBFIL_0 	DDRD
	#define PINxKBFIL_0 		2

	#define PORTWxKBFIL_1 		PORTD
	#define PORTRxKBFIL_1 		PIND
	#define CONFIGIOxKBFIL_1 	DDRD
	#define PINxKBFIL_1 		3

	#define PORTWxKBFIL_2 		PORTD
	#define PORTRxKBFIL_2 		PIND
	#define CONFIGIOxKBFIL_2 	DDRD
	#define PINxKBFIL_2 		4

	#define PORTWxKBFIL_3 		PORTD
	#define PORTRxKBFIL_3 		PIND
	#define CONFIGIOxKBFIL_3 	DDRD
	#define PINxKBFIL_3 		5

	#define PORTWxKBCOL_0 		PORTD
	#define PORTRxKBCOL_0 		PIND
	#define CONFIGIOxKBCOL_0 	DDRD
	#define PINxKBCOL_0 		6

	#define PORTWxKBCOL_1 		PORTD
	#define PORTRxKBCOL_1 		PIND
	#define CONFIGIOxKBCOL_1 	DDRD
	#define PINxKBCOL_1		    7

	#define PORTWxKBCOL_2 		PORTB
	#define PORTRxKBCOL_2 		PINB
	#define CONFIGIOxKBCOL_2 	DDRB
	#define PINxKBCOL_2 		0

	#define PORTWxKBCOL_3 		PORTB
	#define PORTRxKBCOL_3 		PINB
	#define CONFIGIOxKBCOL_3 	DDRB
	#define PINxKBCOL_3 		1

	//////////////////////////////////////
	#define KB_LYOUT_KEY_1      0
	#define KB_LYOUT_KEY_2      1
	#define KB_LYOUT_KEY_3      2
	#define KB_LYOUT_KEY_UP     3

	#define KB_LYOUT_KEY_4      4
	#define KB_LYOUT_KEY_5      5
	#define KB_LYOUT_KEY_6      6
	#define KB_LYOUT_KEY_DOWN   7

	#define KB_LYOUT_KEY_7      8
	#define KB_LYOUT_KEY_8      9
	#define KB_LYOUT_KEY_9      10
	#define KB_LYOUT_KEY_2ND    11

	#define KB_LYOUT_KEY_CLEAR  12
	#define KB_LYOUT_KEY_0      13
	#define KB_LYOUT_KEY_MENU   14
	#define KB_LYOUT_KEY_ENTER  15

	static inline uint8_t keyRead_pin_key0(void)
    {
        PinTo0(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY0, PINxKBCOL_KEY0);
    }
    static inline uint8_t keyRead_pin_key1(void)
    {
        PinTo0(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY1, PINxKBCOL_KEY1);
    }
    static inline uint8_t keyRead_pin_key2(void)
    {
        PinTo0(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY2, PINxKBCOL_KEY2);
    }
    static inline uint8_t keyRead_pin_key3(void)
    {
        PinTo0(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY3, PINxKBCOL_KEY3);
    }
    //
    static inline uint8_t keyRead_pin_key4(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo0(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY4, PINxKBCOL_KEY4);
    }
    static inline uint8_t keyRead_pin_key6(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo0(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY5, PINxKBCOL_KEY5);
    }
    static inline uint8_t keyRead_pin_key7(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo0(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY6, PINxKBCOL_KEY6);
    }
    static inline uint8_t keyRead_pin_key8(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo0(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY7, PINxKBCOL_KEY7);
    }
    //
    static inline uint8_t keyRead_pin_key9(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo0(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY8, PINxKBCOL_KEY8);
    }
    static inline uint8_t keyRead_pin_key10(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo0(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY9, PINxKBCOL_KEY9);
    }
    static inline uint8_t keyRead_pin_key11(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo0(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY10, PINxKBCOL_KEY10);
    }
    static inline uint8_t keyRead_pin_key12(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo0(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo1(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY11, PINxKBCOL_KEY11);
    }
    static inline uint8_t keyRead_pin_key13(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo0(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY12, PINxKBCOL_KEY12);
    }
    static inline uint8_t keyRead_pin_key14(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo0(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY13, PINxKBCOL_KEY13);
    }
    static inline uint8_t keyRead_pin_key15(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo0(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY14, PINxKBCOL_KEY14);
    }
    static inline uint8_t keyRead_pin_key16(void)
    {
        PinTo1(PORTWxKBFIL_0, PINxKBFIL_0);
        PinTo1(PORTWxKBFIL_1, PINxKBFIL_1);
        PinTo1(PORTWxKBFIL_2, PINxKBFIL_2);
        PinTo0(PORTWxKBFIL_3, PINxKBFIL_3);
        KeyPad4x4_readkey_setupTime();
        return PinRead(PORTRxKBCOL_KEY15, PINxKBCOL_KEY15);
    }
#endif // iKB

#ifdef iKEY


	#define PORTWxKB_KEY0		PORTB
	#define PORTRxKB_KEY0		PINB
	#define CONFIGIOxKB_KEY0	DDRB
	#define PINxKB_KEY0			2

//	#define PORTWxKB_KEY1		PORTA
//	#define PORTRxKB_KEY1		PINA
//	#define CONFIGIOxKB_KEY1	DDRA
//	#define PINxKB_KEY1			6

    static inline
	uint8_t keyRead_pin_key0(void)
    {
        return PinRead(PORTRxKB_KEY0, PINxKB_KEY0);
    }
//    static inline
//	uint8_t keyRead_pin_key1(void)
//    {
//        return PinRead(PORTRxKB_KEY1, PINxKB_KEY1);
//    }

#endif // iKEY

struct _key key[KB_NUM_KEYS];
const struct _key_prop propEmpty;

//#define KB_PERIODIC_ACCESS_MS 20		//msE-3
//#define KB_KEY_SCAN_COUNT_DEBOUNCE_MS 20

void ikb_init(void)
{
	#ifdef iKPAD
		ConfigOutputPin(CONFIGIOxKBFIL_0, PINxKBFIL_0);
		ConfigOutputPin(CONFIGIOxKBFIL_1, PINxKBFIL_1);
		ConfigOutputPin(CONFIGIOxKBFIL_2, PINxKBFIL_2);
		ConfigOutputPin(CONFIGIOxKBFIL_3, PINxKBFIL_3);

		ConfigInputPin(CONFIGIOxKBCOL_0, PINxKBCOL_0);
		ConfigInputPin(CONFIGIOxKBCOL_1, PINxKBCOL_1);
		ConfigInputPin(CONFIGIOxKBCOL_2, PINxKBCOL_2);
		ConfigInputPin(CONFIGIOxKBCOL_3, PINxKBCOL_3);

		key[0].keyRead = keyRead_pin_key0;
		key[1].keyRead = keyRead_pin_key1;
		key[2].keyRead = keyRead_pin_key2;
		key[3].keyRead = keyRead_pin_key3;
		key[4].keyRead = keyRead_pin_key4;
		key[5].keyRead = keyRead_pin_key6;
		key[6].keyRead = keyRead_pin_key7;
		key[7].keyRead = keyRead_pin_key8;
		key[8].keyRead = keyRead_pin_key9;
		key[9].keyRead = keyRead_pin_key10;
		key[10].keyRead = keyRead_pin_key11;
		key[11].keyRead = keyRead_pin_key12;
		key[12].keyRead = keyRead_pin_key13;
		key[13].keyRead = keyRead_pin_key14;
		key[14].keyRead = keyRead_pin_key15;
		key[15].keyRead = keyRead_pin_key16;

	#endif // iKPAD
	#ifdef iKEY
		/* Activate pull-ups */
		PinTo1(PORTWxKB_KEY0, PINxKB_KEY0);
		//PinTo1(PORTWxKB_KEY1, PINxKB_KEY1);

		__delay_ms(1);
		//
		ConfigInputPin(CONFIGIOxKB_KEY0, PINxKB_KEY0);
		//ConfigInputPin(CONFIGIOxKB_KEY1, PINxKB_KEY1);

		//
		key[0].keyRead = keyRead_pin_key0;
		//key[1].keyRead = keyRead_pin_key1;

	#endif // iKEY

	/*by default init with onKeyPressed = 1*/
	struct _key_prop prop = {0};
	prop = propEmpty;
	prop.uFlag.f.onKeyPressed = 1;
	for (int8_t k=0; k< KB_NUM_KEYS; k++)
	{
		ikb_setKeyProp(key, k, prop);
	}
	key[0].prop.numGroup=1;//existe un bug cuando numGroup = 0
	//key[1].prop.numGroup=1;

}



