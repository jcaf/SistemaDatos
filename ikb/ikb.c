/*
 * ikb.c
 *
 *  Created on: Sep 28, 2021
 *      Author: jcaf
 */

#include "../system.h"
#include "../types.h"
#include "ikb.h"

//set by software
#define KB_KEY_STATE_PRESSED 1
#define KB_KEY_STATE_RELEASED 0

#define IKEY_SCAN_SETUP_TIMEuS 5//us

void ikey_scan(struct _key *key, int8_t num_keys, int8_t count_debounce)
{
    int8_t k = num_keys-1;
    do
    {
        __delay_us(IKEY_SCAN_SETUP_TIMEuS);

        if (key[k].sm0 == 0)
        {
            if (key[k].keyRead() == KB_KEY_PINLEVEL_PRESSED)
            {
                key[k].sm0++;
            }
        }
        else if (key[k].sm0 == 1)
        {
            if (++key[k].counter0 == count_debounce)
            {
                key[k].counter0 = 0x00;

                if (key[k].keyRead() != KB_KEY_PINLEVEL_PRESSED)
                {
                    key[k].sm0--;
                }
                else
                {
                    key[k].statusFlag.state = KB_KEY_STATE_PRESSED;
                    key[k].sm0++;
                }
            }
        }
        else if (key[k].sm0 == 2)
        {
            if (key[k].keyRead() == KB_KEY_PINLEVEL_RELEASED)
                key[k].sm0++;
        }
        else if (key[k].sm0 == 3)
        {
            if (++key[k].counter0 == count_debounce)
            {
                key[k].counter0 = 0x00;

                if (key[k].keyRead() != KB_KEY_PINLEVEL_RELEASED)
                {
                    key[k].sm0--;
                }
                else
                {
                    key[k].statusFlag.state = KB_KEY_STATE_RELEASED;
                    key[k].sm0 = 0x00;
                }
            }
        }

    }while (--k >= 0);
}

///////////////////////////////////////////////////////////////////////////
#define NO_GROUP_X 0
//#define AGRUPED 1
//#define GROUP_LOCKED 1
//#define GROUP_UNLOCKED	0

//uint8_t group_x[KB_NUM_KEYS];
//
//static uint8_t ikey_is_accessible(struct _key *key, uint8_t k)
//{
//    if (key[k].prop.numGroup > NO_GROUP_X)	//esta agrupado ?
//    {
//        //aqui fue puesto al presionar la tecla
//        if (group_x[key[k].prop.numGroup] == GROUP_LOCKED)
//        {
//            //a todos sus companeros se les prohibe el ingreso, solo quien fue el primero en ganar el acceso al grupo entra
//            if ( !key[k].statusFlag.ownerOfGroup)	//Si no es quien gano la propiedad...
//            {
//                return 0;
//            }
//        }
//    }
//    return 1;
//}

//uint8_t group_x[KB_NUM_KEYS];

static uint8_t ikey_is_accessible(struct _key *key, uint8_t k)
{
    if (key[k].prop.numGroup > NO_GROUP_X)	//esta agrupado ?
    {
    	return !key[k].statusFlag.lockedInGroup;

    }
    else
    {
    	return 1;
    }
}

///////////////////////////////////////////////////////////////////////////
#define KBAPP_READY2READ	1
#define KBAPP_ALREADYREAD	0

//Stages definition, incremental and contiguous order
enum _STAGE_
{
    _STAGE_START_PARSING_=0,
    _STAGE_ISKEY_RELEASED_,
    _STAGE_ISKEY_RELEASED2_,
    _STAGE_ISKEY_READFROMAPP_,
    _STAGE_FIREATIMEEXPIRED_,
    _STAGE_FIREATIMEEXPIRED_2_,
    _STAGE_ONKEYRELEASED_,
    _STAGE_WHILEPRESSING_,
    //Reptt, _1, _2 deben ser contiguos!
    _STAGE_REPTT_,
    _STAGE_REPTT_1_,
    _STAGE_REPTT_2_,
    //
    _STAGE_END_PARSING_,
};

void ikey_parsing(struct _key *key, int8_t num_keys)
{
    uint8_t k;
    int8_t numGroup;

    for (k=0; k < num_keys; k++)
    {
        if ( key[k].sm1 == _STAGE_START_PARSING_ )
        {
            if ( ikey_is_accessible(key, k) )
			//if (1)
            {
                if (key[k].statusFlag.state == KB_KEY_STATE_PRESSED)
                {
                    //........................................................
                    if (key[k].prop.uFlag.f.onKeyPressed)
                    {
                        key[k].statusFlag.readyToRead = KBAPP_READY2READ;	//Populate Key to app.

                        if (key[k].prop.uFlag.f.reptt)
                        {
                            key[k].sm1 = _STAGE_REPTT_;
                        }
                        else
                        {
                            key[k].sm1 = _STAGE_ISKEY_RELEASED_;
                        }
                    }
                    else if ( key[k].prop.uFlag.f.onKeyReleased)
                    {
                        key[k].sm1 = _STAGE_ONKEYRELEASED_;
                    }
                    else if ( key[k].prop.uFlag.f.whilePressing)
                    {
                        key[k].sm1 = _STAGE_WHILEPRESSING_;
                    }
                    else if (key[k].prop.uFlag.f.atTimeExpired)
                    {
                        key[k].sm1 = _STAGE_FIREATIMEEXPIRED_;
                    }
                    else if (key[k].prop.uFlag.f.atTimeExpired2)
                    {
                        key[k].sm1 = _STAGE_FIREATIMEEXPIRED_2_;
                    }

                    //Add++
                    key[k].statusFlag.inProcessing = 1;
                    //++

                    //........................................................

//                    if (key[k].prop.numGroup > NO_GROUP_X)
//                    {
//                        group_x[ key[k].prop.numGroup ] = GROUP_LOCKED;
//                        key[k].statusFlag.ownerOfGroup = 1;
//                    }
                    numGroup = key[k].prop.numGroup;
                    if ( numGroup > NO_GROUP_X)
					{
                    	//bloquear a los restantes de su grupo
                    	for (int8_t  i=0; i<num_keys; i++)
                    	{
                    	    if (key[i].prop.numGroup == numGroup)
                    	    {
                    	    	/* Todos los pertencientes a ese grupo, excepto el actual, quedan bloqueados */
                    	    	if (i != k)
                    	    	{
                    	    		key[i].statusFlag.lockedInGroup = 1;//queda bloqueado
                    	    	}
                    	    }
                    	}
					}
                    //........................................................
                }
            }
        }

        //_________________________________________________________________________________________
        //comprueba entre las pasadas de repeticion/no repeticion si se ha soltado la tecla
        if ( (key[k].sm1 >= _STAGE_REPTT_ ) && (key[k].sm1 <= _STAGE_REPTT_2_))
        {
            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED)			//ya solto?
            {
                key[k].counter1 = 0;				//stop timmer
                key[k].sm1 = _STAGE_ISKEY_READFROMAPP_;
            }
        }
        //.............................................................................
        if (key[k].sm1 == _STAGE_REPTT_ )
        {
            if ( ++key[k].counter1 >= key[k].prop.repttTh.breakTime)
            {
                key[k].counter1 =0;
                key[k].sm1 = _STAGE_REPTT_1_;
            }
        }
        if (key[k].sm1 == _STAGE_REPTT_1_)	//ping-pong between 1<->2
        {
            if (key[k].statusFlag.readyToRead == KBAPP_ALREADYREAD)			//ya leyo?
            {
                key[k].statusFlag.readyToRead = KBAPP_READY2READ;				//Repopulate again
                key[k].sm1 = _STAGE_REPTT_2_;
            }
        }
        if (key[k].sm1 == _STAGE_REPTT_2_)	//ping-pong 2<->1
        {
            if ( ++key[k].counter1 >= key[k].prop.repttTh.period)
            {
                key[k].counter1 = 0;
                key[k].sm1 = _STAGE_REPTT_1_;
            }
        }
        //_________________________________________________________________________________________
        if (key[k].sm1 == _STAGE_ONKEYRELEASED_ )
        {
            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED)	//ya solto?
            {
                key[k].statusFlag.readyToRead = KBAPP_READY2READ;							//Populate Key to app.
                key[k].sm1 = _STAGE_ISKEY_READFROMAPP_;		//esperar x leer
            }
        }
        if (key[k].sm1 == _STAGE_WHILEPRESSING_ )   //added 2019
        {
            key[k].statusFlag.readyToRead = KBAPP_READY2READ;	//Populate Key to app.
            key[k].sm1 = _STAGE_ISKEY_RELEASED2_;
        }
        //_________________________________________________________________________________________
//        if (key[k].sm1 == _STAGE_FIREATIMEEXPIRED_ )
//        {
//            key[k].counter1++;
//
//            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED)	//ya solto?
//            {
//                if (key[k].counter1 <= (_FIRE_AT_TIME_THRESHOLD_) )	//USANDO UN SOLO CONTADOR,
//                {
//                    key[k].counter1	= 0;							//PUES SE USA EN TIEMPOS DIFERENTES
//
//                    key[k].statusFlag.readyToRead = KBAPP_READY2READ;
//                    key[k].statusFlag.atTimeExpired_beforeOrAfter = KB_BEFORE_THR;
//
//                    key[k].sm1 = _STAGE_ISKEY_READFROMAPP_;		//Go directly to wait for read key from high-level application
//                }
//            }
//
//            if (key[k].counter1 > (_FIRE_AT_TIME_THRESHOLD_) )
//            {
//                key[k].counter1 = 0x0000;
//
//                key[k].statusFlag.readyToRead = KBAPP_READY2READ;
//                key[k].statusFlag.atTimeExpired_beforeOrAfter = KB_AFTER_THR;	//queda senializado
//
//                if (!key[k].prop.uFlag.f.reptt)							//x No Reptt
//                {
//                    key[k].sm1 = _STAGE_ISKEY_RELEASED_;   //wait for key is release
//                }
//                else
//                {
//                    key[k].sm1 = _STAGE_REPTT_2_;
//                }
//            }
//        }
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //NO SE ADMITE REPETICION
        //.........|thr|....................................|thr2|
        //before       | no se reconoce como pulsacion      |after si cumplio este tiempo, se reconoce como la segunda pulsacion
//        if (key[k].sm1 == _STAGE_FIREATIMEEXPIRED_2_)//nueva opcion 2017
//        {
//            key[k].counter1++;
//
//            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED) //ya solto?
//            {
//                if (key[k].counter1 <= (_FIRE_AT_TIME_THRESHOLD_)) //primer threshold
//                {
//                    key[k].counter1 = 0; //PUES SE USA EN TIEMPOS DIFERENTES
//
//                    key[k].statusFlag.readyToRead = KBAPP_READY2READ;
//                    key[k].statusFlag.atTimeExpired_beforeOrAfter = KB_BEFORE_THR;
//
//                    key[k].sm1 = _STAGE_ISKEY_READFROMAPP_; //Go directly to wait for read key from high-level application
//                }
//            }
//            /* PODRIA SER OTRA OPCION EN DONDE SI SE RECONOCE ANTES DEL THRESHOLD2 COMO PULSACION*/
//
//            //#define _FIRE_AT_TIME_THRESHOLD2_ (3000E-3/KB_PERIODIC_ACCESS)
//            if (key[k].counter1 >= (_FIRE_AT_TIME_THRESHOLD2_))
//            {
//                key[k].counter1 = 0x0000;
//
//                key[k].statusFlag.readyToRead = KBAPP_READY2READ;
//                key[k].statusFlag.atTimeExpired_beforeOrAfter = KB_AFTER_THR; //queda senializado
//
//                //NO SE ADMITE REPETICION
//                key[k].sm1 = _STAGE_ISKEY_RELEASED2_; //wait for key is release
//            }
//        }
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //_________________________________________________________________________________________
        if (key[k].sm1 == _STAGE_ISKEY_RELEASED_)		//MODO->FireOnKeyPressed + no Repetitibilidad 	//MODO->solo esperar que suelte
        {
            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED)						//ya solto?
            {
                key[k].sm1 = _STAGE_ISKEY_READFROMAPP_;   //esperar x leer
            }
        }
        //_________________________________________________________________________________________
        if (key[k].sm1 == _STAGE_ISKEY_RELEASED2_)		//added 2019
        {
            if (key[k].statusFlag.state == KB_KEY_STATE_RELEASED)
            {
                key[k].statusFlag.readyToRead = KBAPP_ALREADYREAD;
                key[k].sm1 = _STAGE_END_PARSING_;
            }
        }
        //_________________________________________________________________________________________
        if (key[k].sm1 == _STAGE_ISKEY_READFROMAPP_)
        {
            if (key[k].statusFlag.readyToRead == KBAPP_ALREADYREAD)	//ya leyo? //added 2019: || (terminate_key) la app. mata el proceso...
            {
                key[k].sm1 = _STAGE_END_PARSING_;
            }
        }
        //_________________________________________________________________________________________
        if (key[k].sm1 == _STAGE_END_PARSING_)
        {
        	numGroup = key[k].prop.numGroup;
            if (numGroup > NO_GROUP_X)	//forma grupo?
            {
                //group_x[key[k].prop.numGroup] = GROUP_UNLOCKED;
                //key[k].statusFlag.ownerOfGroup = 0;

            	/* option 1*/
				/* clear all readyToRead flag
				 * No es necesario limpiar todos los flags xq igual al haber
				 * entrado a la condición, al resto se bloqueo, y nunca llegó
				 * a poner su flag readyToRead a 1
				 * */
				//key[i].statusFlag.readyToRead = 0;

				/* option 2 */
				/*
				 * Controlar por hardware que ya a soltado las teclas q conforman
				 * el grupo
				 */
            	int8_t keysGroupedArentPressing = 1;


            	for (int8_t i=0; i<num_keys; i++)
                {
                	if (key[i].prop.numGroup == numGroup)
                	{
                		if (key[i].statusFlag.state != KB_KEY_STATE_PRESSED)
                		{
                			key[i].statusFlag.lockedInGroup = 0;
                		}
                		else
                		{
                			keysGroupedArentPressing = 0;
                		}
                	}
                }
            	if (keysGroupedArentPressing == 1)
				{
					key[k].sm1 = _STAGE_START_PARSING_;
				}
            }

            /* Si quedaría en este punto sm1=0, significa q es la Option 1*/
            //key[k].sm1 = _STAGE_START_PARSING_;

            //Add++
            key[k].statusFlag.inProcessing = 0;
            //++
        }
    }
}
int8_t ikey_is_all_hw_released(struct _key *key, int8_t KB_NUM_KEYS)//hardware release
{
    for (int8_t i=0; i<KB_NUM_KEYS; i++)
    {
        if (key[i].statusFlag.state == KB_KEY_STATE_PRESSED)
        {
            return 0;
        }
    }
    return 1;
}

void ikey_clear_all_flag(struct _key *key, int8_t num_keys)//clear ReadyToRead
{
    for (int8_t i=0; i<num_keys; i++)
    {
        //if (key[i].statusFlag.ReadyToRead)
            key[i].statusFlag.readyToRead = 0;
    }
}

int8_t ikey_is_all_process_finished(struct _key *key, int8_t num_keys)
{
    for (int8_t i=0; i<num_keys; i++)
    {
        if (key[i].statusFlag.inProcessing == 1)
        {
            return 0;
        }
    }
    return 1;
}


void ikb_setKeyProp(struct _key *key, uint8_t k, struct _key_prop prop)
{
    key[k].prop = prop;
}

//void ikb_job(struct _key *key, int8_t KB_NUM_KEYS)
//{
//    ikey_scan(key, KB_NUM_KEYS);
//    ikey_parsing(key, KB_NUM_KEYS);
//}
void ikb_flush(struct _key *key, int8_t num_keys)
{
    ikey_clear_all_flag(key, num_keys);
}
uint8_t ikb_get_AtTimeExpired_BeforeOrAfter(struct _key *key, int8_t k)
{
    return key[k].statusFlag.atTimeExpired_beforeOrAfter;
}
uint8_t ikb_key_is_ready2read(struct _key *key, int8_t k)
{
    return key[k].statusFlag.readyToRead;
}
void ikb_set_ready2read(struct _key *key, int8_t k)
{
    key[k].statusFlag.readyToRead = 1;
}
void ikb_key_was_read(struct _key *key, int8_t k)
{
    key[k].statusFlag.readyToRead = 0;
}
//void ikb_execfunct(struct _key *key, int8_t k)
//{
//    key[k].keyDo();
//}
