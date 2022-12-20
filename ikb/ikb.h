/*
Reglas:
-primero se lee el teclado. el indice empiezo con ellos
*/
#ifndef iKB_H_
#define iKB_H_

struct _key_prop
{
	union _key_prop_uFlag
	{
		struct _key_prop_uFlag_f
		{
			unsigned onKeyPressed:1;//when pressed
			unsigned reptt:1;
			unsigned onKeyReleased:1;//when released
			unsigned whilePressing:1;//new 2019
			unsigned atTimeExpired:1;
			unsigned atTimeExpired2:1;//new mode 2017
			unsigned __a:2;
		}f;

		uint8_t packed;
	}uFlag;

	struct _repttTh
	{
		uint16_t breakTime;     //break this time to enter to repetition
		uint16_t period;        //each time access to repp after the "breakTime"
	}repttTh;

	uint8_t  numGroup;
};

struct _key
{
    PTRFX_retUINT8_T keyRead;
    //PTRFX_retVOID keyDo;
    uint8_t sm0;
    //int8_t k_debounce_ms;
    uint8_t counter0;
    uint8_t sm1;
    uint16_t counter1;

    struct _key_prop prop;

    struct _key_statusFlag
    {
        unsigned state:1;
        unsigned readyToRead:1;
        unsigned atTimeExpired_beforeOrAfter:1;
        //unsigned ownerOfGroup:1;
        unsigned inProcessing:1;
        unsigned lockedInGroup:1;
        unsigned __a:3;
        //unsigned __a:2;
    }statusFlag;
};



    //scan hardware
    #define KB_KEY_PINLEVEL_PRESSED 0
    #define KB_KEY_PINLEVEL_RELEASED 1

	/* estos dos opciones solo se diferencias en que _2 no tiene repiticion...pero igual
	 * podria usarse solo 1 sola kte para todos casos, ya que seria una variable
	 * */
	//
//    #define _FIRE_AT_TIME_THRESHOLD_ (1000.0/KB_PERIODIC_ACCESS)//in ms
//    #define _FIRE_AT_TIME_THRESHOLD2_ (2000.0/KB_PERIODIC_ACCESS)//in ms


	#define KB_BEFORE_THR	0
    #define KB_AFTER_THR	1
    //////////////////////////////////////////////////////////////////////////////////////

	void ikey_scan(struct _key *key, int8_t num_keys, int8_t count_debounce);
	void ikey_parsing(struct _key *key, int8_t num_keys);
    void ikb_flush(struct _key *key, int8_t num_keys);
    //
    uint8_t ikb_get_AtTimeExpired_BeforeOrAfter(struct _key *key, int8_t k);
    uint8_t ikb_key_is_ready2read(struct _key *key, int8_t k);
    void ikb_set_ready2read(struct _key *key, int8_t k);
    void ikb_key_was_read(struct _key *key, int8_t k);
    void ikb_execfunct(struct _key *key, int8_t k);


    //////////////////////////////////////////////////////////////////////////////////////

    void ikb_setKeyProp(struct _key *key, uint8_t k, struct _key_prop prop);
    extern const struct _key_prop propEmpty;


#endif /* IKB_H_ */
