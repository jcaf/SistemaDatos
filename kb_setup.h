/*
 * kb_setup.h
 *
 *  Created on: Dec 3, 2022
 *      Author: jcaf
 */

#ifndef KB_SETUP_H_
#define KB_SETUP_H_

#include "ikb/ikb.h"
	//Total Number of keys
	#define KB_NUM_KEYS 1

	void ikb_init(void);

	extern struct _key key[KB_NUM_KEYS];

#endif /* KB_SETUP_H_ */


