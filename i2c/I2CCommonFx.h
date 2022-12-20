#ifndef I2CCOMMONFX_H_
#define I2CCOMMONFX_H_

#include "../types.h"
//void I2Ccfx_Write1Byte(byte SLAVE_ADDRESS, byte START_ADDRESS, byte DATA);
//void I2Ccfx_WriteArray(byte SLAVE_ADDRESS, byte START_ADDRESS, volatile byte *pDATA, byte NUMBYTES_TOWRITE);
//void I2Ccfx_ReadRegistersAtAddress(byte SLAVE_ADDRESS, byte START_ADDRESS, volatile byte * pDATA, byte NUMBYTES_TOREAD);


void I2Ccfx_Write1Byte(uint8_t SLAVE_ADDRESS, uint8_t START_ADDRESS, uint8_t DATA);
void I2Ccfx_WriteArray(uint8_t SLAVE_ADDRESS, uint8_t START_ADDRESS, volatile uint8_t *pDATA, uint8_t NUMBYTES_TOWRITE);
void I2Ccfx_ReadRegistersAtAddress(uint8_t SLAVE_ADDRESS, uint8_t START_ADDRESS, volatile uint8_t * pDATA, uint8_t NUMBYTES_TOREAD);

#endif


