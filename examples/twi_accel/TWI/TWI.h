/*
*
*	Component name:	TWI
*
*	File name: TWI.h
*
* Description: Two Wire Interface (I2C) public interface declarations.
*
*/

void vTWI_Init		(void);
void vTWI_Write		(uint8_t u8address, uint8_t   u8data);
void vTWI_Read		(uint8_t u8subAdd,  uint8_t *pu8data);
