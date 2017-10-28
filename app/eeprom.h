#ifndef  _EEPROM_H_

#define  _EEPROM_H_

#include "stm32f10x.h"
#include "_ansi.h"
#include "math.h"

_BEGIN_STD_C

/*
 * PIN PB_9 SDA
 * PIN PB_8 SCL
 *
*/

// Definition of pin used as SDA.
#define SDA GPIO_Pin_9

//Definition of pin used as SCL.
#define SCL GPIO_Pin_8

// Define slave address EEPROM
// ----------------------------------------
// 0x59 - LC
// 0x5A - LM
// 0x5C - B
// 0x5D - C
// 0x5E - M
// 0x5F - Y
// --------------------------------------

/* Definition of 7 bit slave address. */
#define SLAVE_ADDRESS 0x5A

#define INITIALIZE_SDA() (GPIOB->CRH &= ~GPIO_CRH_MODE9)

/*
 * PIN PB_9 SDA
 * PIN PB_8 SCL
 *
*/
#define READ_SCL() ((GPIOB->IDR & SCL)?1:0) // Read SCL
#define READ_SDA() ((GPIOB->IDR & SDA)?1:0) // Read SDA

#define SETSDA()	(GPIOB->CRH &= ~GPIO_CRH_MODE9)
#define CLRSDA()	(GPIOB->CRH |= GPIO_CRH_MODE9)

#define SETSCL()	(GPIOB->CRH &= ~GPIO_CRH_MODE8)
#define CLRSCL()	(GPIOB->CRH |= GPIO_CRH_MODE8)

#define LED_ON()	(GPIOC->BRR = GPIO_Pin_13)
#define LED_OFF()	(GPIOC->BSRR = GPIO_Pin_13)

#define DISABLE_TWI_INTERRUPT() (EXTI->IMR &= ~(EXTI_IMR_MR9))	// Disables SDA interrupt.
#define ENABLE_TWI_INTERRUPT() 	(EXTI->IMR |= EXTI_IMR_MR9)		// Enables SDA interrupt.


// Name macros for twi state machine
#define TWI_SLA_REQ_W_ACK_RTD              0x60    // Write to eeprom
#define TWI_SLA_DATA_RCV_ACK_RTD           0x80
#define TWI_SLA_DATA_RCV_NACK_RTD          0x88

#define TWI_SLA_REQ_R_ACK_RTD              0xA8    // Read from eeprom
#define TWI_SLA_DATA_SND_ACK_RCV           0xB8
#define TWI_SLA_DATA_SND_NACK_RCV          0xC0
#define TWI_SLA_LAST_DATA_SND_ACK_RCV      0xC8

#define TWI_SLA_REPEAT_START               0xA0  // repeat start
#define TWI_SLA_STOP                       0x68  // stop
#define I2C_IDLE                           0x00  // idle

void GPIO_setup(void);
void INTERRUPT_setup(void);
void Delay_ms(uint32_t ms);

void twi_slave_init(void);
void twi_slave_enable(void);
void twi_slave_disable(void);
unsigned char readI2Cslavebyte(void);
unsigned char readMemAddress(void);
void GetStartCondition(void);
void TWI_state_machine(void);
void receivedata(void);
void senddata(void);

// Dedicated general purpose registers.
register unsigned char TWSR asm("r2");;
register unsigned char TWDR asm("r3");;
register unsigned char TWEA asm("r4");;

/* EEPROM data */
unsigned char Buffer[128] = {
  0x0A, 0x9A, 0x23, 0x85, 0x4F, 0xAA, 0x5D, 0x33, 0x97, 0x2A, 0x65, 0xCC, 0x2B, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x26, 0xA5, 0x93, 0xB2, 0x60, 0x1C, 0x17,
  0x81, 0x81, 0x58, 0x40, 0x69, 0x29, 0x10, 0x37, 0xB1, 0x90, 0x46, 0x04, 0x8A, 0x0A, 0x21, 0x3F,
  0x24, 0x32, 0x6C, 0x64, 0xE0, 0x00, 0x60, 0x5A, 0x14, 0xDD, 0xF9, 0x0E, 0x6D, 0x8E, 0xE5, 0xD1,
  0x42, 0x10, 0x7F, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x92, 0x00, 0x00, 0x1E, 0x00, 0x0C, 0x00,
  0x99, 0x80, 0x1A, 0x46, 0x9F, 0xAA, 0x02, 0x5D, 0x1C, 0x70, 0x53, 0x4E, 0xF1, 0x21, 0x67, 0x01,
  0x40, 0x00, 0x10, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0x5E, 0x02, 0x0C, 0x00
};

_END_STD_C

#endif /* _EEPROM_H_ */
