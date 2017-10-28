#include "eeprom.h"

/* Глобальные переменные */
unsigned char incomingBuffer[8];	// Входящий буфер
unsigned char outgoingBuffer[6] = {0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF};	// Выходной буфер для тестирования
unsigned char walker = 0;	// Указатель ячейки для чтения или записи
unsigned char offset = 0;	// Смещение в массиве

// Обработчик прерывания
void EXTI9_5_IRQHandler(void)
{
	//volatile unsigned char retval;

	if(TWSR == I2C_IDLE)
	{
		GetStartCondition();
	}


	// Call the TWi state machine
	TWI_state_machine();
	ENABLE_TWI_INTERRUPT();

	// Сбрасываем флаг прерываний
	EXTI->PR |= EXTI_PR_PR9;
}

int main(void)
{
	// подключаем порт GPIOB к периферийной шине APB2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// подключаем порт GPIOC к периферийной шине APB2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Enable AFIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_setup();
	INTERRUPT_setup();

	__enable_irq();

	twi_slave_init();
	twi_slave_enable();

    while(1)
    {

    }
}

// GPIO setup
void GPIO_setup(void)
{
	// ------ PIN PB_9 SDA --------------
	// настраиваем вывод PB9 на вход
	GPIOB->CRH &= ~GPIO_CRH_MODE9;	// MODEy[1:0] = 00 Input mode (reset state)
	GPIOB->CRH &= ~GPIO_CRH_CNF9;
	GPIOB->CRH |= GPIO_CRH_CNF9_0;	// CNFy[1:0] = 01 Floating input (reset state)

	// ------ PIN PB_8 SCL --------------
	// настраиваем вывод PB8 на вход
	GPIOB->CRH &= ~GPIO_CRH_MODE8;	// MODEy[1:0] = 00 Input mode (reset state)
	GPIOB->CRH &= ~GPIO_CRH_CNF8;
	GPIOB->CRH |= GPIO_CRH_CNF8_0;	// CNFy[1:0] = 01 Floating input (reset state)

	// ------ PIN PC_13 LED --------------
	// настраиваем вывод PC13 на выход
	GPIOC->CRH |= GPIO_CRH_MODE13;	// MODEy[1:0] = 00 Input mode (reset state)
	GPIOC->CRH &= ~GPIO_CRH_CNF13;	// CNFy[1:0] = 00: General purpose output push-pull
}

// Interrupt setup
// PB9 SDA line
void INTERRUPT_setup(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	// ------------------------------------------
	// Инициализация прерываня
	// ------------------------------------------
	// Connect EXTI9 Line to PB9 pin
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	/* Configure EXTI8 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI9_5 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

void twi_slave_init(void)
{
	INITIALIZE_SDA();

	TWEA = 1;
	TWSR = I2C_IDLE;
}

/* enable twi slave
 */
void twi_slave_enable(void)
{
	ENABLE_TWI_INTERRUPT();
}

void twi_slave_disable(void)
{
	DISABLE_TWI_INTERRUPT();
}

/* Read the slave byte after start condition */
inline unsigned char readI2Cslavebyte(void)
{
	unsigned char index = 0;
	unsigned char val = 0;
	unsigned char cPin = 0;

	// Let SCL go low first. MCU comes here while SCL is still high
	while(READ_SCL());

	// read 8 bits from master, respond with ACK.
	// SCL could be high or low depending on CPU speed
	for(index = 0; index < 8; index++)
	{
		while(!READ_SCL());
		cPin = READ_SDA();

		val = (val << 1) | cPin;
		while(READ_SCL())
		{
			// if SDA changes while SCL is high, it indicates STOP or START
			if((val & 1) != cPin)
			{
				if(READ_SDA())
					TWSR = TWI_SLA_STOP;
				else
					TWSR = TWI_SLA_REPEAT_START;
				return 0;
				// return READ_SDA()? I2C_STOP_DETECTED:I2C_START_DETECTED;
			}
			else
			{
				cPin = READ_SDA();
			}
		}
	}

	// Send ACK, SCL is low now
	if((val & 0xFE) == (SLAVE_ADDRESS << 1))
	{
		CLRSDA();
		while(!READ_SCL());
		while(READ_SCL());
		SETSDA();
		//CLRSCL(); //!!!clock_stretching start
	}
	else
	{
		TWSR = I2C_IDLE;
		return 0;
	}

	return val;
}

// После адреса устройства читается адрес ячейки памяти
unsigned char readMemAddress(void)
{
	unsigned char index = 0;
	unsigned char val = 0;
	unsigned char cPin = 0;

	// Let SCL go low first. MCU comes here while SCL is still high
	while(READ_SCL());

	// read 8 bits from master, respond with ACK.
	// SCL could be high or low depending on CPU speed
	for(index = 0; index < 8; index++)
	{
		while(!READ_SCL());
		cPin = READ_SDA();

		val = (val << 1) | cPin;
		while(READ_SCL())
		{
			// if SDA changes while SCL is high, it indicates STOP or START
			if((val & 1) != cPin)
			{
				if(READ_SDA())
					TWSR = TWI_SLA_STOP;
				else
					TWSR = TWI_SLA_REPEAT_START;
				return 0;
				// return READ_SDA()? I2C_STOP_DETECTED:I2C_START_DETECTED;
			}
			else
			{
				cPin = READ_SDA();
			}
		}
	}

	// Send ACK, SCL is low now
	CLRSDA();
	while(!READ_SCL());
	while(READ_SCL());
	SETSDA();
	//CLRSCL(); //!!!clock_stretching start

	return val;
}

/* Identify start condition */
inline void GetStartCondition(void)
{
	unsigned char retval = 0;

	// Make sure it is the start by checking SCL high when SDA goes low
	if(READ_SCL())
	{
		DISABLE_TWI_INTERRUPT();
	}
	// false trigger; exit the ISR
	else {
		ENABLE_TWI_INTERRUPT();
		return;
	}

	// lop for one or several start conditions before a STOP
	if(TWSR == I2C_IDLE || TWSR == TWI_SLA_REPEAT_START)
	{
		retval = readI2Cslavebyte();	// read address
		if(retval == 0)	// STOP or otehr address received
		{
			TWSR = I2C_IDLE;
			ENABLE_TWI_INTERRUPT();
			return;
		}
		else
		{
			if(retval & 1)
				TWSR = TWI_SLA_REQ_R_ACK_RTD;
			else
				TWSR = TWI_SLA_REQ_W_ACK_RTD;

			// Всегда после адреса устройства на шине i2c
			// следует адрес ячейки памяти
			// Вставить код приема адреса ячейки памяти
			walker = readMemAddress();
		}
	}

	TWDR = retval;
}

/* TWI slave send data */
inline void senddata(void)
{
  unsigned char index;
  for(index = 0;index < 8; index++)
  {
    while(READ_SCL());
    if((TWDR >> (7 - index))&1)
    	SETSDA();
    else
		CLRSDA();
		SETSCL(); // clock_stretching stop
    while(!READ_SCL());
  }
  // See if we get ACK or NACk
  while(READ_SCL());

  // tristate the pin to see if ack comes or not
  SETSDA();

  while(!READ_SCL());
  if(!READ_SDA())
    TWSR = TWI_SLA_DATA_SND_ACK_RCV;
  else
    TWSR = TWI_SLA_DATA_SND_NACK_RCV;
}


/* TWI slave receive data */
inline void receivedata(void)
{
  unsigned char index;
  TWDR = 0;
  SETSCL(); // clock_stretching stop

  // read 8 bits from master, respond with ACK.
  // SCL could be high or low depending on CPU speed
  for(index = 0;index < 8; index++)
  {
    while(!READ_SCL());
    TWDR = (TWDR<<1) | READ_SDA();
    while(READ_SCL())
    {
      //!if SDA changes while SCL is high, it indicates STOP or START
      if((TWDR & 1)!= READ_SDA())
      {
        if(READ_SDA())
          TWSR = TWI_SLA_STOP;
        else
          TWSR = TWI_SLA_REPEAT_START;
        return;
      }
    }
  }

  if(TWEA && walker != 0x18)
  {
    //!Send ACK, SCL is low now
    CLRSDA();
    while(!READ_SCL());
    while(READ_SCL());
    SETSDA();
    TWSR = TWI_SLA_DATA_RCV_ACK_RTD;
    TWEA = 0;
  }
  else
  {
    TWSR = TWI_SLA_DATA_RCV_NACK_RTD;
  }
}


/* TWI state machine software algorithm that emulates the hardware TWI state machine. */
void TWI_state_machine(void)
{
    // get current state
START:
    // lock twi task
    switch (TWSR)
    {
        /* Own SLA_W has been received;
         * ACK has been returned
         */
        case TWI_SLA_REQ_W_ACK_RTD:
            // walker = 0;
            // incomingBuffer[walker++] = TWDR;
            TWEA = 1;
            receivedata();
            goto START;

            break;


        // data recieved, NACK has been returned
        case TWI_SLA_DATA_RCV_NACK_RTD:
            TWSR = I2C_IDLE;
            TWEA = 1;

            break;

        // data recieved, ack has been returned
        case TWI_SLA_DATA_RCV_ACK_RTD:
        	if(walker > 0x18)
        	{
        		Buffer[walker++] = TWDR;
        	}
        	else if (walker < 0x18)
        	{
        		if(walker == 0x0)
        		{
        			TWDR &= 0xF0;
        		}

        		Buffer[walker++] |= TWDR;
        	}

            TWEA = 1;

            receivedata();
            goto START;

            break;


        /* Own SLA_R has been received;
         * ACK has been returned
         */
        case TWI_SLA_REQ_R_ACK_RTD:
            // walker = 0;
            TWDR = Buffer[walker++];
            TWEA = 1;

            senddata();
            goto START;

            break;

        // data has been transmitted, ACK has been received.
        case TWI_SLA_DATA_SND_ACK_RCV:
            TWDR = Buffer[walker++];
            TWEA = 1;

            senddata();
            goto START;

            break;

        // last data has been transmitted, ACK has been received.
        case TWI_SLA_LAST_DATA_SND_ACK_RCV:

        // data has been transmitted, NACK has been received.
        case TWI_SLA_DATA_SND_NACK_RCV:
            TWEA = 1;
            TWSR = I2C_IDLE;

            break;

        // met stop or repeat start
        case TWI_SLA_STOP:
              // return to idle state
              TWEA = 1;
              TWSR = I2C_IDLE;

            break;
        case TWI_SLA_REPEAT_START:
          GetStartCondition();
          goto START;

        // Idle or bus error
        case I2C_IDLE:
        default:
            TWEA = 1;
            break;
    }
}

void Delay_ms(uint32_t ms)
{
	int i;
	for(i = 0; i < ms; i++)
	{
		__NOP();
	}
}
