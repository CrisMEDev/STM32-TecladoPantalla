#ifndef __KEYPAD__
#define __KEYPAD__

#include "stm32f1xx_hal_def.h"


#define		ROW1_PIN		GPIO_PIN_7		//	PA7
#define  	ROW1_PORT		GPIOA

#define		ROW2_PIN		GPIO_PIN_8		//	PA8
#define  	ROW2_PORT		GPIOA

#define		ROW3_PIN		GPIO_PIN_9		//	PA9
#define  	ROW3_PORT		GPIOA

#define		ROW4_PIN		GPIO_PIN_10		//	PA10
#define  	ROW4_PORT		GPIOA

#define		COL1_PIN		GPIO_PIN_11		//	PA11
#define  	COL1_PORT		GPIOA

#define		COL2_PIN		GPIO_PIN_12		//	PA12
#define  	COL2_PORT		GPIOA

#define		COL3_PIN		GPIO_PIN_13		//	PA13
#define  	COL3_PORT		GPIOA

#define		COL4_PIN		GPIO_PIN_14		//	PA14
#define  	COL4_PORT		GPIOA


// void keypad_init(void);
char keypad_read(void);
#endif
