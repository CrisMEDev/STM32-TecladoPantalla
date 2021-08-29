/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "retarget.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "keypad.h"
#include "lcd3310_GPIO.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char tecla = 0;				// Variable que guarda la tecla presionada
char arrayIngresar[10];		// Arreglo que guarda el dato nuevo a ingresar
char arrayModificar[10]; 	// Arreglo que guarda el dato de modificar temperatura nuevo
int i = 0; 					// Contador para los arreglos
double numero = 0.0; 			// Variable donde se guarda el numero convertido del arrayIngresar
double numero1 = 27.0; 		// Variable donde se guarda el numero convertido del arrayMOdificar
uint8_t menu = 0; 			// Variable para cambiar de funcion

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void principal(void);
void ingresarValor(void);
void modificarTempPredefinida(void);
void mostrarTemperaturasLCD(char *temperaturaActual, char *temperaturaObjetivo);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char stringCelsius[10] = "";
  float celsius = 0.0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart2);		// INICIA LA TRANSFERENCIA POR UART2
  //Inicializar la pantalla LCD
  LCDInit();
  LCDContrast (0x60);
  LCDClear();
  LCDUpdate();
  keypad_init(); 				// Inicializa el teclado

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	sprintf(stringCelsius, "%.4f", celsius);			 	// Conversión de celsius a string

	mostrarTemperaturasLCD(stringCelsius, arrayIngresar);		// Array copia es la temperatura objetivo
	memset(stringCelsius, 0, 10);							// Limpia la variable stringCelsius

	if(menu == 0)
		principal(); 											// Por default la variable menu = 0 entrando inicialmente aqui
	else if(menu == 1)
		ingresarValor();
	else if(menu == 2)
		modificarTempPredefinida();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void principal(void){ 			// La funcion principal está leyendo las teclas desde el principio
	tecla = keypad_read();
	if(tecla){
		if(tecla == 'A'){ 		// Si se presiona la tecla A, la variable menu = 1, lo que pasa a la funcion de ingresar nuevo valor
			menu = 1; 			// Cuando se termine de ingresar valor, entra otra vez a la funcion principal

			// Vaciar el array de ingreso
			memset(arrayIngresar, 0, 10);
		}
		if(tecla == 'C'){
			printf("Temperatura Predefinida: %.2f\r\n", numero1); // Si la tecla es C, muestra la cantidad que tiene por defecto
		}
		if(tecla == 'D'){
			printf("Editar temp\n\r"); // Si se presiona la tecla D, la variable menu = 2, lo que entra en la funcion modificarTemp
			menu = 2;
		}
	}

}

void ingresarValor(){

	tecla = keypad_read();
	if(tecla){
		if((tecla >= '0' && tecla <= '9') || (tecla == '.')){ //el array se llena solo si se presionan esas teclas
		arrayIngresar[i] = tecla;
		i++;
		}
		if(tecla == '#'){ 							// Si se presiona el #, convierte el array a numero, y lo guarda en la variable numero
			numero = atof(arrayIngresar);
			printf("numero convertido a float: %.2f\n\r", numero*2);
			i = 0; 									// Se reinicia el indice del arreglo
			menu = 0; 								// La variable menu es 0, para que entre otra vez a la funcion principal

			// Vaciar el array de ingreso
			memset(arrayIngresar, 0, 10);
		}
		if(tecla == 'B'){ 							// Si se presiona la tecla B, borra el ultimo elemento del arreglo
			printf("menos\n\r");
			arrayIngresar[strlen(arrayIngresar)-1] = '\0';
			printf("%s\n\r",arrayIngresar);
			i--;

		}
	}

}

void modificarTempPredefinida(){ 	// Funcion para modificar la temp predefinida
	tecla = keypad_read();			// Es lo mismo que ingresarValor(), solo que el valor se guarda en la variable numero1
		if(tecla){					// despues de hacer esto, si se presiona la C, ya muestra el valor nuevo
			if((tecla >= '0' && tecla <= '9') || (tecla == '.')){
			//printf("%c\n", tecla);
			arrayModificar[i] = tecla;
			i++;
			}
			if(tecla== '#'){
				numero1 = atof(arrayModificar);
				printf("numero: %f\n\r", numero1);
				i = 0;
				menu = 0;
				memset(arrayModificar, 0, 10);
			}
			if(tecla == 'B'){
				printf("menos\n\r");
				arrayModificar[strlen(arrayModificar)-1] = '\0';
				printf("%s\n\r",arrayModificar);
				i--;
			}
		}
}

void mostrarTemperaturasLCD(char *temperaturaActual, char *temperaturaObjetivo){
	LCDStr(0, (unsigned char *) "T Actual (°C)", 0);
	LCDStr(1, (unsigned char *) temperaturaActual, 0);
	LCDStr(3, (unsigned char *) "T Fijada (°C)", 0);
	LCDStr(4, (unsigned char *) temperaturaObjetivo, 0);
	LCDUpdate();

	return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
