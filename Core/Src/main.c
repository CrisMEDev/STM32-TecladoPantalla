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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char tecla = 0;				// Variable que guarda la tecla presionada
char arrayIngresar[10];		// Arreglo que guarda el dato nuevo a ingresar
int i = 0; 					// Contador para los arreglos
double numero = 0.0; 		// Variable donde se guarda el numero convertido del arrayIngresar (Temperatura objetivo)
double numero1 = 30.0; 		// Variable que almacena el valor objetivo por defecto
char control = 'Z';			// Variable que almacena los comandos introducidos desde teclado matricial

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

////////////////////////////////////////////////////////////////////////////////
// Mapeo de una lectura de entrada analógica en rango (in_min, in_max)		  //
// a un valor en el rango (out_min, out_max)								  //
////////////////////////////////////////////////////////////////////////////////
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);

////////////////////////////////////////////////////////////////////////////////
// Almacena un valor de entrada por teclado en el arreglo arrayIngresar		  //
// y lo convierte a float, ya sea para cambiar el valor objetivo (numero)	  //
// o la temperatura pre-establecida (numero1), según sea el caso del comando  //
// seleccionado con la interrupción											  //
// La tecla enter se representa por #										  //
// La tecla B elimina un caracter en caso de que el usuario se equivoque	  //
////////////////////////////////////////////////////////////////////////////////
void ingresarValor(void);

////////////////////////////////////////////////////////////////////////////////
// Muestra la temperatura actual contra la temperatura objetivo				  //
////////////////////////////////////////////////////////////////////////////////
void mostrarTemperaturasLCD(char *temperaturaActual, char *temperaturaObjetivo);

////////////////////////////////////////////////////////////////////////////////
// Muestra le primer mensaje por pantalla para darle a entender al usuario	  //
// que puede introducir un comando											  //
////////////////////////////////////////////////////////////////////////////////
void mensajeInterrupcion(void);

////////////////////////////////////////////////////////////////////////////////
// Muestra por pantalla el comando seleccionado por el usuario:				  //
// A -> Para cambiar la temperatura objetivo								  //
// B -> Para usar el valor pre-establecido como temperatura objetivo		  //
// C -> Para modificar el valor pre-establecido								  //
// # -> Representa un 'enter' una vez seleccionado un comando anterior		  //
// 																			  //
// Caracteres diferentes a los anteriores son ignorados, dar un enter(#)	  //
// sin comando seleccionado, hace caso omiso de la interrupcion y mantiene	  //
// al sistema trabajando con el valor que ya tenía antes de la interrupción	  //
////////////////////////////////////////////////////////////////////////////////
void mostrarComandoElegido(char mensaje);

////////////////////////////////////////////////////////////////////////////////
// Funciones necesarias para inicializar la LCD agrupadas en esta función	  //
// ya que inicializar la pantalla en la interrupción es necesario			  //
////////////////////////////////////////////////////////////////////////////////
void inicializarLCD(void);

////////////////////////////////////////////////////////////////////////////////
// Convierte flotantes a cadenas											  //
////////////////////////////////////////////////////////////////////////////////
static char * _float_to_char(float x, char *p, int CHAR_BUFF_SIZE);

////////////////////////////////////////////////////////////////////////////////
// Función llamada para verificar que pin o pines provocaron la interrupción  //
//																			  //
// Si el usuario provoca interrupcion con el botón de usuario (PC13)		  //
// Se muestra un mensaje y se espera la entrada de un comando, dependiendo	  //
// de la elección se realiza la acción correspondiente						  //
////////////////////////////////////////////////////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
  char voltajeEntradaPC0[10] = "";
  unsigned int valorADC1_PC0 = 0;
  float valorADC1_PC0_toFloat = 0;

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart2);		// INICIA LA TRANSFERENCIA POR UART2

  //Inicializar la pantalla LCD
  inicializarLCD();

  // Se comenta la linea de configuración del teclado ya que se uso la interfaz stm32CubeIDE
  // keypad_init(); 				// Inicializa el teclado

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Se alamacena el valor pre establecido en numero(valor objetivo) para que en
  // un inicio el sistema comience a ajustarse a ese valor
  numero = numero1;
  sprintf(arrayIngresar, "%.4f", numero);	// _float_to_char(numero, arrayIngresar, 10);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_ADC_Start(&hadc1);
	// HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	valorADC1_PC0 = HAL_ADC_GetValue(&hadc1);
	valorADC1_PC0_toFloat = mapfloat(valorADC1_PC0, 0, 4095, 0, 5);

	// _float_to_char(valorADC1_PC0_toFloat, voltajeEntradaPC0, 10);
	sprintf(voltajeEntradaPC0, "%.4f", valorADC1_PC0_toFloat);	// Conversion de voltaje de entrada a string

	// printf("Objetivo: %s\n\r", arrayIngresar);
	// printf("Numero: %f\n\r", numero);

	mostrarTemperaturasLCD(voltajeEntradaPC0, arrayIngresar);	// Muestra el voltaje medido en PC0 y el voltaje objetivo
	memset(voltajeEntradaPC0, 0, 10);							// Limpia la variable voltajeEntradaPC0

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA8 PA9 
                           PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
	return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void ingresarValor(){

	tecla = keypad_read();
	if(tecla){
		if((tecla >= '0' && tecla <= '9') || (tecla == '.')){ // El array se llena solo si se presionan esas teclas
		arrayIngresar[i] = tecla;
		i++;
		}
		if(tecla == '#'){ 							// Si se presiona el #, convierte el array a numero, y lo guarda en la variable numero

			double aux;	// Para manejar un dato de entrada incorrecto

			// printf("atof: %f\n\r", atof(arrayIngresar));
			aux = atof(arrayIngresar) ? atof(arrayIngresar) : numero;

			// Si el comando introducido fue A, almacena su valor como el objetivo, sino como el valor predefinido
			if (control == 'A') numero = aux;
			else if(control == 'C') numero1 = aux;

			// printf("numero convertido a float: %.2f\n\r", numero*2);
			i = 0; 									// Se reinicia el indice del arreglo

			// Vaciar el array de ingreso
			memset(arrayIngresar, 0, 10);
		}
		if(tecla == 'B'){ 							// Si se presiona la tecla B, borra el ultimo elemento del arreglo

			if (control == 'A'){
				LCDStr(3, (unsigned char *)"              ", 0);	// Para visualizar el borrado de el digito introducido
				arrayIngresar[strlen(arrayIngresar)-1] = '\0';		// Borra el último caracter de el arreglo de ingreso
				// printf("%s\n\r",arrayIngresar);
			}
			else if (control == 'C'){
				LCDStr(4, (unsigned char *)"              ", 0);	// Para visualizar el borrado de el digito introducido
				arrayIngresar[strlen(arrayIngresar)-1] = '\0';		// Borra el último caracter de el arreglo de ingreso
				// printf("%s\n\r",arrayIngresar);
			}

			// Si se elimina el primer caracter del arreglo, mantiene a 'i' en cero
			(i < 0) ? i = 0 : i--;

		}
	}

	return;
}

void mostrarTemperaturasLCD(char *temperaturaActual, char *temperaturaObjetivo){
	LCDStr(0, (unsigned char *) "T Actual(~C)", 0);
	LCDStr(1, (unsigned char *) temperaturaActual, 0);
	LCDStr(3, (unsigned char *) "T Objetivo(~C)", 0);
	LCDStr(4, (unsigned char *) temperaturaObjetivo, 0);
	LCDUpdate();

	return;
}

void mensajeInterrupcion(){

	inicializarLCD();
	LCDStr(0, (unsigned char *) "   Dame un    ", 1);
	LCDStr(1, (unsigned char *) "   comando:   ", 1);
	LCDUpdate();

	return;
}

void mostrarComandoElegido(char mensaje){

	char comando[5] = "";

	// printf("%c\n\r", mensaje);
	// printf("%s\n\r", comando);

	if ( mensaje == 'A' || mensaje == 'B' || mensaje == 'C' ){
		control = mensaje;			// Almacena el comando seleccionado por el usuario
		comando[0] = mensaje;		// Se almacena el char a un apuntador char para mostrar en pantalla
		LCDStr(3, (unsigned char *) comando, 0);
		LCDUpdate();
		memset( comando, 0, 5 );	// Se limpia la memoria en 'comando'
	}

	return;
}

void inicializarLCD(){
	LCDInit();
	LCDContrast (0x60);
	LCDClear();
	LCDUpdate();
}

static char * _float_to_char(float x, char *p, int CHAR_BUFF_SIZE){
    char *s = p + CHAR_BUFF_SIZE; // go to end of buffer
    for (i=0; i<CHAR_BUFF_SIZE; i++)
    	p[i]='0';
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -100) % 100; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 100) % 100;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    decimals /= 10; // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    *--s = '.';

    while (units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if (x < 0) *--s = '-'; // unary minus sign for negative numbers
    return s;
}

// Función que se ejecuta con las interrupciones
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if ( GPIO_Pin == GPIO_PIN_13 ){
		char comando = 'Z';
		control = 'Z';

		// Para indicar que la interrupción está activa, se activa el led verde de la placa
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

		mensajeInterrupcion();

		// Lee el teclado hasta que se coloque el caracter '#'
		while (comando != '#'){
			comando = keypad_read();
			mostrarComandoElegido(comando);
		}
		comando = 'Z';
		// printf("El comando seleccionado es: %c\n\r", control);

		// Vaciar el array de ingreso
		memset(arrayIngresar, 0, 10);

		inicializarLCD();

		switch(control){
			case 'A':	// Ingresar un valor de temperatura objetivo

				LCDStr(0, (unsigned char *) "   Modificar  ", 1);
				LCDStr(1, (unsigned char *) "   objetivo:  ", 1);
				LCDUpdate();

				while( comando != '#' ){
					ingresarValor();
					LCDStr(3, (unsigned char *) arrayIngresar, 0);
					comando = tecla;
				}

				// _float_to_char(numero, arrayIngresar, 10);
				sprintf(arrayIngresar, "%.4f", numero);			 	// Conversión de numero a string

				break;

			case 'B':	// Establece el valor objetivo con un valor pre-establecido en la variable numero1

				LCDStr(0, (unsigned char *) "   Valor      ", 1);
				LCDStr(1, (unsigned char *) "   objetivo   ", 1);
				LCDStr(2, (unsigned char *) "   fijado en  ", 1);

				numero = numero1;
				// _float_to_char(numero, arrayIngresar, 10);
				sprintf(arrayIngresar, "%.4f", numero);			 	// Conversión de numero a string
				LCDStr(4, (unsigned char *) arrayIngresar, 0);

				LCDUpdate();

				// Delay para que el usuario pueda visualizar el mensaje del cambio
				// del valor objetivo
				{
					unsigned int c = 0;

					while(c < 1000000)c++; c = 0;
					while(c < 1000000)c++; c = 0;
					while(c < 1000000)c++; c = 0;
				}

				break;

			case 'C':	// Cambia el valor de la variable que tiene un valor pre-establecido
						// Para usarse este valor es necesario usar el comando B despues de actualizar el dato aqui

				LCDStr(0, (unsigned char *) "  Modificar   ", 1);
				LCDStr(1, (unsigned char *) "  valor pre   ", 1);
				LCDStr(2, (unsigned char *) "  establecido ", 1);
				LCDUpdate();

				while( comando != '#' ){
					ingresarValor();
					LCDStr(4, (unsigned char *) arrayIngresar, 0);
					comando = tecla;
				}

				// _float_to_char(numero, arrayIngresar, 10);
				sprintf(arrayIngresar, "%.4f", numero);			 	// Conversión de numero a string

				break;
			default:
				break;
		}

		// Para indicar que la interrupción está inactiva, se desactiva el led verde de la placa
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		memset(arrayIngresar, 0, 10);

		// Esta línea hace que se mantenga el valor reflejado en pantalla con la funcion mostrarTemperaturasLCD()
		// _float_to_char(numero, arrayIngresar, 10);
		sprintf(arrayIngresar, "%.4f", numero);

		LCDClear();
	}
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
