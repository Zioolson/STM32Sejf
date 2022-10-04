/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//////////////////////////FLASH
#include "FLASH_PAGE.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define C1_PORT C1_GPIO_Port
#define C1_PIN C1_Pin
#define C2_PORT C2_GPIO_Port
#define C2_PIN C2_Pin
#define C3_PORT C3_GPIO_Port
#define C3_PIN C3_Pin
#define C4_PORT C4_GPIO_Port
#define C4_PIN C4_Pin

#define R1_PORT R1_GPIO_Port
#define R1_PIN R1_Pin
#define R2_PORT R2_GPIO_Port
#define R2_PIN R2_Pin
#define R3_PORT R3_GPIO_Port
#define R3_PIN R3_Pin
#define R4_PORT R4_GPIO_Port
#define R4_PIN R4_Pin

#define MAX_BUFFER_SIZE 256
#define MAX_COMMAND_SIZE 135
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct lcd_disp disp;
typedef struct
{
	uint8_t last_rec_char;
	uint8_t rx_buff[MAX_BUFFER_SIZE];
	uint8_t rx_e;
	uint8_t rx_b;
}buff_receiver;

typedef struct
{
	uint8_t tx_buff[MAX_BUFFER_SIZE];
	uint8_t tx_e;
	uint8_t tx_b;
}buff_sender;
buff_receiver buff_rec;
buff_sender buff_send;
uint8_t cmd_pointer = 0;
uint8_t cmd[MAX_COMMAND_SIZE];

uint8_t key[] = {'1','2','3','4'};
char znak;
uint8_t pin[10];
uint8_t pointer_pin = 0;
bool star = 0;
bool open = 0;
uint8_t i = 0;
bool frame_activated = false;
bool bad_char = 0;

uint32_t first_addr = 0x08006800; //ostatnia komórka 31strony
uint8_t rec_char;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__IO uint32_t Rx_Data[4];
uint8_t string[10];
//////////////////////////////////////////FLASH

void encode_frame(char *msg,char *output){
	int length = strlen(msg) + 3;
	int checksum = calc_checksum(msg);
	sprintf (output, "#%02X%s%02X%c\r\n",length, msg, checksum,'$');
}

void fsend(char* format,...){
  char tmp_rs[128];
  int i;
  __IO int idx;
  va_list arglist;
  va_start(arglist,format);
  vsprintf(tmp_rs,format,arglist);
  va_end(arglist);

  char encframe[134] = {0};
  encode_frame(tmp_rs,encframe);

  idx=buff_send.tx_b;
  for(i=0;i<strlen(encframe);i++){
  buff_send.tx_buff[idx]=encframe[i];
  idx++;
  if(idx >= MAX_BUFFER_SIZE)idx=0;
  }
  __disable_irq();
  if((buff_send.tx_e==buff_send.tx_b)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){
  buff_send.tx_e=idx;
  uint8_t tmp=buff_send.tx_buff[buff_send.tx_b];
  buff_send.tx_b++;
  if(buff_send.tx_b >= MAX_BUFFER_SIZE)buff_send.tx_b=0;
  HAL_UART_Transmit_IT(&huart2, &tmp, 1);
  }else{
	  buff_send.tx_e=idx;
  }
  __enable_irq();
  }

int calc_checksum(const char *msg){
    int sum_of_bytes = 0;
    int result = 0;
    for(int i = 0; i < strlen(msg);i++){
        sum_of_bytes = sum_of_bytes + msg[i];
        result = sum_of_bytes %= 256;
    }
    return result;
}

void dec_to_hex(int complimentInt,char* complimentHex)
{
    sprintf(complimentHex,"%X",complimentInt);
}

int hex_to_dec(char* hex)
{
	int decimal = 0;
	  int length = 0;
	  length = 2;
	  char digit;
	  for (int i = 0; i < length; i++)
	    {
	      digit = hex[length - 1 - i];
	      if (digit >= '0' && digit <= '9')
		decimal += (digit - '0') * pow (16, i);
	      else
		decimal += (digit - 'A' + 10) * pow (16, i);
	    }
	  return decimal;
}


void check_cmd()
{
	uint8_t* strippedcommand = &cmd[2];
	if (!strcmp((const char*) strippedcommand,"open;"))
	{
	fsend("Odebrano poprawnie\r\n");
	set_ang(0,0);
	sprintf((char *)disp.s_line, "OTWARTE");
	lcd_display(&disp);
	}
	else if (!strcmp((const char*) strippedcommand,"close;"))
	{
	fsend("Odebrano poprawnie\r\n");
	set_ang(900,0);
	sprintf((char *)disp.s_line, "ZAMKNIETE");
	lcd_display(&disp);
	}
	else fsend("Nie odebrano\r\n");
}

void check_frame()
{
	char frame_lenHex[2] = {cmd[0],cmd[1]};
	int frame_lenInt = hex_to_dec(frame_lenHex);
	if(frame_lenInt > 128 || frame_lenInt < 3)
	{
		memset(cmd,0,sizeof cmd);
		cmd_pointer = 0;
		return;
	}
	int calculated_length = strlen((char*)&cmd[2]) + 1;
		if(frame_lenInt != calculated_length){
			memset(cmd,0,sizeof cmd);
			cmd_pointer = 0;
			return;
		}
	char checksum_from_frame[2] = {cmd[frame_lenInt-1],cmd[frame_lenInt]};
	int  checksum_from_frameInt = hex_to_dec(checksum_from_frame);
	cmd[frame_lenInt-1] = '\0';
	int calculated_checksum = calc_checksum((char*)&cmd[2]);

	if(calculated_checksum == checksum_from_frameInt)
	{
		check_cmd();
	}
	memset(cmd,0,sizeof cmd);
	cmd_pointer = 0;
}
void check_pin()
{

	uint8_t i;
		if(!strcmp(pin, string)){
		  sprintf((char *)disp.f_line, "");
		  lcd_display(&disp);
		  sprintf((char *)disp.s_line, "Otwarte");
		  lcd_display(&disp);
		  set_ang(0,0);
		  pointer_pin = 0;
	      open = 1;
	  }
	  else{
		  sprintf((char *)disp.s_line, "Niepoprawny pin!");
		  lcd_display(&disp);
		  pin[0]=' ';
		  pin[1]=' ';
		  pin[2]=' ';
		  pin[3]=' ';
		  pointer_pin = 0;
		  open = 0;
	  }
}

char read_keypad (void)
{

	star = 0;
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		return '1';

	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		return '2';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		return '3';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		return 'A';
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		return '4';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		return '5';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		return '6';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		return 'B';
	}


	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		return '7';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		return '8';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		return '9';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		return 'C';
	}


	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		star = 1;

		if(open){
				sprintf((char *)disp.f_line,"");
				lcd_display(&disp);
				sprintf((char *)disp.s_line, "WPISZ NOWY PIN");
				lcd_display(&disp);
		}
		return '*';
	}

	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		return '0';
	}

	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		sprintf((char *)disp.f_line, "");
				lcd_display(&disp);
				sprintf((char *)disp.s_line, "ZAMKNIETE");
				lcd_display(&disp);
				set_ang(900,0);

				pin[0]=' ';
				pin[1]=' ';
				pin[2]=' ';
				pin[3]=' ';
				pointer_pin = 0;
				star = 1;
				open = 0;

		return '#';
	}

	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		return 'D';
	}
	return 0;
}

	void check_char(){
		if(rec_char > 34 && rec_char < 126)
		{
			bad_char = 0;
		}
		else{
			bad_char = 1;
		}
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &buff_rec.last_rec_char, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  disp.addr = (0x20 << 1);
  disp.bl = true;
  lcd_init(&disp);
  sprintf((char *)disp.s_line, "ZAMKNIETE");
  lcd_display(&disp);
  set_ang(900,0);

  Flash_Write_Data(first_addr, key);
  Flash_Read_Data(first_addr, Rx_Data);
  Convert_To_Str(Rx_Data, string);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(buff_rec.rx_e != buff_rec.rx_b)
	 	    {
			  rec_char = buff_rec.rx_buff[buff_rec.rx_b];
	 		  buff_rec.rx_b++;
	 		  if(buff_rec.rx_b >= MAX_BUFFER_SIZE)
	 		  {
	 			  buff_rec.rx_b = 0;
	 		  }
	 	  	  if(rec_char == '#')
	 	  	  {
	 	  			memset(cmd,0,sizeof cmd);
	 	  			cmd_pointer = 0;
	 	  		frame_activated = true;

	 	  	  }
	 	  	  check_char();
	 	  	  if(bad_char){
	 	  		memset(cmd,0,sizeof cmd);
	 	  		cmd_pointer = 0;
	 	  		frame_activated = false;
	 	  	  }

	 		  if(rec_char == '$' && frame_activated)
	 		  {
	 				 frame_activated = false;
	 				 check_frame();
	 		  }
	 			  if(frame_activated && rec_char != '#' && rec_char != '$' && rec_char > 38 && rec_char < 126)
	 		 		  {
	 		 			  cmd[cmd_pointer] = rec_char;
	 		 			  cmd_pointer++;
	 		 		  }

	 		 		  if(cmd_pointer >= MAX_COMMAND_SIZE)
	 		 		  {
	 		 			  cmd_pointer = 0;
	 		 			  frame_activated = false;
	 		 		  }
	 		 }


	  if((znak = read_keypad())!=0)
	    {
	 	  if(star == 0)
	 	  	{
				pin[pointer_pin] = znak;
				pointer_pin ++;
				switch(pointer_pin){
				case 1:
				sprintf((char *)disp.f_line,"*");
				lcd_display(&disp);
				break;
				case 2:
				sprintf((char *)disp.f_line,"**");
				lcd_display(&disp);
				break;
				case 3:
				sprintf((char *)disp.f_line,"***");
				lcd_display(&disp);
				break;
				case 4:
				sprintf((char *)disp.f_line,"****");
				lcd_display(&disp);
				break;
				}
	 	  	}

		  if(pointer_pin == 4)
			{
			  if(open == 0){
				check_pin();
			  }
			  else{
				Flash_Write_Data(first_addr, pin);
				Flash_Read_Data(first_addr, Rx_Data);
				Convert_To_Str(Rx_Data, string);

				if(pointer_pin >= 4){
				sprintf((char *)disp.s_line,"PIN ZMIENIONY");
				lcd_display(&disp);

				}
			  }
			}
		  if(pointer_pin>=4)
		   {
				pointer_pin = 0;
				pin[0]=' ';
				pin[1]=' ';
				pin[2]=' ';
				pin[3]=' ';
		   }
		  star=0;

	    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		buff_rec.rx_buff[buff_rec.rx_e] = buff_rec.last_rec_char;
		buff_rec.rx_e++;
		if(buff_rec.rx_e >= MAX_BUFFER_SIZE)
		{
			buff_rec.rx_e = 0;
		}
		HAL_UART_Receive_IT(&huart2, &buff_rec.last_rec_char, 1);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
			if(buff_send.tx_e != buff_send.tx_b)
			{
				HAL_UART_Transmit_IT(&huart2, &buff_send.tx_buff[buff_send.tx_b], 1);
				buff_send.tx_b++;
				if(buff_send.tx_b >= MAX_BUFFER_SIZE)
				{
					buff_send.tx_b = 0;
				}
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
