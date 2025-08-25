/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "stepmotor.h"

/*THIS is AHSM_0317 file*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 64 //모터 ?��?��?��
uint8_t rx_buffer5[RX_BUFFER_SIZE];
uint8_t rx_data5[RX_BUFFER_SIZE];
uint8_t data_length5 = 0;
uint8_t uart_flag =0;
uint8_t command[20];

#define DEBOUNCE_DELAY 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
// 20250822-phg
int tf = 0;
uint32_t lastTime1 = 0;
uint32_t lastTime2 = 0;
uint32_t lastTime3 = 0;
uint32_t lastTime4 = 0;
uint32_t lastTime5 = 0;
int currentSequence = 0;
int buttonState = 0;
int buttonLastState = 0;
uint32_t pressStartTime = 0;
int pauseState = 0;
int emergencyStop = 0;
int isRunning = 0;
int testflag = 0;
int testflag_hair = 0;



int systemRunning = 0;
int ledSequence = 0;
int sequencePaused = 0;
int flag_01=0;
int flag_C=0;
int hair_detect = 0;
int hair_detect_02=0;
int sequence_01=1;
int sequence_02=1;
int sequence_stop=0;
int sequence_04=0;
int sequence_05=0;

double setlen = 71.256;

int hair=0;
int case_flag_07=0;
int test1 = 0;

int motor2_initialized = 0;
int motor3_initialized = 0;
int dummy_B =0;
int sw_test_01 =0;
int case_01_flag=0;
int caseIntervals1 = 5000;
int caseIntervals2 = 5000;
int caseIntervals3 = 5000;
int caseIntervals4 = 5000;
int caseIntervals5 = 5000;

static int step = 0;
static int step2 = 0;
static int step3 = 0;
static int step4 = 0;
static int step5 = 0;

uint32_t LastTimePE6 = 0;
uint32_t LastTimePC13 = 0;

typedef enum {

	MOTOR_INIT_C,
	MOTOR_INIT_B,

	MOTOR_INIT_DONE
} Motor_Init_State;


Motor_Init_State motorInitState = MOTOR_INIT_C;

StepMotor motor1; //motorA
StepMotor motor2;//motorB
StepMotor motor3;//motorC
StepMotor motor4;//motorD

double motor1_flag = 0;
double motor2_flag = 0;
double motor3_flag = 0;
double motor4_flag = 0;
double hairang = 71;

int Laststate = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void LED_Sequence_01(void);
void LED_Sequence_02(void);
void LED_Sequence_stop(void);
void LED_Sequence_04(void);

void Emergency_Stop(void);

void Stop_All(void);
void Emergency_Stop(void);
void Resume_Sequence(void);
void Polling_Buttons(void);
void sw_test(void);
void add_port_test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int flagexti = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0){
		flagexti++;
		if (motor1_flag == 0){
			motor1.current_angle_step = -19500; // it was -19500 => --9750
			motor1.set_desired_angle(&motor1, 0);
			motor1_flag=-1;

		}
		flag_01++;
	}

	else if (GPIO_Pin == GPIO_PIN_1) {
		if (motor2_flag == 0) {
			motor2.current_angle_step = 0;
			motor2.set_desired_angle(&motor2, 0);
			motor2_flag = -1;
			motor2.step_speed=50;
		}
		//		flagexti++;
	}

	// Motor 3
	else if (GPIO_Pin == GPIO_PIN_2) {
		if (motor3_flag == 0) {
			motor3.current_angle_step = 0;
			motor3.set_desired_angle(&motor3, 0);
			motor3_flag = -1;

			//			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)){ //sensor off

			motor2.set_desired_angle(&motor2, 0);
			motor2.current_angle_step = 2400;
			motor2_flag = 0;
			flag_C++;
			//			} else {
			//
			//				motor2.current_angle_step = -200;
			//				motor2_flag =1000;
			//				flag_C=2;
			//			}
		}
		//		flagexti = 3;
	}

	// Motor 4 (
	else if (GPIO_Pin == GPIO_PIN_3) {
		if (motor4_flag == 0) {
			motor4.set_desired_angle(&motor4, 0);
			motor4.current_angle_step = 0;
			motor4_flag = -1;
			//			motor4.step_speed=50;
		}
		//		flagexti = 4;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//rx_buffer7[0]++;
	if (huart == &huart5) {
		rx_data5[data_length5++] = rx_buffer5[0];
		if (rx_buffer5[0] == '\n' || data_length5 >= RX_BUFFER_SIZE - 1) {
			HAL_UART_Transmit(&huart5, rx_data5, data_length5, HAL_MAX_DELAY);
			rx_data5[data_length5 - 1] = '\0';
			data_length5 = 0;
			uart_flag = 1;
		}

		HAL_UART_Receive_IT(&huart5, rx_buffer5, 1);
		//		flagexti++;
	}
}
void uart_decoder()
{
	if (uart_flag == 1) {
		char *token;
		token = strtok(rx_data5, ",");
		strcpy(command, token);
		if ((token != NULL)) {
			if (strncmp(command, "Bping", 5) == 0) {
				uint8_t print[] = "pong\n";
				HAL_UART_Transmit(&huart5, print, sizeof(print) - 1,
						HAL_MAX_DELAY);
				testflag=12;
			} else if (strncmp(command, "init", 4) == 0) {
				motor_pos_init();

			} else if (strncmp(command, "setlen", 6) == 0) {
				//				testflag=123;
				token = strtok(NULL, ",");
				hairang = atof(token);
				hairang = hairang - 1.0;
			} else if (strncmp(command, "end", 3) == 0) {
				systemRunning = 0;
				isRunning=0;
				solV_blow_OFF();
				solV_9_OFF();
				step=0;
				step2=0;
				step4=0;

			} else if (strncmp(command, "test1", 5) == 0) {
				token = strtok(NULL, ",");
				test1 = atof(token);

			} else if (strncmp(command, "z", 1) == 0) {
				token = strtok(NULL, ",");
				if (atof(token)>500) {
					solV_Grab_ON();
					//					testflag = 10;
					solV_8_ON();
					solV_12_ON();
					solV_16_ON();
					solV_14_ON();
					solV_18_ON();
				} else {
					solV_Grab_OFF();
					//					testflag = 11;
					solV_8_OFF();
					solV_12_OFF();
					solV_16_OFF();
					solV_14_OFF();
					solV_18_OFF();
				}



			} else if (strncmp(command, "01", 2) == 0) {
				solV_1_ON();
			} else if (strncmp(command, "02", 2) == 0) {
				solV_2_ON();
			} else if (strncmp(command, "03", 2) == 0) {
				solV_3_ON();
			} else if (strncmp(command, "04", 2) == 0) {
				solV_4_ON();
			} else if (strncmp(command, "05", 2) == 0) {
				solV_5_ON();
			} else if (strncmp(command, "06", 2) == 0) {
				solV_6_ON();
			} else if (strncmp(command, "07", 2) == 0) {
				solV_7_ON();
			} else if (strncmp(command, "08", 2) == 0) {
				solV_8_ON();
			} else if (strncmp(command, "09", 2) == 0) {
				solV_9_ON();
			} else if (strncmp(command, "10", 2) == 0) {
				solV_10_ON();
			} else if (strncmp(command, "11", 2) == 0) {
				solV_11_ON();
			} else if (strncmp(command, "12", 2) == 0) {
				solV_12_ON();
			} else if (strncmp(command, "13", 2) == 0) {
				solV_13_ON();
			} else if (strncmp(command, "14", 2) == 0) {
				solV_14_ON();
			} else if (strncmp(command, "15", 2) == 0) {
				solV_15_ON();
			} else if (strncmp(command, "16", 2) == 0) {
				solV_16_ON();
			} else if (strncmp(command, "17", 2) == 0) {
				solV_17_ON();
			} else if (strncmp(command, "18", 2) == 0) {
				solV_18_ON();

			} else if (strncmp(command, "Start", 5) == 0) {
				test1 = 0;
				systemRunning = 1;
				isRunning=1;
				solV_16_OFF();
				solV_12_OFF();

			} else if (strncmp(command, "stop", 5) == 0) {
				systemRunning = 0;
				isRunning=0;
				solV_blow_OFF();


			} else if (strncmp(command, "reset", 5) == 0) {
				LED_Sequence_stop();

			} else if (strncmp(command, "force", 5) == 0) {

				solV_16_ON();
				solV_12_ON();


			} else if (strncmp(command, "motors", 6) == 0) {

				uint8_t print[] = "rp go\n";
				HAL_UART_Transmit(&huart5, print, sizeof(print) - 1,
						HAL_MAX_DELAY);

			} else if (strncmp(command, "roll", 4) == 0) {
				token = strtok(NULL, ",");
				uint8_t print[] = "roll go\n";
				HAL_UART_Transmit(&huart5, print, sizeof(print) - 1,
						HAL_MAX_DELAY);
				motor1.set_desired_angle(&motor2, atof(token));
			} else if (strncmp(command, "pitch", 5) == 0) {
				token = strtok(NULL, ",");
				//				step_go_degree(atof(token));
				uint8_t print[] = "pitch go\n";
				HAL_UART_Transmit(&huart5, print, sizeof(print) - 1,
						HAL_MAX_DELAY);
				motor1.set_desired_angle(&motor1, atof(token));
			}
		}
		uart_flag = 0;
	}
}


void Polling_Buttons(void) {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		if (HAL_GetTick() - LastTimePC13 > DEBOUNCE_DELAY){
			LastTimePC13 = HAL_GetTick();

			emergencyStop = 1;
			Stop_All();
			currentSequence = 0;
			systemRunning = 0;
			ledSequence = 1;
			sequencePaused = 0;
		}
	}

	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET) {
		if (HAL_GetTick() - LastTimePE6>DEBOUNCE_DELAY){
			LastTimePE6=HAL_GetTick();
			if (Laststate == 0) {
				Laststate = 1;
				testflag++;
				if (emergencyStop == 0) {
					if (systemRunning == 0) {
						systemRunning = 1;
						isRunning = 1;

					} else if (systemRunning == 1) {
						systemRunning = 0;
						isRunning = 0;
						solV_blow_OFF();
					}
				}
			}

		}
	}
	else {
		if (HAL_GetTick() - LastTimePE6>DEBOUNCE_DELAY){
			LastTimePE6=HAL_GetTick();
			Laststate = 0;

		}
	}
}
void motor_pos_init(void){
	solV_8_ON();
	solV_12_ON();solV_16_ON();
	solV_10_OFF();
	solV_4_OFF();
	solV_5_OFF();
	step=0;
	step2=0;
	//step3=0;
	step4=0;

	motor1.step_speed=5;
	motor2.step_speed=250;
	motor3.step_speed=50;
	motor4.step_speed=250;
	motor1.set_desired_angle(&motor1, 0);
	motor2.set_desired_angle(&motor2, 0);
	motor3.set_desired_angle(&motor3, 0);
	motor4.set_desired_angle(&motor4, 0);
	//-----------MOTOR A initialize--------------------//
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)){ //sensor off
		//		motor1.set_desired_angle(&motor1, -10);
		motor1.current_angle_step = 25000;//25000=>12500
		//HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		motor1_flag = 0;
	} else {
		//		motor1.set_desired_angle(&motor1, 10);
		motor1.current_angle_step = -1000;//-1000=>-500
		motor1_flag =5000;
	}
	//Initialize_Motors_StepByStep();
	//-----------MOTOR C initialize--------------------//

	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)){ //sensor off
		//		motor1.set_desired_angle(&motor1, -10);
		motor3.current_angle_step = 1000; //5000=> 500
		//HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		motor3_flag = 0;
	} else {
		//		motor1.set_desired_angle(&motor1, 10);
		motor3.current_angle_step = -1000;//-5000=> -500
		motor3_flag =5000;

	}
	motor2.current_angle_step = -100;
	//-----------MOTOR D initialize--------------------//
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)){ //sensor off
		motor4.current_angle_step = -2400;//4000=> 400
		motor4_flag = 0;
	} else {
		motor4.current_angle_step = 2000;//-4000=> -400
		motor4_flag =10000;
	}
	HAL_Delay(2000);
	solV_12_OFF();solV_16_OFF();solV_9_OFF();
}

void Stop_All(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 |GPIO_PIN_11 | GPIO_PIN_13 , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12 | GPIO_PIN_10 | GPIO_PIN_14, GPIO_PIN_RESET);


	motor1.set_desired_angle(&motor1, 0);
	motor2.set_desired_angle(&motor2, 0);
	motor3.set_desired_angle(&motor3, 0);
	motor4.set_desired_angle(&motor4, 0);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	tf=1;
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
	MX_UART5_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart5, rx_buffer5, 1);
	testflag++;
	initStepMotor(&motor1, GPIOG, GPIO_PIN_7,GPIOG, GPIO_PIN_4);
	motor1.deg_step_ratio = 360.0/5000.0;
	//initStepMotor(&motor2, GPIOF, GPIO_PIN_4,GPIOE, GPIO_PIN_8);//original code
	initStepMotor(&motor2, GPIOC, GPIO_PIN_4,GPIOF, GPIO_PIN_5); //changed code(temporal)
	initStepMotor(&motor3, GPIOF, GPIO_PIN_10,GPIOE, GPIO_PIN_7);
	initStepMotor(&motor4, GPIOD, GPIO_PIN_14,GPIOD, GPIO_PIN_15);




	HAL_Delay(500);
	motor_pos_init();
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		////////////////////////////////TESTING//////////////////////

		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		////////////////////////////////////////////////////

		testflag_hair=(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)== GPIO_PIN_SET);
		//		testflag== 2;
		uart_decoder();
		Polling_Buttons();
		motor1.step_while(&motor1);
		motor2.step_while(&motor2);
		motor3.step_while(&motor3);
		motor4.step_while(&motor4);



		//		flagexti = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)){
			if(motor1_flag > 1){
				motor1_flag= motor1_flag -1 ;
			}
			else if (motor1_flag == 1 ) {
				motor1.current_angle_step = 1200;//1200 =>600
				motor1_flag = 0;
			}
		}
		//motor2.step_while(&motor2);
		//		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)){
		//			if(motor2_flag > 1){
		//				motor2_flag= motor2_flag - 1 ;
		//			}
		//			else if (motor2_flag == 1 ) {
		//				motor2.current_angle_step = 1500; //5000=> 500
		//				motor2_flag = 0;
		//			}
		//		}
		//motor3.step_while(&motor3);

		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)){
			if(motor3_flag > 1){
				motor3_flag= motor3_flag - 1 ;
			}
			else if (motor3_flag == 1 ) {
				motor3.current_angle_step = 1600;//8000=> 800
				motor3_flag = 0;
			}

			//motor4.step_while(&motor4);

		}if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)){
			if(motor4_flag > 1){
				motor4_flag= motor4_flag - 1 ;

			}
			else if (motor4_flag == 1 ) {
				motor4.current_angle_step = -2000;//5000=> 500
				motor4_flag = 0;

			}
		}

		if (systemRunning == 1 && !emergencyStop) {
			//sw_test();

			LED_Sequence_01();
			LED_Sequence_02();
			LED_Sequence_04();
			//add_port_test();
			//LED_Sequence_stop();
			//LED_Sequence_test();
			if ((HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_SET) && step==6  && test1 ==0)
			{
				hair_detect=1;

			}
			if (step2==13){
				hair_detect=0;
				sequence_01=1;
			}

			/*if (step!=7||step2==13)
	  	{
	  		hair_detect =0;

	  	}
	  	else if(step==7)
	  	{
	  		hair_detect =1; // hair detecting
	  		hair_detect_02 =1;

	  	}*/




		}

		if (emergencyStop) {
			Stop_All();
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12
			|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
			|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
			|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
			|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
			|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
			|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
			|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
			|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11
			|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pins : PE5 PE6 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PC13 PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PF0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : PF2 PF3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : PF4 PF5 PF10 PF12
                           PF13 PF14 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12
			|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
			|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
			|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PC4 PC6 PC7 PC8
                           PC9 PC10 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
			|GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE14 PE0 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
			|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB13 PB14 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
			|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
			|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PD11 PD12 PD13 PD14
                           PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
			|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PG4 PG7 PG10 PG11
                           PG13 PG15 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11
			|GPIO_PIN_13|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PD0 PD1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Emergency_Stop(void)
{
	Stop_All();
	currentSequence = 0;
	isRunning = 0;
}



void LED_Sequence_01(void)
{
	//static int step = 0;
	//	static int caseIntervals1 = 5000;

	if (HAL_GetTick() - lastTime1 >= caseIntervals1 && sequence_01==1)
	{
		lastTime1 = HAL_GetTick();
		motor1.step_speed=1;
		switch (step)
		{
		case 0: // All solV off
			/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 , GPIO_PIN_RESET);
			 */
			solV_5_OFF();
			solV_4_OFF();
			case_01_flag ++;
			caseIntervals1 = 300;
			break;
		case 1:
			motor1.set_desired_angle(&motor1, 00);
			solV_blow_Hair_ON();
			caseIntervals1 = 500;
			break;

		case 2:
			solV_4_ON();
			solV_blow_Hair_OFF();
			caseIntervals1 = 500;
			break;
		case 3: // solV with motor clip on
			solV_5_ON();;
			caseIntervals1 = 500;
			break;
		case 4:
			motor1.set_desired_angle(&motor1, -170);// motor go front

			caseIntervals1 = 3000;
			break;
		case 5:

			caseIntervals1 = 1;
			break;
		case 6:// solV with clip down
			if (hair_detect==1)
			{
				motor1.set_desired_angle(&motor1, -710);// motor go front
			}else
			{
				step=0;
				solV_5_OFF();solV_4_OFF();solV_6_OFF();
				motor1.set_desired_angle(&motor1, 0);
				motor1.step_speed=5;

				caseIntervals1 = 1000;
			}

			caseIntervals1 = 1500;
			break;
		case 7:// clip on #5 GPIO_PIN_12 onI
			solV_6_ON();
			caseIntervals1 = 500;
			break;
		case 8:// solV with limit switch down
			motor1.set_desired_angle(&motor1, -675);// motor go front
			sequence_01=0;
			sequence_02=1;
			caseIntervals1 = 1000;
			break;
		case 9:
			caseIntervals1 = 1;
			break;



		}

		step = (step + 1) % 10;
	}
}

/* LED_Sequence2 Function */
void LED_Sequence_02(void)
{
	if ((HAL_GetTick() - lastTime2) >= caseIntervals2&&sequence_02==1)
	{
		lastTime2 = HAL_GetTick();
		motor2.step_speed=25;
		motor3.step_speed=7;
		motor4.step_speed=40;
		switch (step2)
		{
		case 0:
			//motor4.set_desired_angle(&motor4, 65);

			caseIntervals2 = 10;
		case 1:

			solV_8_ON();
			caseIntervals2 = 500;
			break;
		case 2:

			motor2.set_desired_angle(&motor2, 0);

			motor2.step_speed = 250;

			solV_9_ON();

			if(hair_detect==1)
			{

			}
			else
			{
				sequence_02=0;
			}
			caseIntervals2 = 800;
			break;
		case 3:
			solV_8_OFF();
			solV_11_OFF();
			caseIntervals2 = 800;
			break;

		case 4:
			motor4.step_speed = 250;
			motor4.set_desired_angle(&motor4, 0); //70->0 (25.08.22)
			caseIntervals2 = 100;
			break;
		case 5:
			motor4.step_speed=250;
			caseIntervals2 = 1;
			break;
		case 6:
			motor1.set_desired_angle(&motor1, -710);
			motor1.step_speed=5;
			motor4.step_speed=250;
			caseIntervals2 = 800;
			break;
		case 7:
			solV_7_ON();
			caseIntervals2 = 500;
			motor4.step_speed=250;
			break;
		case 8:
			solV_5_OFF();
			solV_6_OFF();
			solV_4_OFF();

			//hair_detect=0;
			caseIntervals2 = 300;
			break;
		case 9:
			solV_8_ON();

			motor1.set_desired_angle(&motor1, 10);
			motor1.step_speed=1;
			caseIntervals2 = 1000;
			break;
		case 10:
			caseIntervals2 = 1;
			break;
		case 11:
			solV_9_OFF();
			solV_blow_ON();
			motor2.set_desired_angle(&motor2, hairang);
			caseIntervals2 = 900;
			break;
		case 12:
			solV_10_ON();
			caseIntervals2 = 100;
			break;
		case 13:
			solV_8_OFF();
			caseIntervals2 = 600;
			break;// sequence Checked now
		case 14:
			motor3.set_desired_angle(&motor3, 390);
			caseIntervals2 = 800;
			break;
		case 15:
			motor4.set_desired_angle(&motor4, 6); // 76->6 (25.08.22)
			motor4.step_speed=100;
			solV_10_OFF();
			caseIntervals2 = 500;
			break;
		case 16:
			motor2.set_desired_angle(&motor2, hairang+5);
			motor2.step_speed=100;
			caseIntervals2 =400;

			break;
		case 17:
			solV_11_ON();
			caseIntervals2 =300;

			break;
		case 18:

			motor2.set_desired_angle(&motor2, hairang+10);
			motor2.step_speed=100;
			caseIntervals2 =400;
			break;
		case 19:
			motor2.set_desired_angle(&motor2, hairang+5);
			motor2.step_speed=50;
			solV_11_OFF();
			solV_blow_OFF();
			caseIntervals2 = 400;
			break;
		case 20:
			motor4.set_desired_angle(&motor4, -65); // 5->-65 (25.08.22)motor D move 30 deg (80 -> 50)
			motor4.step_speed=100;
			caseIntervals2 = 550;
			break;
		case 21:
			motor2.set_desired_angle(&motor2, hairang-10);
			motor4.set_desired_angle(&motor4, -55); //15->-55(25.08.22)
			caseIntervals2 = 500;
			break;
		case 22:
			motor3.set_desired_angle(&motor3, 460); // motor C cw 60 deg
			caseIntervals2 = 300;
			break;
		case 23:
			motor2.set_desired_angle(&motor2, hairang+10);
			motor2.step_speed=100;
			caseIntervals2 = 300;
			motor3.set_desired_angle(&motor3, 400);//400 degree
			caseIntervals2 = 300;
			break;
		case 24:
			motor2.set_desired_angle(&motor2, hairang-20);
			motor4.set_desired_angle(&motor4, -55); //25->-45(25.08.22)//maximum 58 degree
			caseIntervals2 = 100;
			break;
		case 25:
			solV_8_ON();
			//motor3.set_desired_angle(&motor3, 360);
			caseIntervals2 = 500;
			break;
		case 26:
			solV_15_OFF();
			caseIntervals2 = 80;
			solV_16_ON();
			caseIntervals2 = 80;
			solV_18_ON();
			caseIntervals2 = 100;
			break;
		case 27:
			solV_17_ON();
			caseIntervals2 = 120;
			break;
		case 28:
			solV_18_OFF();
			caseIntervals2 = 300;
			break;
		case 29:
			solV_15_ON();

			solV_7_OFF();

			solV_11_ON();
			caseIntervals2 = 80;
			break;
		case 30:
			solV_16_OFF();
			caseIntervals2 = 80;
			solV_12_ON();
			caseIntervals2 = 80;
			solV_14_ON();
			caseIntervals2 = 80;
			break;
		case 31:
			solV_17_OFF();
			caseIntervals2 = 80;
			break;
		case 32:
			motor3.set_desired_angle(&motor3, 0);
			solV_8_ON();
			caseIntervals2 = 1;
			break;
		case 33:
			//solV_11_OFF();
			caseIntervals2 = 1;
			break;//hair brushing,sorting... head up!

		case 34:
			//motor4.set_desired_angle(&motor4, 70);
			//			motor2.set_desired_angle(&motor2, 0);
			motor2.step_speed = 250;
			motor2.current_angle_step = motor2.current_angle_step+200;
			motor2_flag = 0;
			motor4.step_speed = 250;
			motor4.current_angle_step = motor4.current_angle_step-200;
			motor4_flag = 0;

			sequence_04=1; //brushing sequence start

			caseIntervals2 = 100;
			uint8_t print[] = "next\n";
			HAL_UART_Transmit(&huart5, print, sizeof(print) - 1,
					HAL_MAX_DELAY);

			break;


		}

		step2 = (step2 + 1) % 35;
	}
}
void LED_Sequence_04(void)
{
	//static int step = 0;

	if (HAL_GetTick() - lastTime4 >= caseIntervals4 && sequence_04==1)
	{
		lastTime4 = HAL_GetTick();

		switch (step4)
		{
		case 0:
			solV_9_ON();
			caseIntervals4 = 120;
			break;
		case 1:

			caseIntervals4 = 1;
			break;
		case 2:

			caseIntervals4 = 1;
			break;
		case 3:

			caseIntervals4 = 1;
			break;
		case 4:
			solV_13_ON();
			caseIntervals4 = 80;
			break;
		case 5:

			caseIntervals4 = 1;
			break;
		case 6:
			solV_14_OFF();
			caseIntervals4 = 300;
			break;
		case 7:
			solV_12_OFF();
			caseIntervals4 = 80;
			break;
		case 8:
			solV_13_OFF();
			caseIntervals4 = 80;
			break;
		case 9:

			sequence_04=0;
			caseIntervals4 = 10;
			break;



		}

		step4 = (step4 + 1) % 10;
	}
}

void LED_Sequence_stop(void)
{
	if ((HAL_GetTick() - lastTime3) >= caseIntervals3)
	{
		lastTime3 = HAL_GetTick();
		switch (step3)
		{
		case 0:
			motor4.set_desired_angle(&motor4, 0);
			motor3.set_desired_angle(&motor3, 0);
			motor2.set_desired_angle(&motor2, 45);
			solV_7_OFF();
			solV_blow_OFF();
			caseIntervals3 = 3000;
			break;
		case 1:
			solV_8_ON();
			caseIntervals3 = 3000;
			break;
		case 2:
			solV_4_OFF();solV_5_OFF();solV_6_OFF();solV_7_OFF();
			solV_9_OFF();solV_10_OFF();solV_11_OFF();
			solV_12_OFF();solV_13_OFF();solV_14_OFF();solV_15_OFF();
			solV_16_OFF();solV_17_OFF();solV_18_OFF();

			caseIntervals3 = 2000;
			break;

		}

	}

}

void sw_test(void)
{
	if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_SET)
	{
		HAL_Delay(50);
		sw_test_01 ++;
	}
}

void add_port_test(void)
{
	//static int step = 0;
	//	static int caseIntervals1 = 5000;

	if (HAL_GetTick() - lastTime5 >= caseIntervals5 )
	{
		lastTime5 = HAL_GetTick();

		switch (step5)
		{
		case 0: // All solV off
			solV_Grab_OFF();
			solV_blow_ON();
			caseIntervals5 = 500;
			break;
		case 1: // All solV off
			solV_blow_OFF();
			solV_Grab_ON();
			caseIntervals5 = 1000;
			break;
		}

		step5 = (step5 + 1) % 2;
	}
}


//##############Sollenoid Valve Functions###############//
//------------For Hair extraction-----------//
void solV_1_ON(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);}
void solV_1_OFF(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);}
void solV_2_ON(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);}
void solV_2_OFF(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);}
void solV_3_ON(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);}
void solV_3_OFF(){HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);}
void solV_4_ON(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);}
void solV_4_OFF(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);}
void solV_5_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);}
void solV_5_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);}
void solV_6_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);}
void solV_6_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);}


//-----------For Hair Knotting-----------//
void solV_7_ON(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);}
void solV_7_OFF(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);}
void solV_8_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);}
void solV_8_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);}
//void solV_9_ON(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);}
//void solV_9_OFF(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);}

void solV_9_ON(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);}
void solV_9_OFF(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);}

void solV_10_ON(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);}
void solV_10_OFF(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);}
void solV_11_ON(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);}
void solV_11_OFF(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);}


//-----------For Hair Brushing-----------//
void solV_12_ON(){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);}
void solV_12_OFF(){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);}
void solV_13_ON(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);}
void solV_13_OFF(){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);}
void solV_14_ON(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);}
void solV_14_OFF(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);}
void solV_15_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);}
void solV_15_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);}
void solV_16_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);}
void solV_16_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);}
void solV_17_ON(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);}
void solV_17_OFF(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);}
void solV_18_ON(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);}
void solV_18_OFF(){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);}

void solV_blow_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);}
void solV_blow_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);}
void solV_blow_Hair_ON(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);}
void solV_blow_Hair_OFF(){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);}

void solV_Grab_ON(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);}
void solV_Grab_OFF(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
