/* ---- USER CODE: Header ---- */
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

/* ---- USER CODE: Includes ---- */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h> // For atoi()
#include "ertc-datalogger.h"
#include "SX1509_Registers.h"

/* ---- USER CODE: PD ---- */
//#define	HAL_TIMEOUT		1
#define TS	0.01

#define VBATT	12.0
#define V2DUTY	((float)(TIM8_ARR_VALUE+1)/VBATT)
#define DUTY2V	((float)VBATT/(TIM8_ARR_VALUE+1))

#define RPM2RADS	2*M_PI/60

#define P_gain 0.871
#define I_gain 7.5563

#define SX1509_I2C_ADDR1 0x3E //	SX1509 Proxy Sensors I2C address
#define SX1509_I2C_ADDR2 0x3F //	SX1509 Keypad I2C address

#define HAL_TIMEOUT 100
#define I2C_TIMEOUT 200

#define MAX_DIGITS 3  // Maximum 3-digit frequency (e.g., "125#")

/* ---- USER CODE: PV ---- */
struct ertc_dlog logger;

const char keypad[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/* ---- USER CODE: 0 ---- */

char decode_key(uint8_t col_data, uint8_t row_data) {
  int row = -1, col = -1;

  // Identify row
  switch (row_data) {
    case 0xFE: row = 3; break;  // Row 1 (bit 0 changed)
    case 0xFD: row = 2; break;  // Row 2 (bit 1 changed)
    case 0xFB: row = 1; break;  // Row 3 (bit 2 changed)
    case 0xF7: row = 0; break;  // Row 4 (bit 3 changed)
}

  // Identify column
  switch (col_data) {
    case 0xFE: col = 0; break;  // Column 1
    case 0xFD: col = 1; break;  // Column 2
    case 0xFB: col = 2; break;  // Column 3
    case 0xF7: col = 3; break;  // Column 4
}

  // Return the corresponding key if valid row and column detected
  if (row != -1 && col != -1) {
      return keypad[row][col];
  }

  return '?';  // Unknown key
}

char inputBuffer[MAX_DIGITS + 1] = {0};  // Buffer to store user input
int myindex = 0;  // Tracks number of entered digits

struct datalog {
	float w1, w2;
	float u1, u2;
} data;


// reads the encoders and calculate the motor speed
float ReadCurrentSpeed(int motorNumber){
	uint32_t CurrentCount = 0;
	int32_t DiffCount = 0;
	static uint32_t PreviousCount = 0;

	// get diff count for specified motor htim3 for motor 1 and htim4 for motor 2
	switch (motorNumber){
		case 1:
			CurrentCount = __HAL_TIM_GET_COUNTER(&htim3);

			// calculate diff count of motor 1
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			 {
				 /* check for counter underflow */
				 if (CurrentCount <= PreviousCount) // Case of no underflow
					 DiffCount = CurrentCount- PreviousCount;
				 else // Case of underflow
					 DiffCount =-((TIM3_ARR_VALUE+1)- CurrentCount)- PreviousCount;
			 }
			 else
			 {
				 /* check for counter overflow */
				 if (CurrentCount >= PreviousCount) // Case of no overflow
					 DiffCount = CurrentCount- PreviousCount;
				 else	// Case of overflow
					 DiffCount = ((TIM3_ARR_VALUE+1)- PreviousCount) + CurrentCount;
			 }
			PreviousCount = CurrentCount; // Storing the current value

			break;
		case 2:
			CurrentCount = __HAL_TIM_GET_COUNTER(&htim4);

			// calculate diff count of motor 2
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
			 {
				 /* check for counter underflow */
				 if (CurrentCount <= PreviousCount) // Case of no underflow
					 DiffCount = CurrentCount- PreviousCount;
				 else // Case of underflow
					 DiffCount =-((TIM4_ARR_VALUE+1)- CurrentCount)- PreviousCount;
			 }
			 else
			 {
				 /* check for counter overflow */
				 if (CurrentCount >= PreviousCount) // Case of no overflow
					 DiffCount = CurrentCount- PreviousCount;
				 else	// Case of overflow
					 DiffCount = ((TIM4_ARR_VALUE+1)- PreviousCount) + CurrentCount;
			 }
			PreviousCount = CurrentCount; // Storing the current value
			break;
	}

	/*
	// Calculating speed from pulse count
	/* We are using 4x encoder mode this means that Pulse-Per-Round for the wheel is 3840
	 * So the number of revolutions is TIM3_DiffCount/3840. To Calculate speed we should
	 * divide the number of revolutions by the sampling time Ts to get the speed in rps
	 * and then we can convert it to rpm by multiplying by 60
	 *
	 */

	float currentSpeed = 0;
	currentSpeed = ((float)DiffCount /(float)TIM3_ARR_VALUE)/ TS * 60.0; // Current speed in rpm

	return currentSpeed;
}

float clamp(float Output_temp){
	  // saturating the output
	 if(Output_temp > 6.0) Output_temp = 6.0;
	 if(Output_temp <-6.0) Output_temp = -6.0;
	 return Output_temp;
}

static float I_term = 0;
static float I_term_1 = 0;
static float I_term_2 = 0;

// calculate the duty based on the current speed and reference speed using a pi controller
float calculateControlSignal(float currentSpeed, float referenceSpeed, float* I_term, int motorNumber){
	 // PI Controller
	 float error = (referenceSpeed - currentSpeed)*RPM2RADS; //rad/sec
	 float P_term = P_gain*error;

   if(motorNumber == 1){
	 *I_term_1 = *I_term_1 + I_gain*TS*error;
	 float Output_temp = P_term+*I_term_1; // volt
   }
	 //*I_term = *I_term + I_gain*TS*error;
   else if (motorNumber == 2){
	 *I_term_2 = *I_term_2 + I_gain*TS*error;
	 float Output_temp = P_term+*I_term_2; // volt
   } 

	 //float Output_temp = P_term+*I_term; // volt
	 float Output_clamped = clamp(Output_temp);

	 data.u1 = Output_temp;
   // Anti wind-up
	 bool is_saturated = (Output_clamped != Output_temp);
	 bool worsens_saturation = (error * Output_temp > 0);
	  // saturating the output
	 //if(Output_clamped > 6.0) Output_clamped = 6.0;
	 //if(Output_clamped <-6.0) Output_clamped = -6.0;

	 if((is_saturated)&&(worsens_saturation)&&(motorNumber == 1)){
		 *I_term_1 = *I_term_1 - I_gain*TS*error;
	 }
   else if ((is_saturated)&&(worsens_saturation)&&(motorNumber == 2))
   {
    *I_term_2 = *I_term_2 - I_gain*TS*error;
   }
   



	 return Output_clamped;
}

void ForwardCoast (int32_t motorNumber, int32_t duty){
	switch (motorNumber){
		case 1:
			if (duty >= 0) { // rotate forward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)duty);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)-duty);
			 }
		case 2:
			if (duty >= 0) { // rotate forward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)duty);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)-duty);
			 }
		}
}
void ForwardBreak (int32_t motorNumber, int32_t duty){
	switch (motorNumber){
		case 1:
			if (duty >= 0) { // rotate forward
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)TIM8_ARR_VALUE);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(TIM8_ARR_VALUE- duty));
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)(TIM8_ARR_VALUE - (-duty)));
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)TIM8_ARR_VALUE);
			 }
		case 2:
			if (duty >= 0) { // rotate forward
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)TIM8_ARR_VALUE);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(TIM8_ARR_VALUE- duty));
			 } else { // rotate backward
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(TIM8_ARR_VALUE - (-duty)));
			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)TIM8_ARR_VALUE);
			 }
		}
}

float referenceSignal = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	static int kLed = 0;



	/* Speed ctrl routine */
	if(htim->Instance == TIM6)
	{
		// Read the speed from the motors
		float currentSpeed1 = ReadCurrentSpeed(1);
		float currentSpeed2 = ReadCurrentSpeed(2);

		float referenceSignal1 = 60;
		float referenceSignal2 = 60;

		float Output_1 = calculateControlSignal(currentSpeed1, referenceSignal1, &I_term_1, 1);
		float Output_2 = calculateControlSignal(currentSpeed2, referenceSignal2, &I_term_2, 2);

		// Set the correct PWM Value
    int32_t duty1;
    int32_t duty2;
    /* calculate duty properly */
		duty1 = Output_1*V2DUTY;
    duty2 = Output_2*V2DUTY;


		ForwardBreak(1, duty1);
    ForwardBreak(2, duty2;)
		//controlMotor(2,duty2);


     	/*	Prepare data packet */
		data.w1 = referenceSignal1 ;

		data.w2 = currentSpeed1;
		//data.u1 = ;
		data.u2 = Output_1;

		ertc_dlog_send(&logger, &data, sizeof(data));

		// Indicate that the program is running
		if(++kLed >= 10)
		{
			kLed = 0;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
}



// configuration code
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
      //recognizes and correctly handles the keypad interrupts
      if (GPIO_Pin == GPIO_PIN_4) { // Check if the interrupt is from PF4
        uint8_t key_data_1, key_data_2;

        // Read the column data
        if (HAL_I2C_Mem_Read(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_DATA_1, 1, &key_data_1, 1, I2C_TIMEOUT) != HAL_OK) {
            printf("Error reading REG_KEY_DATA_1\n");
            return;
        }

        // Read the row data
        if (HAL_I2C_Mem_Read(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_DATA_2, 1, &key_data_2, 1, I2C_TIMEOUT) != HAL_OK) {
            printf("Error reading REG_KEY_DATA_2\n");
            return;
        }

        // Decode and print the key
        char key = decode_key(key_data_1, key_data_2);

        if (key >= '0' && key <= '9') {
          if (myindex < MAX_DIGITS) {
              inputBuffer[myindex++] = key;  // Store digit
          }
        }
        else if (key == '#') {  // End input on `#`
            inputBuffer[myindex] = '\0';  // Null-terminate string
            referenceSignal = (float)atoi(inputBuffer);  // Convert to integer

            // saturate reference signal
            if(referenceSignal > 160) referenceSignal = 160.0;
            if(referenceSignal < -160) referenceSignal = -160.0;

            // Reset buffer for next input
            memset(inputBuffer, 0, sizeof(inputBuffer));
            myindex = 0;
        }
    }
}

/* ---- USER CODE: 1 ---- */
  uint8_t data;
  HAL_StatusTypeDef status;

/* ---- USER CODE: 2 ---- */

  logger.uart_handle = huart3; // for serial
  //logger.uart_handle = huart2; // for wifi

  /* Reset LCD */
  HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  /* Disable EXTI4_IRQ during SX1509 initialization */
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    ITM_SendChar('h');

    /* Software reset */
    data = 0x12;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    data = 0x34;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    HAL_Delay(100);

    /* Set KeyPad scanning engine */

    /* Set RegClock to 0x40 (enable internal oscillator; 2MHz freq) */
    data = 0x40;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_CLOCK, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank A RegDir to 0xF0 (IO[0:3] as out) */
    data = 0xF0;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegDir to 0x0F (IO[8:11] as in) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank A RegOpenDrain to 0x0F (IO[0:3] as open-drain outputs) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_OPEN_DRAIN_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegPullup to 0x0F (pull-ups enabled on inputs IO[8:11]) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_PULL_UP_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set Bank B RegDebounceEnable to 0x0F (enable debouncing on IO[8:11]) */
    data = 0x0F;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_ENABLE_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegDebounceConfig to 0x05 (16ms debounce time) */
    data = 0x05;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_CONFIG, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegKeyConfig1 to 0x7D (8s auto-sleep; 32ms scan time per row) */
    data = 0x7D;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_1, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegKeyConfig2 to 0x1B (4 rows; 4 columns) */
    data = 0x1B;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_2, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Enable EXTI4_IRQ after SX1509 initialization */
    HAL_Delay(100);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* Disable EXTI2_IRQ during SX1509 initialization */
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);

  /* Software reset */
    data = 0x12;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    data = 0x34;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    HAL_Delay(100);

    /* Set RegDirA to 0xFF (all IO of Bank A configured as inputs) */
    data = 0xFF; // 0 = out; 1 = in
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegDirB to 0xFF (all IO of Bank B configured as inputs) */
    data = 0xFF; // 0 = out; 1 = in
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegInterruptMaskA to 0x00 (all IO of Bank A will trigger an interrupt) */
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_INTERRUPT_MASK_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegSenseHighA to 0xAA (IO[7:4] of Bank A will trigger an interrupt on falling edge) */
    data = 0xAA;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_HIGH_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Set RegSenseLowA to 0xAA (IO[3:0] of Bank A will trigger an interrupt on falling edge) */
    data = 0xAA;
    status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_LOW_A, 1, &data, 1, I2C_TIMEOUT);
    if (status != HAL_OK)
      printf("I2C communication error (%X).\n", status);

    /* Enable EXTI2_IRQ after SX1509 initialization */
    HAL_Delay(100);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);




  /* Start encoders timers */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* Start servomotors PWM (avoid floating inputs to servomotors) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* Start motor PWM */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* Start speed ctrl ISR */
  HAL_TIM_Base_Start_IT(&htim6);


/* ---- USER CODE: WHILE ---- */
  while (1)
  {

/* ---- USER CODE: 3 ---- */
	  ertc_dlog_update(&logger);

  }

/* ---- USER CODE: Error_Handler_Debug ---- */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  static uint32_t kLed = 0;
	  if(++kLed >= 1000)
	  {
		  kLed = 0;
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  }
  }

/* ---- USER CODE: 6 ---- */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
