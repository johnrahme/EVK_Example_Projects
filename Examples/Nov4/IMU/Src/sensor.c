/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "sensor.h"

/* USER CODE BEGIN Includes */
//accelerometer address
#define ACCEL_ADDR_WR 0x32
#define ACCEL_ADDR_RD 0x33

//accelerometer register map
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25

#define REFERENCE_A 0x26

#define STATUS_REG_A 0x27

#define OUT_X_L_A    0x28
#define OUT_X_H_A    0x29
#define OUT_Y_L_A    0x2A
#define OUT_Y_H_A    0x2B
#define OUT_Z_L_A    0x2C
#define OUT_Z_H_A    0x2D

#define FIFO_CTRL_REG_A		0x2E
#define FIFO_SRC_REG_A 		0x2F

#define INT1_CFG_A		0x30
#define INT1_SRC_A		0x31
#define INT1_THS_A		0x32
#define INT1_DURATION_A	0x33

#define INT2_CFG_A		0x34
#define INT2_SRC_A 		0x35
#define INT2_THS_A		0x36
#define INT2_DURATION_A 0x37

#define CLICK_CFG_A		0x38
#define CLICK_SRC_A		0x39
#define CLICK_THS_A		0x3A

#define TIME_LIMIT_A	0x3B
#define TIME_LATENCY_A	0x3C
#define TIME_WINDOW_A	0x3D


// magnetometer address
#define MAG_ADDR_WR 0x3C
#define MAG_ADDR_RD 0x3D
//magnetometer register map

#define CRA_REG_M	0x00
#define	CRB_REG_M	0x01
#define	MR_REG_M 	0x02

#define OUT_X_H_M	0x03
#define OUT_X_L_M	0x04
#define OUT_Z_H_M	0x05
#define OUT_Z_L_M	0x06
#define OUT_Y_H_M	0x07
#define OUT_Y_L_M	0x08

#define SR_REG_M	0x09

#define IRA_REG_M	0x0A
#define IRB_REG_M	0x0B
#define IRC_REG_M	0x0C

#define TEMP_OUT_H_M 0x31
#define TEMP_OUT_L_M 0x32

//gyroscope address
//#define GYRO_ADDR_WR 0xD4
//#define GYRO_ADDR_RD 0xD5
#define GYRO_ADDR_WR 0xD6
#define GYRO_ADDR_RD 0xD7

#define WHO_AM_I 0x0F

#define CTRL1 0x20
#define CTRL2 0x21
#define CTRL3 0x22
#define CTRL4 0x23
#define CTRL5 0x24


#define REFERENCE 0x25

#define OUT_TEMP  0x26
#define STATUS 	  0x27

#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

#define FIFO_CTRL   0x2E
#define FIFO_SRC	0x2F

#define IG_CFG		0x30
#define IG_SRC		0x31

#define IG_THS_XH   0x32
#define IG_THS_XL 	0x33
#define IG_THS_YH	0x34
#define IG_THS_YL	0x35
#define IG_THS_ZH	0x36
#define IG_THS_ZL	0x37

#define IG_DURATION 0x38
#define LOW_ODR		0x39





/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t tx_data_ACC[10];
uint8_t rx_data_ACC[30];

uint8_t tx_data_MAG[10];
uint8_t rx_data_MAG[30];

uint8_t tx_data_GYRO[10];
uint8_t rx_data_GYRO[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void sensorSetup(){

	/* USER CODE BEGIN 1 */
		uint8_t i;
	  /* USER CODE END 1 */
	int16_t   OUT_X_ACCEL, OUT_Y_ACCEL, OUT_Z_ACCEL;
	int16_t   OUT_X_MAG,OUT_Y_MAG,OUT_Z_MAG;
	int16_t   OUT_X_GYRO,OUT_Y_GYRO,OUT_Z_GYRO;
	  /* MCU Configuration----------------------------------------------------------*/

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	  HAL_Init();

	  /* Configure the system clock */
	  SystemClock_Config();

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_I2C2_Init();

	  /* USER CODE BEGIN 2 */


	  // Accelerometer
	   tx_data_ACC[0]=CTRL_REG1_A; // default 0x07
	   tx_data_ACC[1]=0x27;        //ODR3-ODR0=0010,Normal mode (10Hz)
	   HAL_I2C_Master_Transmit(&hi2c2,ACCEL_ADDR_WR,tx_data_ACC,2,1000);

	   tx_data_ACC[0]=CTRL_REG4_A; // default 0x00
	   tx_data_ACC[1]=0x38;        //FS1-FS0=11
	   HAL_I2C_Master_Transmit(&hi2c2,ACCEL_ADDR_WR,tx_data_ACC,2,1000);

	   // Magenetometer
	   tx_data_MAG[0]=CRA_REG_M;  //default 0x03
	     tx_data_MAG[1]=0x9C;       //TEMP_EN=1 , Temp Sensor enabled, DO2-DO0 = 111,220 Hz minimum data output rate
	     HAL_I2C_Master_Transmit(&hi2c2,MAG_ADDR_WR,tx_data_MAG,2,1000);

	  tx_data_MAG[0]=MR_REG_M;  //default 0x03
	  tx_data_MAG[1]=0x00;       //MD1-MD0 = 00, continuous conversion mode
	  HAL_I2C_Master_Transmit(&hi2c2,MAG_ADDR_WR,tx_data_MAG,2,1000);


	  //  GYROSCOPE

	//        tx_data_GYRO[0]=CTRL1;
	//        tx_data_GYRO[1]=0x00;
	//        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,2,1000);

	//        tx_data_GYRO[0]=CTRL5;
	//        tx_data_GYRO[1]=0x80;
	//        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,2,1000);
	//
	//        HAL_Delay(10);
	//
	//        tx_data_GYRO[0]=CTRL1;
	//        tx_data_GYRO[1]=0x09;
	//        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,2,1000);
	//
	        tx_data_GYRO[0]=CTRL1;  // default value 0x07
	        tx_data_GYRO[1]=0x0F;
	        tx_data_GYRO[2]=CTRL2;
	        tx_data_GYRO[3]=0x00;
	        tx_data_GYRO[4]=CTRL3;
	        tx_data_GYRO[5]=0x30;   // FS1-FS0=11, 2000 dps
	        tx_data_GYRO[6]=CTRL4;
	        tx_data_GYRO[7]=0x00;
	        tx_data_GYRO[8]=CTRL5;  //default value 0x00
	        tx_data_GYRO[9]=0x40;   //FIFO enable

	//                tx_data_GYRO[0]=CTRL1;
	//                tx_data_GYRO[1]=0xF;
	//                tx_data_GYRO[0]=CTRL2;
	//                tx_data_GYRO[1]=0xF;
	//                tx_data_GYRO[0]=CTRL3;
	//                tx_data_GYRO[1]=0xF;
	//                tx_data_GYRO[0]=CTRL4;
	//                tx_data_GYRO[1]=0xF;
	//                tx_data_GYRO[0]=CTRL5;
	//                tx_data_GYRO[1]=0xF;
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[0],2,1000);
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[2],2,1000);
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[4],2,1000);
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[6],2,1000);
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[8],2,1000);

	        tx_data_GYRO[0]=FIFO_CTRL;  //default value 0x00
	        tx_data_GYRO[1]=0x60;   //FIFO Control
	        HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[0],2,1000);


	//tx_data[1]=0x08;
	//HAL_I2C_Master_Transmit(&hi2c2,ACCELEMETER_ADDR_WR,tx_data,2,1000);

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
}
int main(void)
{


  while (1)
  {
  /* USER CODE END WHILE */
   //tx_data[0]=0x27;
   //tx_data[0]=CTRL_REG1_A;
   //tx_data[1]=0x17;
	    //tx_data[0]=CTRL_REG1_A;
	    //tx_data[1]=0x0F;
	   // tx_data[2]=CTRL_REG4_A;
	   // tx_data[3]=0x00;
	    //HAL_I2C_Master_Transmit(&hi2c2,ACCEL_ADDR_WR,tx_data,2,1000);
   tx_data_ACC[0]=(STATUS_REG_A|0x80);
 //  tx_data[0]=CTRL_REG1_A;
   HAL_I2C_Master_Transmit(&hi2c2,ACCEL_ADDR_WR,tx_data_ACC,1,1000);
   HAL_I2C_Master_Receive(&hi2c2,ACCEL_ADDR_RD,rx_data_ACC,7,1000);
  // printf("\n STATUS_REG_A = %u",rx_data_ACC[0]);

   OUT_X_ACCEL = (rx_data_ACC[2]<<8)|rx_data_ACC[1];
   OUT_Y_ACCEL = (rx_data_ACC[4]<<8)|rx_data_ACC[3];
   OUT_Z_ACCEL = (rx_data_ACC[6]<<8)|rx_data_ACC[5];



      printf("\n OUT_X_ACCEL =  %d",OUT_X_ACCEL);
      printf("\n OUT_Y_ACCEL =  %d",OUT_Y_ACCEL);
      printf("\n OUT_Z_ACCEL =  %d",OUT_Z_ACCEL);

   // MAGNETOMETER
        tx_data_MAG[0]=MR_REG_M;

        HAL_I2C_Master_Transmit(&hi2c2,MAG_ADDR_WR,tx_data_MAG,1,1000);
        HAL_I2C_Master_Receive(&hi2c2,MAG_ADDR_RD,rx_data_MAG,7,1000);
     //   printf("\n MR_REG_M = %d",rx_data_MAG[0]);
        OUT_X_MAG = (rx_data_MAG[1]<<8)|rx_data_MAG[2];
        OUT_Y_MAG =(rx_data_MAG[3]<<8)|rx_data_MAG[4];
        OUT_Z_MAG = (rx_data_MAG[5]<<8)|rx_data_MAG[6];


                   printf("\n OUT_X_MAG =  %d",OUT_X_MAG);
                   printf("\n OUT_Y_MAG =  %d",OUT_Y_MAG);
                   printf("\n OUT_Z_MAG =  %d",OUT_Z_MAG);

         tx_data_MAG[0]=TEMP_OUT_H_M|0x80;
		  HAL_I2C_Master_Transmit(&hi2c2,MAG_ADDR_WR,tx_data_MAG,1,1000);
		  HAL_I2C_Master_Receive(&hi2c2,MAG_ADDR_RD,rx_data_MAG,2,1000);
//		  printf("\n TEMP_OUT_MAG_H = %d",rx_data_MAG[0]);
//		  printf("\n TEMP_OUT_MAG_L = %d",rx_data_MAG[1]);
		  //int16_t TEMP_OUT_MAG;
		 // printf("\n TEMP_OUT_MAG= %d",(rx_data_MAG[0]<<4)|(rx_data_MAG[1]>>4));

                   ////GYROSCOPE
                             tx_data_GYRO[0]=WHO_AM_I;
							 HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,1,1000);
							 HAL_I2C_Master_Receive(&hi2c2,GYRO_ADDR_RD,rx_data_GYRO,1,1000);
						//	 printf("\n GYRO device identification register = %x",rx_data_GYRO[0]);

							 //tx_data_GYRO[0]=(CTRL1|0x80);

//							 tx_data_GYRO[0]=(OUT_X_L|0x80);
//
//							 HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,&tx_data_GYRO[0],1,1000);
//							 HAL_I2C_Master_Receive(&hi2c2,GYRO_ADDR_RD,rx_data_GYRO,6,1000);

//							    for(i=0;i<6;i++){
//							 	   printf("\n %d %d",i,rx_data_GYRO[i]);
//							    }


//                          tx_data_GYRO[0]=OUT_TEMP;
//					 HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,1,1000);
//					 HAL_I2C_Master_Receive(&hi2c2,GYRO_ADDR_RD,rx_data_GYRO,1,1000);
//					 int8_t TEMPERATURE =rx_data_GYRO[0];
//					 printf("\n TEMPERATURE = %d",TEMPERATURE);
//
//        	   tx_data_GYRO[0]=STATUS;
							 tx_data_GYRO[0]=(STATUS|0x80);
			  HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,1,1000);
			  HAL_I2C_Master_Receive(&hi2c2,GYRO_ADDR_RD,rx_data_GYRO,7,1000);
			//  printf("\n STATUS = %d",rx_data_GYRO[0]);
			  OUT_X_GYRO = (rx_data_GYRO[2]<<8)|rx_data_GYRO[1];
			  OUT_Y_GYRO = (rx_data_GYRO[4]<<8)|rx_data_GYRO[3];
			  OUT_Z_GYRO = (rx_data_GYRO[6]<<8)|rx_data_GYRO[5];


						 printf("\n OUT_X_GYRO =  %d",OUT_X_GYRO);
						 printf("\n OUT_Y_GYRO =  %d",OUT_Y_GYRO);
						 printf("\n OUT_Z_GYRO =  %d",OUT_Z_GYRO);



   HAL_Delay(1000);

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
