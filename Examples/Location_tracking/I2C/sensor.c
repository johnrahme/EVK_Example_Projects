/**
 ******************************************************************************
 * @file OptimizedI2Cexamples/src/main.c
 * @author  MCD Application Team
 * @version  V4.0.0
 * @date     06/18/2010
 * @brief  Main program body
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "I2CRoutines.h"
//#include <stdio.h>
//#include "misc.h"   /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

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



/** @addtogroup Optimized I2C examples
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;


///* Buffer of data to be received by I2C1 */   // saroj 25/11/2016
//uint8_t Buffer_Rx1[255];
///* Buffer of data to be transmitted by I2C1 */
//uint8_t Buffer_Tx1[255] = {0x5, 0x6,0x8,0xA};


uint8_t tx_data_ACC[10];
uint8_t rx_data_ACC[30];

uint8_t tx_data_MAG[10];
uint8_t rx_data_MAG[30];

uint8_t tx_data_GYRO[10];
uint8_t rx_data_GYRO[30];


extern __IO uint8_t Tx_Idx1 , Rx_Idx1;
extern __IO uint8_t Tx_Idx2 , Rx_Idx2;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void NVIC_Configuration(void);
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Main program
 * @param  None
 * @retval : None
 */
void initSensor(void)

{

	//  NVIC_Configuration();   // saroj 25/11/2016
	I2C_LowLevel_Init(I2C2);

	// Accelerometer

	tx_data_ACC[0]=CTRL_REG1_A; // default 0x07
	tx_data_ACC[1]=0x27;        //ODR3-ODR0=0010,Normal mode (10Hz)
	I2C_Master_BufferWrite(I2C2,tx_data_ACC,2,Polling, ACCEL_ADDR_WR);
	tx_data_ACC[0]=CTRL_REG4_A; // default 0x00
	tx_data_ACC[1]=0x38;        //FS1-FS0=11
	I2C_Master_BufferWrite(I2C2,tx_data_ACC,2,Polling, ACCEL_ADDR_WR);

	// Magenetometer
	tx_data_MAG[0]=CRA_REG_M;  //default 0x03
	tx_data_MAG[1]=0x9C;       //TEMP_EN=1 , Temp Sensor enabled, DO2-DO0 = 111,220 Hz minimum data output rate

	I2C_Master_BufferWrite(I2C2,tx_data_MAG,2,Polling, MAG_ADDR_WR);

	tx_data_MAG[0]=MR_REG_M;  //default 0x03
	tx_data_MAG[1]=0x00;       //MD1-MD0 = 00, continuous conversion mode
	I2C_Master_BufferWrite(I2C2,tx_data_MAG,2,Polling, MAG_ADDR_WR);


	//  GYROSCOPE
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

	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[0],2,Polling, GYRO_ADDR_WR);
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[2],2,Polling, GYRO_ADDR_WR);
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[4],2,Polling, GYRO_ADDR_WR);
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[6],2,Polling, GYRO_ADDR_WR);
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[8],2,Polling, GYRO_ADDR_WR);

	tx_data_GYRO[0]=FIFO_CTRL;  //default value 0x00
	tx_data_GYRO[1]=0x60;   //FIFO Control
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[0],2,Polling, GYRO_ADDR_WR);


}
int updateSensor()
{
	int i,j;
	//I2C_Master_BufferWrite(I2C2, Buffer_Tx1,120,Interrupt, 0x28);
	//Buffer_Tx2[4]=(STATUS_REG_A|0x80);

	//I2C_Master_BufferWrite(I2C2,Buffer_Tx2,1,Polling, ACCEL_ADDR_WR);
	//	    Buffer_Tx2[0]=CTRL_REG1_A; // default 0x07
	//	    Buffer_Tx2[1]=0x27;        //ODR3-ODR0=0010,Normal mode (10Hz)
	//
	//	    Buffer_Tx2[2]=CTRL_REG4_A; // default 0x00
	//	    Buffer_Tx2[3]=0x38;        //FS1-FS0=11
	//	    I2C_Master_BufferWrite(I2C2,Buffer_Tx2,4,Polling, ACCEL_ADDR_WR);
	tx_data_ACC[0]=(STATUS_REG_A|0x80);
	//Buffer_Tx2[0]=STATUS_REG_A;
	I2C_Master_BufferWrite(I2C2,tx_data_ACC,1,Polling, ACCEL_ADDR_WR);
	//	    I2C_Master_BufferRead(I2C2,Buffer_Rx2,1,Polling, ACCEL_ADDR_RD);
	//	    printf("Buffer_Rx2[0]=%d \n",Buffer_Rx2[0]);
	// printf("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n");

	//	    Buffer_Tx2[0]=WHO_AM_I;
	//	    I2C_Master_BufferWrite(I2C2,Buffer_Tx2[0],1,Polling, GYRO_ADDR_WR);
	//	     I2C_Master_BufferRead(I2C2,&Buffer_Rx2[0],1,Polling, GYRO_ADDR_RD);
	//	    	    printf("Buffer_Rx2[0]=%d \n",Buffer_Rx2[0]);

	//	    Buffer_Tx2[0]=OUT_X_L_A;
	//	    I2C_Master_BufferWrite(I2C2,&Buffer_Tx2[0],1,Polling, ACCEL_ADDR_WR);
	//	    I2C_Master_BufferRead(I2C2,&Buffer_Rx2[0],1,Polling, ACCEL_ADDR_RD);
	//	    printf("Buffer_Rx2[0]=%d \n",Buffer_Rx2[0]);

	// printf("Buffer_Rx2[1]=%d \n",Buffer_Rx2[1]);
	//   for(i=7;i>0;i--){
	// I2C_Master_BufferRead(I2C2,&Buffer_Rx2[i],1,Polling, ACCEL_ADDR_RD);
	//      printf("Buffer_Rx2[%d]= %d\n",i,Buffer_Rx2[i]);
	//  }
	I2C_Master_BufferRead(I2C2,&rx_data_ACC,7,Polling, ACCEL_ADDR_RD);
	//     for(i=7;i>0;i--){
	//    	// I2C_Master_BufferRead(I2C2,&Buffer_Rx2[i],1,Polling, ACCEL_ADDR_RD);
	//         printf("Buffer_Rx2[%d]= %d\n",i,Buffer_Rx2[i]);
	//     }

	OUT_X_ACCEL = (rx_data_ACC[2]<<8)|rx_data_ACC[1];
	OUT_Y_ACCEL = (rx_data_ACC[4]<<8)|rx_data_ACC[3];
	OUT_Z_ACCEL = (rx_data_ACC[6]<<8)|rx_data_ACC[5];
	//printf("\n OUT_X_ACCEL =  %d",OUT_X_ACCEL);
	//printf("\n OUT_Y_ACCEL =  %d",OUT_Y_ACCEL);
	//printf("\n OUT_Z_ACCEL =  %d",OUT_Z_ACCEL);

	// MAGNETOMETER

	tx_data_MAG[0]=MR_REG_M;


	I2C_Master_BufferWrite(I2C2,tx_data_MAG,1,Polling, MAG_ADDR_WR);
	I2C_Master_BufferRead(I2C2,rx_data_MAG,7,Polling, MAG_ADDR_RD);

	//   printf("\n MR_REG_M = %d",rx_data_MAG[0]);
	OUT_X_MAG = (rx_data_MAG[1]<<8)|rx_data_MAG[2];
	OUT_Y_MAG =(rx_data_MAG[3]<<8)|rx_data_MAG[4];
	OUT_Z_MAG = (rx_data_MAG[5]<<8)|rx_data_MAG[6];


	//printf("\n OUT_X_MAG =  %d",OUT_X_MAG);
	//printf("\n OUT_Y_MAG =  %d",OUT_Y_MAG);
	//printf("\n OUT_Z_MAG =  %d",OUT_Z_MAG);

	////GYROSCOPE
	//						tx_data_GYRO[0]=WHO_AM_I;
	//						I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[0],1,Polling, GYRO_ADDR_WR);
	//						I2C_Master_BufferRead(I2C2,&rx_data_GYRO[0],1,Polling, GYRO_ADDR_RD);
	//				        printf("\n GYRO device identification register = %x",rx_data_GYRO[0]);


	tx_data_GYRO[0]=(STATUS|0x80);
	//HAL_I2C_Master_Transmit(&hi2c2,GYRO_ADDR_WR,tx_data_GYRO,1,1000);
	I2C_Master_BufferWrite(I2C2,&tx_data_GYRO[0],1,Polling, GYRO_ADDR_WR);
	//  HAL_I2C_Master_Receive(&hi2c2,GYRO_ADDR_RD,rx_data_GYRO,7,1000);
	I2C_Master_BufferRead(I2C2,&rx_data_GYRO[0],7,Polling, GYRO_ADDR_RD);
	//  printf("\n STATUS = %d",rx_data_GYRO[0]);
	OUT_X_GYRO = (rx_data_GYRO[2]<<8)|rx_data_GYRO[1];
	OUT_Y_GYRO = (rx_data_GYRO[4]<<8)|rx_data_GYRO[3];
	OUT_Z_GYRO = (rx_data_GYRO[6]<<8)|rx_data_GYRO[5];


	//printf("\n OUT_X_GYRO =  %d",OUT_X_GYRO);
	//printf("\n OUT_Y_GYRO =  %d",OUT_Y_GYRO);
	//printf("\n OUT_Z_GYRO =  %d",OUT_Z_GYRO);

	//for(j=0;j<1000000;j++);
}

/* Use I2C1 as Slave */
/*! When using Slave with DMA, uncomment //#define SLAVE_DMA_USE in the stm32f10x_it.c file.*/
/*I2C_Slave_BufferReadWrite(I2C1, DMA);

 while(1); */

/**
 * @brief  Configures NVIC and Vector Table base location.
 * @param  None
 * @retval : None
 */
//void NVIC_Configuration(void)
//{
//
//    /* 1 bit for pre-emption priority, 3 bits for subpriority */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//
//    NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
//    NVIC_EnableIRQ(I2C1_EV_IRQn);
//
//    NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
//    NVIC_EnableIRQ(I2C1_ER_IRQn);
//
//
//    NVIC_SetPriority(I2C2_EV_IRQn, 0x00);
//    NVIC_EnableIRQ(I2C2_EV_IRQn);
//
//    NVIC_SetPriority(I2C2_ER_IRQn, 0x01);
//    NVIC_EnableIRQ(I2C2_ER_IRQn);
//
//}


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
