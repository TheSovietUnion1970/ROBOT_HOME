/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
int dirPinX = 2 ;
int stepPinX = 3 ;
int dirPinY = 12 ;
int stepPinY = 11 ;
int dirPinZ = 7 ;
int stepPinZ = 6 ;
	
//float l1=17.6,l2=15,l3=17;
float l1=8.4,l2=25.4,l3=19.7;
//float X = 8.14545,Y = 37.33333333 ,Z = 13.4; // t? s? truy?n
float X = 8.14545,Y = 220/20 ,Z = 36/15; // t? s? truy?n
double PI=3.1415;
float micro = 1.8/16;
uint32_t cc;

int pulseMax = 4; //print to monitor
int pulseMid = 2; //print to monitor
int pulseMin = 0; //print to monitor

float temp1_mem=0, temp2_mem=0, temp3_mem=0;

float dpX, dpY, dpZ;

float posX, posY, posZ;
int Time=600, Update;

float Xung;

float nStepMax;

uint32_t y;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void moveXYZ(long nStepX, int stepPinX, int dirX, long nStepY, int stepPinY, int dirY, long nStepZ, int stepPinZ, int dirZ);
void MovePosition(float Px, float Py, float Pz, float *temp1_mem, float *temp2_mem, float *temp3_mem);
void Move_angle2(int Do_chess, int Dir_chess);
void Move_angle3(int Do_chess, int Dir_chess);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IO(){
	RCC->APB2ENR|=1<<2;
	GPIOA->CRL|= (3<<8)|(3<<12)|(3<<24)|(3<<28);
	GPIOA->CRH|= (3<<12)|(3<<16);
	
	//led PC13
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH |= 1<<20;
}


void init_timer2(void)
{

	//	TIMER
	RCC->APB1ENR |= (1<<0);   //clock for timer2
  TIM2->PSC = 7;          // Prescale set
  TIM2->ARR = 0xffff-1;        // ARR set
  TIM2->CR1 |= (1<<0);      // enable timer 2
	TIM2->EGR |= (1<<0);  //update data
	// CNT - ms
	
  //enable time 2 interrupt
  //TIM2->DIER |= (1<<0);  //timer INt enable
  //NVIC->ISER[0] |= (1<<28);  // enable int timer 2 in NVIC
  // neu k dung ngat --> bo 2 gang tren
}

void delay_us(int us){
	TIM2->CNT=0;
	while(TIM2->CNT<us);
}

void delay_ms(int ms){
	for(int i=0; i<ms; i++){
		delay_us(1000);
	}
}


void UART(){
	// enable clock for UASRT1
	RCC->APB2ENR|= 1<<14;
	// enable clock for GPIOA
	RCC->APB2ENR|=(1<<2);
	
	//A9-output-alternate function, A10-input floating
	GPIOA->CRH |= (9<<4)|(4<<8);
	
	//USART1->BRR= (104<<4)|(3<<0); //9600  
	USART1->BRR = (52<<4)|(1<<0);
	//enable all
	USART1->CR1|= (1<<2)|(1<<3)|(1<<13); 

	// interrupt
	USART1->CR1|= (1<<5); 
	NVIC_EnableIRQ(USART1_IRQn);
}


void send_char (char data)
{
	while ((USART1->SR & (1<<6)) == 0){}   //wait for TC bit
  USART1->DR = data;
}

void send_string (char *s)
{
	while(*s) send_char(*s++);
}

char data_receive[200];
//uint16_t i;
volatile int j, check;
int state0, state1;
int bb=0;
void Blink(){
	for (int y = 0; y<3; y++){
	  GPIOC->ODR|=1<<13;
	  HAL_Delay(300);
	  GPIOC->ODR&=~(1<<13);
	  HAL_Delay(300);
	}
}
void USART1_IRQHandler(void)
{
	if (USART1->SR & (1<<5)) // kiem tra ngat nhan RX
	{
		
		data_receive[j++] = USART1->DR;
		//if (i == 2)
		//len = strlen(USART2->DR);
		if (USART1->DR == '\n')
		{
			data_receive[strcspn(data_receive, "\n")] = '\0';
			check = 1;
			j = 0;
			//Blink();
			//state0 = strcmp(data_receive, "turn 1");
			//state1 = strcmp(data_receive, "turn 0");
			
			//if (state0 == 0 && state1 != 0) {GPIOC->ODR &=~ (1<<13);}
			//if (state0 != 0 && state1 == 0) {GPIOC->ODR |= (1<<13);}
			
			//sscanf(data_receive,"%f,%f,%f", &posX, &posY, &posZ);
			///MovePosition(posX, posY, posZ, &temp1_mem, &temp2_mem, &temp3_mem);
			//sscanf("%f-%f-%f", &posX, &posY, &posZ);
			//send_string(data_receive);
			//memset(data_receive, 0, sizeof(data_receive));  // Clear the buffer
		}
		
		
		//if (USART1->DR == '1') GPIOC->ODR |= (1<<13);
		//if (USART1->DR == '0') GPIOC->ODR &=~ (1<<13);
		//else GPIOC->ODR &=~ (1<<13);
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
  /* USER CODE BEGIN 2 */
  IO();
	init_timer2();
	UART();
	//GPIOA->ODR|=(1<<11)|(1<<12);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     			 //delay_ms(300);
			 //bb++;
		/*
		for (uint32_t j=0; j<8000; j++){
		 GPIOC->ODR^=1<<13;
		 delay_us(500);
			bb++;
		}
		bb=0;
		*/


    if (check == 1){
			//char msg[200];
			sscanf(data_receive,"%f,%f,%f", &posX, &posY, &posZ);
			if (posX == 0 && posY == 0 && posZ == 0) {
				posX = 2.185;
				posY = 0;
				posZ = 21.12;
			}
			else if (posX == 0 && posY == 0 && posZ == 1) {
				posX = 19.70;
				posY = 0;
				posZ = 33.80;
			}
			MovePosition(posX, posY, posZ, &temp1_mem, &temp2_mem, &temp3_mem);
			check = 0;
		}
		//Move_angle3(90,0);
		//break;
//37.66,00.00,26.36
//19.70,00.00,33.80
//2.185,00.00,21.12
//32.40,00.00,30.39
		
		// "32.40-00.00-30.39" --> point1
// "19.70-00.00-33.80" --> point0
// "37.66-00.00-26.36" --> point2
// "2.185-00.00-21.12" --> point0.0
		//MovePosition(2.185, 0, 21.12, &temp1_mem, &temp2_mem, &temp3_mem);
		//break;
		//Blink();
		//MovePosition(37.66, 0, 26.36, &temp1_mem, &temp2_mem, &temp3_mem);
		//break;
		//Blink();
		//MovePosition(40, 0, 25, &temp1_mem, &temp2_mem, &temp3_mem);
		//Blink();
		//MovePosition(19.70, 0, 33.80, &temp1_mem, &temp2_mem, &temp3_mem);
		//Blink();
		

		//HAL_Delay(300);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void moveXYZ(long nStepX, int stepPinX, int dirX, long nStepY, int stepPinY, int dirY, long nStepZ, int stepPinZ, int dirZ) {

  int num1=0, num2=0, num3=0;
  //float nStepMax;
  float nStepMid;
  float nStepMin;
  int stepPinMax;
  int stepPinMin;
  int stepPinMid;

  Update=0;
	Time = 600;
  float ratio = 0, Current=0;
	float a,b;

if (nStepZ < nStepY && nStepY < nStepX) {
  nStepMax = nStepX;
  nStepMid = nStepY;
  nStepMin = nStepZ;
  stepPinMax = stepPinX;
  stepPinMid = stepPinY;
  stepPinMin = stepPinZ;
  //Serial.print("max - mid - min");
  //Serial.println();
}
  else if (nStepZ < nStepX && nStepX < nStepY) {
    nStepMax = nStepY;
    nStepMid = nStepX;
    nStepMin = nStepZ;
    stepPinMax = stepPinY;
    stepPinMid = stepPinX;
    stepPinMin = stepPinZ;
    //Serial.print("mid - max - min");
    //Serial.println();
    }
  else if (nStepY < nStepX && nStepX < nStepZ) {
    nStepMax = nStepZ;
    nStepMid = nStepX;
    nStepMin = nStepY;
    stepPinMax = stepPinZ;
    stepPinMid = stepPinX;
    stepPinMin = stepPinY;
    //Serial.print("mid - min - max");
    //Serial.println();
    }

////
  else if (nStepX < nStepY && nStepY < nStepZ) {
    nStepMax = nStepZ;
    nStepMid = nStepY;
    nStepMin = nStepX;
    stepPinMax = stepPinZ;
    stepPinMid = stepPinY;
    stepPinMin = stepPinX;
    //Serial.print("min - mid - max");
    //Serial.println();
    }
////

  else if (nStepX < nStepZ && nStepZ < nStepY) {
    nStepMax = nStepY;
    nStepMid = nStepZ;
    nStepMin = nStepX;
    stepPinMax = stepPinY;
    stepPinMid = stepPinZ;
    stepPinMin = stepPinX;
    //Serial.print("min - max - md");
    //Serial.println();
    }
  else if (nStepY < nStepZ && nStepZ < nStepX) {
    nStepMax = nStepX;
    nStepMid = nStepZ;
    nStepMin = nStepY;
    stepPinMax = stepPinX;
    stepPinMid = stepPinZ;
    stepPinMin = stepPinY;
    //Serial.print("max - min - mid");
    //Serial.println();
    }


//////// == max == ////////////
////
  else if (nStepX == nStepY && nStepY > nStepZ) {
    nStepMax = nStepX;
    nStepMid = nStepY;
    nStepMin = nStepZ;
    stepPinMax = stepPinX;
    stepPinMid = stepPinY;
    stepPinMin = stepPinZ;
    //Serial.print("max - max - min");
    //Serial.println();
    }
  else if (nStepX == nStepZ && nStepX > nStepY) {
    nStepMax = nStepX;
    nStepMid = nStepZ;
    nStepMin = nStepY;
    stepPinMax = stepPinX;
    stepPinMid = stepPinZ;
    stepPinMin = stepPinY;
    //Serial.print("max - min - max");
    //Serial.println();
    }
  else if (nStepY == nStepZ && nStepZ > nStepX) {
    nStepMax = nStepY;
    nStepMid = nStepZ;
    nStepMin = nStepX;
    stepPinMax = stepPinY;
    stepPinMid = stepPinZ;
    stepPinMin = stepPinX;
    //Serial.print("min - max - max");
    //Serial.println();
    }

//////// == min == ////////////
////
  else if (nStepX == nStepY && nStepY < nStepZ) {
    nStepMax = nStepZ;
    nStepMid = nStepX;
    nStepMin = nStepY;
    stepPinMax = stepPinZ;
    stepPinMid = stepPinX;
    stepPinMin = stepPinY;
    //Serial.print("min - min - max");
    //Serial.println();
    }
  else if (nStepX == nStepZ && nStepX < nStepY) {
    nStepMax = nStepY;
    nStepMid = nStepX;
    nStepMin = nStepZ;
    stepPinMax = stepPinY;
    stepPinMid = stepPinX;
    stepPinMin = stepPinZ;
    //Serial.print("min - max - min");
    //Serial.println();
    }
  else if (nStepY == nStepZ && nStepZ < nStepX) {
    nStepMax = nStepX;
    nStepMid = nStepY;
    nStepMin = nStepZ;
    stepPinMax = stepPinX;
    stepPinMid = stepPinY;
    stepPinMin = stepPinZ;
    //Serial.print("max - min - min");
    //Serial.println();
    }
//////// == == == ////////////
////
  else if (nStepX == nStepY && nStepY == nStepZ) {
    nStepMax = nStepX;
    nStepMid = nStepY;
    nStepMin = nStepZ;
    stepPinMax = stepPinX;
    stepPinMid = stepPinY;
    stepPinMin = stepPinZ;
    //Serial.print("all");
    //Serial.println();
    }

  ratio = nStepMax/900;
    //Serial.print("ratio = ");
    //Serial.print(ratio);
    //Serial.println();
  //int num1;

  float current_axis_min = 0;
  float current_axis_mid = 0;
  long steps_axis_min = 0;
  long steps_axis_mid = 0;
  float ratio_max_min = 0;
  float ratio_max_mid = 0;



  ratio_max_min = nStepMax / nStepMin;
  ratio_max_mid = nStepMax / nStepMid;
  
  if ( dirX == 1)
  {
    //digitalWrite(dirPinX, HIGH);
		GPIOA->ODR|=1<<dirPinX;
  }
  else
  {
    //digitalWrite(dirPinX, LOW);
		GPIOA->ODR&=~(1<<dirPinX);
  }

  if ( dirY == 1)
  {
    //digitalWrite(dirPinY, HIGH);
		GPIOA->ODR|=1<<dirPinY;
  }
  else
  {
    //digitalWrite(dirPinY, LOW);
		GPIOA->ODR&=~(1<<dirPinY);
  }
  if ( dirZ == 1)
  {
    //digitalWrite(dirPinZ, 1);
		GPIOA->ODR|=1<<dirPinZ;
  }
  else
  {
    //digitalWrite(dirPinZ, 0);
		GPIOA->ODR&=~(1<<dirPinZ);
  }

	if (nStepMax>=0 && nStepMax<=2400) {
		a = 1/nStepMax;
		b = -a*nStepMax;
	}
	else {
		a = 2000/pow(nStepMax,2);
	  b = -a*nStepMax;	
	}
	y=0x0000;
	
	
  for (int i = 1 ; i <= nStepMax ; i = i + 1) {
		GPIOC->ODR |= (1<<13);
    current_axis_min = i / ratio_max_min;
    current_axis_mid = i / ratio_max_mid;
		//send_string("hi\n");


    if (current_axis_min - steps_axis_min >= 1) {
      //digitalWrite(stepPinMin, HIGH);
			GPIOA->ODR|=1<<stepPinMin;
      pulseMin = 1;
      steps_axis_min++;
    }
    if (current_axis_mid - steps_axis_mid >= 1) {
      //digitalWrite(stepPinMid, HIGH);
			GPIOA->ODR|=1<<stepPinMid;
      pulseMid = 1;
      steps_axis_mid++;
    }

    //digitalWrite(stepPinMax, HIGH);
		GPIOA->ODR|=1<<stepPinMax;
    pulseMax = 1;

    // pwm/////////////////////////////////////////
    // pwm/////////////////////////////////////////
    if(ratio >= 2){
      num1++;

      if (((num1 - Update) > ratio) && (num1 <= nStepMax*(0.5))){
        Time--;
  
        Update=Update+ratio;
        }
  
      if (((num1 - Update) > ratio) && (num1 > nStepMax*(0.5)) && (num1 < (nStepMax - 100))){
        Time++;
        Update=Update+ratio;
        }
  
      if (i>=nStepMax-100 && i<=nStepMax){
        Time = 1600;
      }
    }

    else {
      if ((i>=nStepMax-150) | (i<=150)) Time = 1500;
      else Time = 250;
    }
		Time = 500;
    y=a*i*i + b*i + 600;
		delay_us(y);

		GPIOA->ODR&=~(1<<stepPinMax);
		GPIOA->ODR&=~(1<<stepPinMid);
		GPIOA->ODR&=~(1<<stepPinMin);

    pulseMax = 0;
    pulseMid = 0;
    pulseMin = 0;

    //New = micros();
    //while(micros() - New < Time);
		delay_us(y);
		//delay_ms(50);

  }
	//free(nStepMax);
	nStepMax=0;
	GPIOC->ODR &=~ (1<<13);
}

// POSITION
void MovePosition(float Px, float Py, float Pz, float *temp1_mem, float *temp2_mem, float *temp3_mem){
  double t1, t2, t3, r, s, D;
  int dirX, dirY, dirZ;

  //Px = Px;
  //Py = Py;

  t1 = atan2(Py, Px);

  //Px = Px - 3.5*cos(t1);
  //Py = Py - 3.5*sin(t1);
  //Pz = Pz + 5;

  r = sqrt(Px*Px + Py*Py);
	//r = 19.7;
  s = Pz - l1;
	//s = 25.4;
  D = (r*r + s*s - l2*l2 - l3*l3) / (2 * l2 * l3);
	//D=0;
  t3 = atan2(-sqrt(1 - D*D), D);
  t2 = atan2(s, r) - atan2(l3 * sin(t3), l2 + l3 * cos(t3));

	

  t2 = PI/2 - t2;
  t3 = PI/2 + t3;

  //t2 =t2 + (2*PI)/9;
  //t3 =t3 + PI/9;

 
  send_string("Goc lech ban dau\n");
	char msg1[200];
	sprintf(msg1, "%.2f   %.2f   %.2f\n", t1*180/3.14, t2*180/3.14, t3*180/3.14);
	send_string(msg1);
	

  dpX = ((t1 -*temp1_mem) * (180 / PI))/micro;
  dpY = ((t2 -*temp2_mem) * (180 / PI))/micro;
  dpZ = ((t3 -*temp3_mem) * (180 / PI))/micro;
	
	if (dpX<0)dpX=-dpX;
	if (dpY<0)dpY=-dpY;
	if (dpZ<0)dpZ=-dpZ;

	
  send_string("Goc thuc thi\n");
	char msg2[200];
	sprintf(msg2, "%.2f   %.2f   %.2f\n", (t1 - *temp1_mem)*180/3.14, (t2 - *temp2_mem)*180/3.14, (t3 - *temp3_mem)*180/3.14);
	send_string(msg2);

  if ((t1 - *temp1_mem) > 0)
  {
    dirX = 1;
  }
  else
  {
    dirX = 0;
  }

  if ((t2 - *temp2_mem) < 0)
  {
    dirY = 1;
  }
  else
  {
    dirY = 0;
  }

  if ((t3 - *temp3_mem) > 0)
  {
    dirZ = 1;
  }
  else
  {
    dirZ = 0;
  }

  dpX = X * dpX;
  dpY = Y * dpY;
  dpZ = Z * dpZ;

	
  send_string("So xung\n");
	char msg3[200];
	sprintf(msg3, "%.2f   %.2f   %.2f\n", dpX, dpY, dpZ);
	send_string(msg3);

  //temp1 = t1;
  //temp2 = t2;
  //temp3 = t3;
  *temp1_mem = t1;
  *temp2_mem = t2;
  *temp3_mem = t3;



  moveXYZ(dpX,stepPinX,dirX, dpY,stepPinY,dirY, dpZ,stepPinZ,dirZ);
  }

/////////////////Angle//////////////////////////////////////////////////////////////////
void Move_angle3(int Do_chess, int Dir_chess) {
  //double long Xung;
  Xung = Do_chess /micro;
  Xung = Xung * Z;
  //Serial.println("2");
  for (int i = 1; i <= Xung; i++)
  {
    //digitalWrite(dirPinZ, Dir_chess);
    //digitalWrite(stepPinZ, HIGH);
		GPIOA->ODR|=1<<stepPinZ;
		if (Dir_chess == 1) GPIOA->ODR|=1<<dirPinZ;
		else GPIOA->ODR&=~(1<<dirPinZ);
    

		delay_us(700);
		cc=i;
    
    //digitalWrite(stepPinZ, LOW);
		GPIOA->ODR&=~(1<<stepPinZ);
    
		delay_us(700);
  }
	//cc=0;
}

void Move_angle2(int Do_chess, int Dir_chess) {
  //float Xung;
  Xung = Do_chess /micro;
  Xung = Xung * Y;
  //Serial.println("2");
  for (uint32_t i=0; i<Xung; i++)
  {
		GPIOA->ODR|=1<<stepPinY;
		if (Dir_chess == 1) GPIOA->ODR|=1<<dirPinY;
		else GPIOA->ODR&=~(1<<dirPinY);
		//bb++;
		bb=i;

		GPIOC->ODR^=1<<13;
		delay_us(700);

		GPIOA->ODR&=~(1<<stepPinY);
    
		GPIOC->ODR^=1<<13;
		delay_us(700);
  }
	//cc=0;
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
