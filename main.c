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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t values[12];
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //START AND INITIALIZE TIMMER CHANNELS
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  htim15.Instance->CCR1=D1B;
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  htim15.Instance->CCR2=D1;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  htim3.Instance->CCR1=D2B;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  htim3.Instance->CCR2=D2;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  htim3.Instance->CCR3=D3B;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  htim3.Instance->CCR4=D3;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  htim1.Instance->CCR1=D4B; //CCR aqui es 100 por lo q D varia entre 0-100
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  htim1.Instance->CCR2=D4;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  htim1.Instance->CCR3=D5B;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  htim2.Instance->CCR1=D5;
  //DUTY CYCLE PARAMETERS
  float delta1=1;
  float delta2=1;
  float delta3=1;
  float delta4=1;
  float delta5=1;
  float sync=0;
  int Dmin=5;
  int Dmax=95;
  int Dini=69;
  int DBini=Dini+sync;
  int D1=Dini;
  int D1B=DBini;
  int D2=Dini;
  int D2B=DBini;
  int D3=Dini;
  int D3B=DBini;
  int D4=Dini;
  int D4B=DBini;
  int D5=Dini;
  int D5B=DBini;
  int step=1;
  //INITIALIZE VARIABLES
  float Vold_1=0;
  float Iold_1=0;
  float Pold_1=0;
  float Dold_1=D1;
  float Vnew_1=0;
  float Inew_1=0;
  float Pnew_1=0;
  float dV_1=0;
  float dI_1=0;
  float dP_1=0;

  float Vold_2=0;
  float Iold_2=0;
  float Pold_2=0;
  float Dold_2=D2;
  float Vnew_2=0;
  float Inew_2=0;
  float Pnew_2=0;
  float dV_2=0;
  float dI_2=0;
  float dP_2=0;

  float Vold_3=0;
  float Iold_3=0;
  float Pold_3=0;
  float Dold_3=D3;
  float Vnew_3=0;
  float Inew_3=0;
  float Pnew_3=0;
  float dV_3=0;
  float dI_3=0;
  float dP_3=0;

  float Vold_4=0;
  float Iold_4=0;
  float Pold_4=0;
  float Dold_4=D4;
  float Vnew_4=0;
  float Inew_4=0;
  float Pnew_4=0;
  float dV_4=0;
  float dI_4=0;
  float dP_4=0;

  float Vold_5=0;
  float Iold_5=0;
  float Pold_5=0;
  float Dold_5=D5;
  float Vnew_5=0;
  float Inew_5=0;
  float Pnew_5=0;
  float dV_5=0;
  float dI_5=0;
  float dP_5=0;

  float Vout_new=0;
  float Iout_new=0;
  float Pout_new=0;

  float Vref=3.3;
  int n1=1000;
  int n2=2500;
  int Tim1=0;
  int Tim2=0;
  int Tim3=0;
  int Tim4=0;
  int Tim_lim=60000; //time lim in mseconds
  int G=200;
  int Rs=0.2;

//GPIOs configuration
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);//INA_1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); //INA_2
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); //INA_3
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //INA_4
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); //INA_5
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); //INA_6
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1); //INA_7
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); //VOLTAGE REGULATOR
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);//SWITCH BATTERY
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);//SWITCH LOAD
  //ADC CALIBRATION
  HAL_ADC_Stop(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc1,values,12);

  //DATA TRANSMISSION
  char buffer[128];
  int reset_int=0;
  uint8_t reset[3];
  int Vnew_f_1=0;
  int Inew_f_1=0;
  int Vnew_f_2=0;
  int Inew_f_2=0;
  int Vnew_f_3=0;
  int Inew_f_3=0;
  int Vnew_f_4=0;
  int Inew_f_4=0;
  int Vnew_f_5=0;
  int Inew_f_5=0;

  int Vout_new_f=0;
  int Iout_new_f=0;
  int Vout_2=0;
  int max_delta=5;
  int Tim_diff=0;
  int iter=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */
  	  Tim1=HAL_GetTick();
  	  //ENABLE INAs and VR
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);//INA_1
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); //INA_2
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); //INA_3
  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //INA_4
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); //INA_5
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); //INA_6
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1); //INA_7
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); //VOLTAGE REGULATOR
  	  HAL_Delay(10);
  	  if (reset_int==999)
  	  {
  		  D1=Dini;
  		  D1B=DBini;
  		  D2=Dini;
  		  D2B=DBini;
  		  D3=Dini;
  		  D3B=DBini;
  		  D4=Dini;
  		  D4B=DBini;
  		  D5=Dini;
  		  D5B=DBini;

  		  Vold_1=0;
  		  Iold_1=0;
  		  Pold_1=0;
  		  Dold_1=D1;

  		  Vold_2=0;
  		  Iold_2=0;
  		  Pold_2=0;
  		  Dold_2=D2;

  		  Vold_3=0;
  		  Iold_3=0;
  		  Pold_3=0;
  		  Dold_3=D3;

  		  Vold_4=0;
  		  Iold_4=0;
  		  Pold_4=0;
  		  Dold_4=D4;

  		  Vold_5=0;
  		  Iold_5=0;
  		  Pold_5=0;
  		  Dold_5=D5;
  	  }
  	  //ADC MEASUREMENTS
  	  Vnew_1=0;
  	  Inew_1=0;
  	  Vnew_2=0;
  	  Inew_2=0;
  	  Vnew_3=0;
  	  Inew_3=0;
  	  Vnew_4=0;
  	  Inew_4=0;
  	  Vnew_5=0;
  	  Inew_5=0;
  	  Vout_new=0;
  	  Iout_new=0;
  	  for (int i=0;i<=n1-1;i++)
  	  {
  		  Vnew_1=Vnew_1+values[0];
  		  Vnew_2=Vnew_2+values[1];
  		  Vnew_3=Vnew_3+values[2];
  		  Vnew_4=Vnew_4+values[3];
  		  Vnew_5=Vnew_5+values[4];
  		  Vout_new=Vout_new+values[11];
  	  }

  	  for (int i=0;i<=n2-1;i++)
  	  {
  		  Inew_1=Inew_1+values[5];
  		  Inew_2=Inew_2+values[6];
  		  Inew_3=Inew_3+values[7];
  	      Inew_4=Inew_4+values[8];
  	      Inew_5=Inew_5+values[9];
  	      Iout_new=Iout_new+values[10];
  	  }
  	  //DISABLE INAs AND VR
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);//INA_1
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); //INA_2
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); //INA_3
  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); //INA_4
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //INA_5
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0); //INA_6
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1); //INA_7
  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); //VOLTAGE REGULATOR

  	  //POWER, dV AND dI CALCULATIONS
  	  Vnew_1=((Vnew_1/n1)-2048)*Vref/2048;
  	  Vnew_2=((Vnew_2/n1)-2048)*Vref/2048;
  	  Vnew_3=((Vnew_3/n1)-2048)*Vref/2048;
  	  Vnew_4=((Vnew_4/n1)-2048)*Vref/2048;
  	  Vnew_5=(Vnew_5*Vref)/(4096*n1);
  	  Vout_new=(Vout_new*Vref)/(4096*n1);

  	  Inew_1=(Inew_1*Vref)/(n2*4096*40);
  	  Inew_2=(Inew_2*Vref)/(n2*4096*40);
  	  Inew_3=(Inew_3*Vref)/(n2*4096*40);
  	  Inew_4=(Inew_4*Vref)/(n2*4096*40);
  	  Inew_5=(Inew_5*Vref)/(n2*4096*40);
  	  Iout_new=(Iout_new*Vref)/(n2*4096*40);

  	  Pnew_1=(Vnew_1*Inew_1);
  	  Pnew_2=(Vnew_2*Inew_2);
  	  Pnew_3=(Vnew_3*Inew_3);
  	  Pnew_4=(Vnew_4*Inew_4);
  	  Pnew_5=(Vnew_5*Inew_5);

  	  dV_1=Vnew_1-Vold_1;
  	  dV_2=Vnew_2-Vold_2;
  	  dV_3=Vnew_3-Vold_3;
  	  dV_4=Vnew_4-Vold_4;
  	  dV_5=Vnew_5-Vold_5;

  	  dI_1=Inew_1-Iold_1;
  	  dI_2=Inew_2-Iold_2;
  	  dI_3=Inew_3-Iold_3;
  	  dI_4=Inew_4-Iold_4;
  	  dI_5=Inew_5-Iold_5;

  	  dP_1=Pnew_1-Pold_1;
  	  dP_2=Pnew_2-Pold_2;
  	  dP_3=Pnew_3-Pold_3;
  	  dP_4=Pnew_4-Pold_4;
  	  dP_5=Pnew_5-Pold_5;

  	  //MPPT MFC1
  	  if (Pnew_1==0)
  	  {
  		  delta1=1;
  	  }
  	  else
  	  {
  		  delta1=dP_1*10/Pnew_1;
  	  }

  	  if (delta1<0)
  	  {
  		  delta1=-delta1;
  	  }
  	  if (delta1>5)
  	  {
  		  delta1=5;
  	  }
  	  if (delta1<1)
  	  {
  		  delta1=1;
  	  }
  	  if (dV_1==0)
  	  {
  		  if (dI_1==0)
  		  {
  			  D1=Dold_1;
  		  }
  		  else
  		  {
  			  if (dI_1>0)        //DECREASE D
  			  {
  				  D1=Dold_1-delta1;
  				  D1B=D1+sync;
  			  }
  			  else            //INCREASE D
  			  {
  				  D1=Dold_1+delta1;
  				  D1B=D1+sync;
  			  }
  		  }
  	  }
  	  else
  	  {
  		   if (dI_1/dV_1== -Inew_1/Vnew_1) //do nothing
  		   {
  			   D1=Dold_1;
  		   }
  		   else if (dI_1/dV_1 > -Inew_1/Vnew_1)  //DECREASE D
  		   {
  			   D1=Dold_1-delta1;
  			   D1B=D1+sync;
  		   }
  		   else    //INCREASE D
  		   {
  			   D1=Dold_1+delta1;
  			   D1B=D1+sync;
  		   }
  	  }
  	  //MPPT MFC2
  	  if (Pnew_2==0)
  	  {
  		  delta2=1;
  	  }
  	  else
  	  {
  		  delta2=dP_2*10/Pnew_2;
  	  }
  	  if (delta2<0)
  	  {
  		  delta2=-delta2;
  	  }
  	  if (delta2>5)
  	  {
  		  delta2=5;
  	  }
  	  if (delta2<1)
  	  {
  		  delta2=1;
  	  }
  	  if (dV_2==0)
  	  {
  		  if (dI_2==0)
  		  {
  			  D2=Dold_2;
  		  }
  		  else
  		  {
  			  if (dI_2>0)        //DECREASE D
  			  {
  				  D2=Dold_2-delta2;
  				  D2B=D2+sync;
  			  }
  			  else            //INCREASE D
  			  {
  				  D2=Dold_2+delta2;
  				  D2B=D2+sync;
  			  }
  		  }
  	  }
  	  else
  	  {
  		   if (dI_2/dV_2== -Inew_2/Vnew_2) //do nothing
  		   {
  			   D2=Dold_2;
  		   }
  		   else if (dI_2/dV_2 > -Inew_2/Vnew_2)  //DECREASE D
  		   {
  			   D2=Dold_2-delta2;
  			   D2B=D2+sync;
  		   }
  		   else    //INCREASE D
  		   {
  			   D2=Dold_2+delta2;
  			   D2B=D2+sync;
  		   }
  	  }
  	  //MPPT MFC3
  	  if (Pnew_3==0)
  	  {
  		  delta3=1;
  	  }
  	  else
  	  {
  		  delta3=dP_3*10/Pnew_3;
  	  }
  	  if (delta3<0)
  	  {
  		  delta3=-delta3;
  	  }
  	  if (delta3>5)
  	  {
  		  delta3=5;
  	  }
  	  if (delta3<1)
  	  {
  		  delta3=1;
  	  }
  	  if (dV_3==0)
  	  {
  		  if (dI_3==0)
  		  {
  			  D3=Dold_3;
  		  }
  		  else
  		  {
  			  if (dI_3>0)        //DECREASE D
  			  {
  				  D3=Dold_3-delta3;
  				  D3B=D3+sync;
  			  }
  			  else            //INCREASE D
  			  {
  				  D3=Dold_3+delta3;
  				  D3B=D3+sync;
  			  }
  		  }
  	  }
  	  else
  	  {
  		   if (dI_3/dV_3== -Inew_3/Vnew_3) //do nothing
  		   {
  			   D3=Dold_3;
  		   }
  		   else if (dI_3/dV_3 > -Inew_3/Vnew_3)  //DECREASE D
  		   {
  			   D3=Dold_3-delta3;
  			   D3B=D3+sync;
  		   }
  		   else    //INCREASE D
  		   {
  			   D3=Dold_3+delta3;
  			   D3B=D3+sync;
  		   }
  	  }
  	  //MPPT MFC4
  	  if (Pnew_4==0)
  	  {
  		  delta4=1;
  	  }
  	  else
  	  {
  		  delta4=dP_4*10/Pnew_4;
  	  }
  	  if (delta4<0)
  	  {
  		  delta4=-delta4;
  	  }
  	  if (delta4>5)
  	  {
  		  delta4=5;
  	  }
  	  if (delta4<1)
  	  {
  		  delta4=1;
  	  }
  	  if (dV_4==0)
  	  {
  		  if (dI_4==0)
  		  {
  			  D4=Dold_4;
  		  }
  		  else
  		  {
  			  if (dI_4>0)        //DECREASE D
  			  {
  				  D4=Dold_4-delta4;
  				  D4B=D4+sync;
  			  }
  			  else            //INCREASE D
  			  {
  				  D4=Dold_4+delta4;
  				  D4B=D4+sync;
  			  }
  		  }
  	  }
  	  else
  	  {
  		   if (dI_4/dV_4== -Inew_4/Vnew_4) //do nothing
  		   {
  			   D4=Dold_4;
  		   }
  		   else if (dI_4/dV_4 > -Inew_4/Vnew_4)  //DECREASE D
  		   {
  			   D4=Dold_4-delta4;
  			   D4B=D4+sync;
  		   }
  		   else    //INCREASE D
  		   {
  			   D4=Dold_4+delta4;
  			   D4B=D4+sync;
  		   }
  	  }
  	  //MPPT MFC5
  	  if (Pnew_5==0)
  	  {
  		  delta5=1;
  	  }
  	  else
  	  {
  		  delta5=dP_5*10/Pnew_5;
  	  }
  	  if (delta5<0)
  	  {
  		  delta5=-delta5;
  	  }
  	  if (delta5>5)
  	  {
  		  delta5=5;
  	  }
  	  if (delta5<1)
  	  {
  		  delta5=1;
  	  }
  	  if (dV_5==0)
  	  {
  		  if (dI_5==0)
  		  {
  			  D5=Dold_5;
  		  }
  		  else
  		  {
  			  if (dI_5>0)        //DECREASE D
  			  {
  				  D5=Dold_5-delta5;
  				  D5B=D5+sync;
  			  }
  			  else            //INCREASE D
  			  {
  				  D5=Dold_5+delta5;
  				  D5B=D5+sync;
  			  }
  		  }
  	  }
  	  else
  	  {
  		   if (dI_5/dV_5== -Inew_5/Vnew_5) //do nothing
  		   {
  			   D5=Dold_5;
  		   }
  		   else if (dI_5/dV_5 > -Inew_5/Vnew_5)  //DECREASE D
  		   {
  			   D5=Dold_5-delta5;
  			   D5B=D5+sync;
  		   }
  		   else    //INCREASE D
  		   {
  			   D5=Dold_5+delta5;
  			   D5B=D5+sync;
  		   }
  	  }

  	  //DUTY CYCLES LIMITS
  	  if (D1<Dmin)
  	  {
  		  D1=Dmin;
  		  D1B=D1+sync;
  	  }
  	  if (D1>Dmax)
  	  {
  		  D1=Dmax;
  		  D1B=D1+sync;
  	  }

  	  if (D2<Dmin)
  	  {
  		  D2=Dmin;
  		  D2B=D2+sync;
  	  }
  	  if (D2>Dmax)
  	  {
  		  D2=Dmax;
  		  D2B=D2+sync;
  	  }

  	  if (D3<Dmin)
  	  {
  		  D3=Dmin;
  		  D3B=D3+sync;
  	  }
  	  if (D3>Dmax)
  	  {
  		  D3=Dmax;
  		  D3B=D3+sync;
  	  }

  	  if (D4<Dmin)
  	  {
  		  D4=Dmin;
  		  D4B=D4+sync;
  	  }
  	  if (D4>Dmax)
  	  {
  		  D4=Dmax;
  		  D4B=D4+sync;
  	  }

  	  if (D5<Dmin)
  	  {
  		  D5=Dmin;
  		  D5B=D5+sync;
  	  }
  	  if (D5>Dmax)
  	  {
  		  D5=Dmax;
  		  D5B=D5+sync;
  	  }

  	  //UPDATE OLD VARIABLES
  	  Dold_1=D1;
  	  Dold_2=D2;
  	  Dold_3=D3;
  	  Dold_4=D4;
  	  Dold_5=D5;

  	  Vold_1=Vnew_1;
  	  Vold_2=Vnew_2;
  	  Vold_3=Vnew_3;
  	  Vold_4=Vnew_4;
  	  Vold_5=Vnew_5;

  	  Iold_1=Inew_1;
  	  Iold_2=Inew_2;
  	  Iold_3=Inew_3;
  	  Iold_4=Inew_4;
  	  Iold_5=Inew_5;

  	  Pold_1=Pnew_1;
  	  Pold_2=Pnew_2;
  	  Pold_3=Pnew_3;
  	  Pold_4=Pnew_4;
  	  Pold_5=Pnew_5;

  	  //DATA TRANSMISSION: Current in uA (x1000000), Voltage in dec mV (x10000)
  	  Vnew_f_1=Vnew_1*10000;
  	  Inew_f_1=Inew_1*1000000;
  	  Vnew_f_2=Vnew_2*10000;
  	  Inew_f_2=Inew_2*1000000;
  	  Vnew_f_3=Vnew_3*10000;
  	  Inew_f_3=Inew_3*1000000;
  	  Vnew_f_4=Vnew_4*10000;
  	  Inew_f_4=Inew_4*1000000;
  	  Vnew_f_5=Vnew_5*10000;
  	  Inew_f_5=Inew_5*1000000;
  	  Vout_new_f=Vout_new*10000;
  	  Iout_new_f=Iout_new*1000000;
  	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer, sprintf(buffer,"D1=%i D2=%i D3=%i D4=%i D5=%i V1=%i V2=%i V3=%i V4=%i V5=%i I1=%i I2=%i I3=%i I4=%i I5=%i Vout=%i Iout=%i",D1,D2,D3,D4,D5,Vnew_f_1,Vnew_f_2,Vnew_f_3,Vnew_f_4,Vnew_f_5,Inew_f_1,Inew_f_2,Inew_f_3,Inew_f_4,Inew_f_5,Vout_new_f,Iout_new_f));
  	  HAL_UART_Receive_IT(&huart1, reset, 3);
  	  reset_int=atoi(reset);

  	  //SET DUTY CYCLES
  	  htim15.Instance->CCR2=D1;
  	  htim15.Instance->CCR1=D1B;

  	  htim3.Instance->CCR2=D2;
  	  htim3.Instance->CCR1=D2B;

  	  htim3.Instance->CCR4=D3;
        htim3.Instance->CCR3=D3B;

  	  htim1.Instance->CCR2=D4;
  	  htim1.Instance->CCR1=D4B;

  	  htim2.Instance->CCR1=D5;
  	  htim1.Instance->CCR3=D5B;

//Vout Control
  	  if (Vout_new<3.2)  // Connect external power source
  	  {
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
  		  HAL_Delay(10);
  		  while (Vout_new<3.25)
  		  {
  			  for (int i=0;i<=10-1;i++)
  			  {
  				  Vout_new=Vout_new+values[11];
  			  }
  			  Vout_new=(Vout_new*Vref)/(4096*10);
  			  Vout_2=Vout_new*100;
  			  HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer, sprintf(buffer,"Vout_2=%i\r\n",Vout_2));
  			  HAL_Delay(500);
  		  }
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
  	  }

  	  if (Vout_new>=3.3) //Connect Load
  	  {
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
  		  HAL_Delay(10);
  		  while (Vout_new>3.25)
  		  {
  			  for (int i=0;i<=10-1;i++)
  			  {
  				  Vout_new=Vout_new+values[11];
  			  }
  			  Vout_new=(Vout_new*Vref)/(4096*10);
  			  Vout_2=Vout_new*100;
  			  HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer, sprintf(buffer,"Vout_2=%i\r\n",Vout_2));
  			  HAL_Delay(500);
  		  }

  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
  	  }
  	// CHECK TIME DIFFERENCE AND CONTINUE IF IT HAS EXCEEDED THE LIMIT
  	  Tim2=HAL_GetTick();
        Tim_diff=(Tim2-Tim1);
        while (Tim_diff<Tim_lim)
        {
      	  Tim2=HAL_GetTick();
      	  Tim_diff=(Tim2-Tim1);
        }
        iter=iter+1;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 12;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB5 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
