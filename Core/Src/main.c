/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PID_MAX 1000//改成自己设定的PWM波的单波�????长输出时�????
#define PID_MIN -1000//改成自己设定的PWM波的单波�????长输出时间的相反�????
typedef struct{
  float sum;//时间的积�????
  float lr;//上一次的速度
  float Kp;
  float Ki;
  float Kd;
  float goal;
}pidstr;
pidstr p1,p2,p3,p4;
float PID(pidstr *a,float ver)
{
  float dr = a->goal - ver;
  a->sum = a->sum + dr;
  float pwm = a->Kp * dr + a->Ki * a->sum + a->Kd * (dr - a->lr);
  a->lr = dr;
  if(pwm >= PID_MAX) return PID_MAX;
  if(pwm <= PID_MIN) return PID_MIN;
  return pwm;
}
const float pi = 3.1415926;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == htim1.Instance){ //TIM1: �??1ms调整�??次输�??
    //我定义的编码器：TIM2,3,4,5，对应pwm chanel 1,2,3,4
    int cnt;
    float ver, pwm;
    static int cc = 0; cc++;
    
    cnt = __HAL_TIM_GetCounter(&htim2);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim2, 0);
    ver = cnt*6.5*pi; // ver cm/s
    pwm = PID(&p1, ver);
    if(cc%100==0) u3_printf("%.2f\n",ver);
    if(pwm<0){
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
      pwm=-pwm;
    }
    else{
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, pwm);

    cnt = __HAL_TIM_GetCounter(&htim3);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim3, 0);
    ver = cnt*6.5*pi; // ver cm/s
    pwm = PID(&p2, ver);
    
    if(pwm<0){
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
      pwm=-pwm;
    }
    else{
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, pwm);

    cnt = __HAL_TIM_GetCounter(&htim4);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim4, 0);
    ver = cnt*6.5*pi; // ver cm/s
    //if(cc%100==0) u3_printf("%d -- %.3f\n", cnt, ver);
    pwm = PID(&p3, ver);
    if(pwm<0){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      pwm=-pwm;
    }
    else{
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, pwm);

    cnt = __HAL_TIM_GetCounter(&htim5);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim5, 0);
    ver = cnt*6.5*pi; // ver cm/s
    //if(cc%100==0) u3_printf("%d -- %.3f\n", cnt, ver);
    pwm = PID(&p4, ver);
    if(pwm<0){
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      pwm=-pwm;
    }
    else{
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4, pwm);  
  }
}
void SetGoal(float vx, float vy){
  p1.goal = vy - vx;
  p2.goal = vy + vx;
  p3.goal = vy - vx;
  p4.goal = vy + vx;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1); //使能定时器TIM1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //使能定时器TIM8的四个�?�道为PWM输出
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //使能四个编码�??
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  const float KP = 0.7, KI = 0.4, KD = 0.4;
  p1.Kp=KP, p1.Ki=KI, p1.Kd=KD;
  p2.Kp=KP, p2.Ki=KI, p2.Kd=KD;
  p3.Kp=KP, p3.Ki=KI, p3.Kd=KD;
  p4.Kp=KP, p4.Ki=KI, p4.Kd=KD;
  HAL_Delay(1000);
  SetGoal(20*1.414, 20*1.414);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(5000);
  p1.goal=p2.goal=p3.goal=p4.goal=0;
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
