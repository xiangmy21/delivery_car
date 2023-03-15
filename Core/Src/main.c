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
#include "jy62.h"
#include "zigbee_edc24.h"
#include <math.h>
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
#define PID_MAX 1000//改成自己设定的PWM波的单波�?????长输出时�?????
#define PID_MIN -1000//改成自己设定的PWM波的单波�?????长输出时间的相反�?????
typedef struct{
  float sum;//时间的积�?????
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
float Theta, V;
void SetGoal(float theta, float v){
  Theta = theta, V = v;
}
void ReSetGoal(float theta, float v){//theta is rad.
  float yaw = GetYaw()/180*pi;
  float vy = v*cos(yaw - theta), vx = v*sin(yaw - theta);
  p1.goal = vy + vx;
  p2.goal = vy - vx;
  p3.goal = vy - vx;
  p4.goal = vy + vx;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == htim1.Instance){ //TIM1: �???1ms调整�???次输�???
    //我定义的编码器：TIM2,3,4,5，对应pwm chanel 1,2,3,4
    ReSetGoal(Theta, V);
    int cnt;
    float ver, pwm;
    static int cc = 0; cc++;
    static float v[5] = {0};
    if(cc%15==0) u3_printf("%.3f ", p1.goal);
    
    cnt = __HAL_TIM_GetCounter(&htim2);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim2, 0);
    ver = cnt*6.5*pi; // ver cm/s
    v[1] += ver;
    if(cc%15==0) u3_printf("%.3f ", v[1]/15), v[1]=0;
    pwm = PID(&p1, ver);
    //if(cc%100==0) u3_printf("%.2f\n",ver);
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
    v[2] += ver;
    if(cc%15==0) u3_printf("%.3f ", v[2]/15), v[2]=0;
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
    v[3] += ver;
    if(cc%15==0) u3_printf("%.3f ", v[3]/15), v[3]=0;
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
    v[4] += ver;
    if(cc%15==0) u3_printf("%.3f\n", v[4]/15), v[4]=0;
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

void jy62_Init(UART_HandleTypeDef* huart); 

_Bool block[300][300];
int cnt_order = 0;
Order_edc24 orders[100];

Position_edc24 P[12] = {{20,20} , {126 , 20} , {236 , 20} 
, {20 , 126} , {236 , 126} 
, {20 , 236} , {126 , 236} , {236 , 236}
, {126 , 60} , {196 , 126} , {126 , 196} , {60 , 126}}
int16_t dis[12][12] , nxt[12][12];


void Pre_dis(){

}

void BFS(Position_edc24 now){
   
}

void enblock(int x,int y,int u ,int v){
  if(x > u) swap(u , x);
  if(y > v) swap(y , v);
  int fur = 3;
  for(int i=max(x-fur , 0);i<=u+fur;i++)
    for(int j=max(y-fur , 0);j<=v+fur;j++)
      block[i][j] = 1;
}
Position_edc24 GetPosition(int16_t x, int16_t y){
  Position_edc24 pos = {x,y};
  return pos;
}
void Solve1(){

  static Position_edc24 temp_goal = {126 , 126} , ulti_goal = {126 , 126} ;
  static int tarid = -1;
  static int is_mapped = 0;
  if(!is_mapped){
    is_mapped = 1;
    memset(block , 0 , sizeof block);
    // for(int i=0;i<5;i++){
    //   Barrier_edc24 B = getOneBarrier(i);
    //   enblock(B.pos_1.x , B.pos_1.y , B.pos_2.x , B.pos_2.y);
    // } if I am fast enough,I can cross them
    enblock(38 , 38 , 107 , 40);
    enblock(147 , 38 , 216 , 40);    
    enblock(38 , 38 , 40 , 107);
    enblock(38 ,147, 40 ,  216 );    
    enblock(38 , 214 , 107 , 216);    
    enblock(147 , 214 , 216 , 216);
    enblock(214 , 38 , 216 , 107);
    enblock(214 , 147 , 216 , 216);

    //如何写安装充电柱？
    orders[cnt_order].depPos = orders[cnt_order].desPos = GetPosition(39,126);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    cnt_order ++;
    orders[cnt_order].depPos = orders[cnt_order].desPos = GetPosition(215,126);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    cnt_order ++;
    orders[cnt_order].depPos = orders[cnt_order].desPos = GetPosition(126,39);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    cnt_order ++;
  }

  Position_edc24 now = getVehiclePos();
  if(now.x == ulti_goal.x && now.y == ulti_goal.y && tarid >= 0){
    orders[tarid].commission = 0;
    if(tarid <= 2){
      setChargingPile();
    }
    tarid = -1;
  }

  // restore new order
  Order_edc24 new_order = getLatestPendingOrder();
  if(cnt_order == 0 || new_order.orderId != orders[cnt_order - 1].orderId)
    orders[cnt_order ++] = new_order;
    
  //get a new ulti_goal
  int ordernum = 0;
  if(ordernum = getOrderNum()){
    //顺便把已经送了的标记一下
    for(int i=0;i<ordernum;i++){
      int id = getOneOrder(i).orderId;
      for(int j=0;j<cnt_order;j++)
        if(orders[j].orderId == id)
          orders[j].commission = 0;
    }
    ulti_goal = getOneOrder(0).desPos; // 手上有快递就先送快递
  }
  else{
    
    BFS(now);

    int mn = 0x3f3f3f3f;
    Position_edc24 minLoc;
    for(int i=0;i<cnt_order;i++)
      if(orders[i].commission > 0){ // commission of orders delivered  will be set to -1
        if(dis[orders[i].depPos.x][orders[i].depPos.y] < mn)
          mn = dis[orders[i].depPos.x][orders[i].depPos.y] ,
          minLoc = orders[i].depPos , 
          tarid = i;
      }
    ulti_goal = minLoc;
    
  }
  
  // get a new temp_goal
  

  // and go 
  Position_edc24 now = getVehiclePos();
  if(now.x == temp_goal.x  && now.y == temp_goal.y)
    SetGoal(0,0);
  else 
    SetGoal(-atan2(temp_goal.y - now.y, temp_goal.x - now.x) , 30);
    
}
void Solve2(){

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
  
  jy62_Init(&huart2);
  HAL_TIM_Base_Start_IT(&htim1); //使能定时器TIM1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //使能定时器TIM8的四个�?�道为PWM输出
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //使能四个编码�???
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  const float ratio = 1;
  const float KP1 = 0.7, KI1 = 1, KD1 = 10;
  const float KP2 = KP1/ratio, KI2 = KI1/ratio, KD2 = KD1/ratio;
  p1.Kp=KP1, p1.Ki=KI1, p1.Kd=KD1;
  p2.Kp=KP1, p2.Ki=KI2, p2.Kd=KD2;
  p3.Kp=KP2, p3.Ki=KI2, p3.Kd=KD2;
  p4.Kp=KP1, p4.Ki=KI1, p4.Kd=KD1;
  
  zigbee_Init(&huart3);
  SetBaud(115200);
  SetHorizontal();
  InitAngle();
  Calibrate();
  SleepOrAwake();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(receive_flag)
    {
      reqGameInfo();
      zigbeeMessageRecord();
    //    u3_printf("time:%d,gameStage:%d,gameStatus:%d,score:%f,posx:%d,posy:%d,remainDist:%d,halfTime:%d\n",
    //     getGameTime(),(int32_t)getGameStage(),(int32_t)getGameStatus(),getScore(),
    //     getVehiclePos().x,getVehiclePos().y,getRemainDist(),getHalfGameDuration());
    //   u3_printf("ownPileNum:%d,oppPileNum:%d,orderNum:%d,latestOrder:(%d %d) (%d %d) %d %d %f,barrier1:(%d %d) (%d %d)\n",
    //     getOwnChargingPileNum(),getOppChargingPileNum(),getOrderNum(),
    //     getLatestPendingOrder().depPos.x,getLatestPendingOrder().depPos.y,
    //     getLatestPendingOrder().desPos.x,getLatestPendingOrder().desPos.y,
    //     getLatestPendingOrder().timeLimit,getLatestPendingOrder().orderId,getLatestPendingOrder().commission,
    //     getOneBarrier(0).pos_1.x,getOneBarrier(0).pos_1.y,getOneBarrier(0).pos_2.x,getOneBarrier(0).pos_2.y);
    //
      if(getGameStatus() == GameStandby) continue;
      int stage = getGameStage();
      if(stage == Prematch) continue;
      if(stage == FirstHalf) Solve1();
      else Solve2();
   }
    /* USER CODE END WHILE */

  }
  /* USER CODE BEGIN 3 */
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
