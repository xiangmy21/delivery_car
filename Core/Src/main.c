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
#define PID_MAX 1000//改成自己设定的PWM波的单波�??????长输出时�??????
#define PID_MIN -1000//改成自己设定的PWM波的单波�??????长输出时间的相反�??????
typedef struct{
  float sum;//时间的积�??????
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
float Theta, V , delta = 0;
void SetGoal(float theta, float v){
  Theta = theta, V = v;
}
void ReSetGoal(float theta, float v){//theta is rad.
  float yaw = (GetYaw() + delta)/180*pi;
  float vy = v*cos(yaw - theta), vx = v*sin(yaw - theta);
  p1.goal = vy + vx;
  p2.goal = vy - vx;
  p3.goal = vy - vx;
  p4.goal = vy + vx;

  v *= 0.01;
  float d = yaw - theta;
  while(d < 0) d += 2 * pi;
  while(d >=  pi) d -=  pi;
  if(d < pi / 2) v = v;
  else v = -v;
  p1.goal += v , p2.goal -= v , p3.goal += v , p4.goal -= v;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == htim1.Instance){ //TIM1: �????1ms调整�????次输�????
    //我定义的编码器：TIM2,3,4,5，对应pwm chanel 1,2,3,4
    ReSetGoal(Theta, V);
    int cnt;
    float ver, pwm;
    static int cc = 0; cc++;
    static float v[5] = {0};
    //if(cc%15==0) u3_printf("%.3f ", p1.goal);
    
    cnt = __HAL_TIM_GetCounter(&htim2);
    if(cnt>1<<15) cnt-=(1<<16);
    __HAL_TIM_SetCounter(&htim2, 0);
    ver = cnt*6.5*pi; // ver cm/s
    v[1] += ver;
    //if(cc%15==0) u3_printf("%.3f ", v[1]/15), v[1]=0;
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
    //if(cc%15==0) u3_printf("%.3f ", v[2]/15), v[2]=0;
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
    //if(cc%15==0) u3_printf("%.3f ", v[3]/15), v[3]=0;
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
    //if(cc%15==0) u3_printf("%.3f\n", v[4]/15), v[4]=0;
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

#define N 12
unsigned int block[2][256][8];


Position_edc24 GetPosition(int16_t x, int16_t y){
  Position_edc24 pos = {x,y};
  return pos;
}
int getblock(unsigned int block[256][8] , int x,int y){
  int t = y >> 5;
  return block[x][t] >> (y & 31) & 1;
}
void updblock(unsigned int block[256][8] , int x,int y){
  int t = y >> 5;
  block[x][t] |= 1 << (y & 31);
}

int cnt_order = 0;
Order_edc24 orders[100];
float Get_time[100];

int CD[100][N][2];
Position_edc24 temp_goal = {126 , 126} , ulti_goal = {126 , 126} ;
Position_edc24 St = {-1 , -1};

Position_edc24 P[N] = {{20,20} , {126 , 20} , {236 , 20} 
, {20 , 126} , {236 , 126} 
, {20 , 236} , {126 , 236} , {236 , 236}
, {126 , 60} , {196 , 126} , {126 , 196} , {60 , 126}};
int dis[N][N] , nxt[N][N] , gnxt[N];

#define INF 0x3f3f3f3f
#define Velocity 30
double param = Velocity/1000.0;
int last_fix , start_fix;
int abs(int x){return x>=0?x:-x;}
int max(int a,int b){return a>b?a:b;}
int min(int a,int b){return a<b?a:b;}
float max_float(float a , float b){ return a > b ? a : b; }
int calc_direct(Position_edc24 s, Position_edc24 t){
  int dx = t.x-s.x, dy = t.y-s.y, blocknum = 0, k = min(abs(s.x-t.x)+abs(s.y-t.y),50);
  // for(int i = 0; i < k; i++){
  //   int x=s.x+dx*i*1.0/k, y=s.y+dy*i*1.0/k;
  //   if(getblock(block[0] , x , y)) return INF;
  //   if(getblock(block[1] , x , y)) blocknum++;
  // }
  double d = sqrt(dx*dx+dy*dy);
  return k ? d + d * blocknum / k / Velocity*100*param : 0;
}

void Pre_dis(){
  // calc dis of P. floyd.
  for(int i=0;i<N;i++)
    for(int j=0;j<N;j++)
      dis[i][j] = calc_direct(P[i],P[j])
      , nxt[i][j] = j;
  for(int k=0;k<N;k++)
    for(int i=0;i<N;i++)
      for(int j=0;j<N;j++)
        if(i!=k && dis[i][j] > dis[i][k] + dis[k][j])
          dis[i][j] = dis[i][k] + dis[k][j] , 
          nxt[i][j] = nxt[i][k];

}

void enblock(int x,int y,int u ,int v, int flg){
  // if(x > u) swap(u , x);
  // if(y > v) swap(y , v);
  int fur = flg==1?5:2;
  for(int i=max(x-fur , 0);i<=u+fur;i++)
    for(int j=max(y-fur , 0);j<=v+fur;j++)
      updblock(block[flg-1] , i , j);
}
int Touch(Position_edc24 a, Position_edc24 b){
  return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) <= 64;
}

int dep , bstorder , have_order_cnt , get_time[5] ;
float bstsc, st_time;
Position_edc24 arr[11] , bst;
Order_edc24 have_order[5];
void swap(void *a, void *b, size_t size) {
    char *p = a, *q = b, tmp;
    for (size_t i = 0; i < size; i++) {
        tmp = p[i];
        p[i] = q[i];
        q[i] = tmp;
    }
}
float H(float sc , float ntime , int d){
  float ret = 0;
  if(ntime > st_time + 1e-7) ret = max_float(ret , sc / (ntime - st_time));
  Order_edc24 have_order_buf[5];
  int get_time_buf[5] , have_order_cnt_buf = have_order_cnt;

  memcpy(have_order_buf , have_order , sizeof have_order);
  memcpy(get_time_buf , get_time , sizeof get_time);
  while(d > 0 && have_order_cnt > 0){
    int mxloc = 0;
    float mx = 0;
    for(int i=0;i<have_order_cnt;i++)
      if(getCommission(have_order[i]  , ntime - get_time[i]) > mx){
        mxloc = i;
        mx = getCommission(have_order[i] , ntime - get_time[i]);
      }
    ntime += 2 , sc += mx;
    ret = max_float(ret , sc / (ntime - st_time));
    swap(&have_order[mxloc] , &have_order[have_order_cnt - 1] , sizeof(Position_edc24));
    swap(&get_time[have_order_cnt - 1] , &get_time[mxloc] , sizeof(int));
    have_order_cnt -- , d--;
  }
  int usd[6] = {}; usd[0] = 0;
  while(d >= 2){
    int mxloc = 0;
    float mx = 0;
    for(int i=0;i<cnt_order;i++) // to be improved
      if(orders[i].isgot == 0)
        if(orders[i].commission > mx)
          mx = orders[i].commission , mxloc = i;
    sc += mx , ntime += 4;
    ret = max_float(ret , sc / (ntime - st_time));
    usd[++usd[0]] = mxloc;
    orders[mxloc].isgot = 1;
    d -= 2;
  }
  for(;usd[0];)
    orders[usd[usd[0]] --].isgot = 0;
  
  memcpy(have_order ,have_order_buf ,  sizeof have_order);
  memcpy(get_time , get_time_buf , sizeof get_time);
  have_order_cnt = have_order_cnt_buf;
  return ret;
}
void dfs(int nd , float sc , float ntime,Position_edc24 now){
  float hsc = H(sc , ntime , dep - nd);
  if(hsc < bstsc) return;
  if(nd == dep){
    bst = arr[0];
    bstsc = hsc;
    return ;
  }

  if(have_order_cnt < 5)
  for(int i=0;i<cnt_order;i++)
    if(!orders[i].isgot){
      float nt = calc_direct(now , orders[i] . desPos) / 45;
      orders[i].isgot = 1;
      arr[nd] = orders[i].depPos;
      get_time[have_order_cnt] = ntime + nt;
      have_order[have_order_cnt ++] = orders[i];
      dfs(nd + 1,sc , ntime + nt , orders[i].depPos);
      have_order_cnt --;
      orders[i].isgot = 0;
    }
  for(int i=0;i<have_order_cnt;i++){
      float nt = calc_direct(now , have_order[i] . desPos) / 45;
      float nsc = getCommission(have_order[i] , ntime + nt - get_time[i]);
      arr[nd] = have_order[i] . desPos;
      swap(&have_order[i] , &have_order[have_order_cnt - 1] , sizeof (Position_edc24));
      swap(&get_time[have_order_cnt - 1] , &get_time[i] , sizeof (int));
      have_order_cnt --;
      dfs(nd + 1 , sc + nsc , ntime + nt , arr[nd]);
      have_order_cnt ++;
      swap(&have_order[i] , &have_order[have_order_cnt - 1] , sizeof (Position_edc24));
      swap(&get_time[have_order_cnt - 1] , &get_time[i] , sizeof (int));
    }
}

void Solve1(){
  static int tmpid = -1;
  static int is_mapped = 0;
  if(!is_mapped){
    is_mapped = 1;
    memset(block , 0 , sizeof block);
    for(int i=0;i<5;i++){
      Barrier_edc24 B = getOneBarrier(i);
      enblock(B.pos_1.x , B.pos_1.y , B.pos_2.x , B.pos_2.y, 2);
    } //if I am fast enough,I can cross them
    enblock(38 , 38 , 107 , 40 , 1);
    enblock(147 , 38 , 216 , 40 , 1);    
    enblock(38 , 38 , 40 , 107 , 1);
    enblock(38 ,147, 40 ,  216 , 1);    
    enblock(38 , 214 , 107 , 216, 1);    
    enblock(147 , 214 , 216 , 216 , 1);
    enblock(214 , 38 , 216 , 107 , 1);
    enblock(214 , 147 , 216 , 216 , 1);

    //如何写安装充电柱�?
    orders[cnt_order].depPos = GetPosition(39,126) ,  orders[cnt_order].desPos = GetPosition(40,126);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    orders[cnt_order].isgot = 0;
    for(int i=0;i<N;i++)
      CD[cnt_order][i][0] = calc_direct(orders[cnt_order].depPos , P[i]),
      CD[cnt_order][i][1] = calc_direct(orders[cnt_order].desPos , P[i]);
   
    cnt_order ++;
    orders[cnt_order].depPos = GetPosition(215,126);orders[cnt_order].desPos = GetPosition(214,126);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    orders[cnt_order].isgot = 0;
    for(int i=0;i<N;i++)
      CD[cnt_order][i][0] = calc_direct(orders[cnt_order].depPos , P[i]),
      CD[cnt_order][i][1] = calc_direct(orders[cnt_order].desPos , P[i]);
   
    cnt_order ++;
    orders[cnt_order].depPos  = GetPosition(126,39); orders[cnt_order].desPos = GetPosition(126,40);
    orders[cnt_order].commission = 100;orders[cnt_order].timeLimit = 100;
    orders[cnt_order].isgot = 0;
    for(int i=0;i<N;i++)
      CD[cnt_order][i][0] = calc_direct(orders[cnt_order].depPos , P[i]),
      CD[cnt_order][i][1] = calc_direct(orders[cnt_order].desPos , P[i]);
   
    cnt_order ++;

    Pre_dis();
  }

  // restore new order
  Order_edc24 new_order = getLatestPendingOrder(); // 还没领取的第�?个订�?
  if(new_order.orderId!=-1 && (cnt_order == 0 || new_order.orderId != orders[cnt_order - 1].orderId)){
    for(int i=0;i<N;i++)
      CD[cnt_order][i][0] = calc_direct(new_order.depPos , P[i]),
      CD[cnt_order][i][1] = calc_direct(new_order.desPos , P[i]);
    orders[cnt_order ++] = new_order;
  }
  for(int i=0;i<getOrderNum();i++){
    int id = getOneOrder(i).orderId;
    for(int j=0;j<cnt_order;j++)
      if(orders[j].orderId == id && orders[j].isgot == 0){
        orders[j].isgot = 1;
        Get_time[j] = getGameTime() / 1000;
      }
  }
    
  Position_edc24 now = getVehiclePos();

  if(now.x >= 20 && now.x <= 240 && now.y >= 20 && now.y <= 240 
    && (getGameTime() - last_fix) >=600000 && (St.x == -1 || getGameTime() - start_fix <= 2000)){
      if(St.x == -1) St = now , start_fix = getGameTime();
      if((getGameTime() - start_fix) >= 1000)
        SetGoal(GetYaw() , 30);  
      else
        SetGoal(0,0) , St = now;
  }
  else{
    if(St.x != -1){
      delta = -atan2(-St.y + now.y, -St.x + now.x) *180/pi - GetYaw();
      St = GetPosition(-1, -1);
      last_fix = getGameTime();
    }

    //tarid : destination -2, none -1, charge 0~2, order >2
    if( Touch(now, ulti_goal)){
      int tarid = -1;
      for(int i=0;i<cnt_order;i++){
        if(orders[i].depPos.x == ulti_goal.x && orders[i].depPos.y == ulti_goal.y){
          tarid = i;
          break;
        }
      }
      if(tarid <= 2 && tarid >= 0){
        setChargingPile();
      }
    }
      
    //get a new ulti_goal
    int ordernum = getOrderNum();
    Position_edc24 tmp = ulti_goal;
    //u1_printf("tmp initial: (%d,%d)\n",tmp.x,tmp.y);

    // 在tmp中填入需要更新ulti_goal的值
    /*
     第一稿：有快递就送

    if(ordernum ){
      //顺便把已经�?�了的标记一�?
      tmp = getOneOrder(0).desPos; // 手上有快递就先�?�快�?
      tarid = -2;//正在送快�?
    }
    else if(tarid >= -1){ // very slow 
      int D[N];
      for(int i=0;i<N;i++) D[i] = calc_direct(now , P[i]);
      for(int i=0;i<N;i++) for(int j=0;j<N;j++) 
        D[j] = min(D[j] , D[i] + dis[i][j]);

      int mn = 0x3f3f3f3f;
      Position_edc24 minLoc;
      for(int i=0;i<cnt_order;i++)
        if(orders[i].isgot == 0){ // commission of orders delivered  will be set to 0
          int DIS = calc_direct(now,orders[i].depPos);
          for(int j=0;j<N;j++) DIS = min(DIS , D[j] + CD[i][j][0]);
          if(DIS < mn)
            mn = DIS ,
            minLoc = orders[i].depPos , 
            tarid = i;
        }
      if(mn!=0x3f3f3f3f) tmp = minLoc;
      //u1_printf(":355\n");
    }
    */

    /*
      第二稿 ： 手上有x个快递，爆搜（剪枝）接下来 10-x步是拿/送哪个快递，最优化目标：分数 / 时间
      距离计算 ： 使用欧几里得距离。
      如何剪枝 ： 使用IDA* ， 平均每段算 2s ， 计算送完当前快递 + 剩下场上最大的快递。（需要写个堆？）
    */
    
    bst = tmp , bstsc = 0;
    int maxdep = 10 - ordernum;
    for(dep = 1;dep <= maxdep;dep ++){
      have_order_cnt = 0;
      for(int i=0;i<ordernum;i++){
        int id = getOneOrder(i).orderId;
        for(int j=0;j<cnt_order;j++)
          if(orders[j].orderId == id){
            get_time[have_order_cnt] = Get_time[j];
            have_order[have_order_cnt ++] = orders[j];
          }
      }
      st_time = getGameTime() / 1000.0;
      dfs(0 , 0 , st_time , now);
    }
    tmp = bst;

    if(tmpid != -1 &&  Touch(now, temp_goal) ){
      tmpid = gnxt[tmpid];
      if(tmpid == -1)
        temp_goal = ulti_goal;
      else
        temp_goal = P[tmpid];
    }
    if(tmp . x != ulti_goal . x || tmp.y != ulti_goal.y){
      //u1_printf(":365\n");
      ulti_goal = tmp;
      int D[N],E[N];
      for(int i=0;i<N;i++)
        D[i] = calc_direct(ulti_goal , P[i]),
        E[i] = calc_direct(now , P[i]);

      int mn = 0x3f3f3f3f , lst = -1;
      for(int i=0;i<N;i++)
        for(int j=0;j<N;j++)
          if(D[j] + E[i] + dis[i][j] < mn){
            mn = D[j] + E[i] + dis[i][j];
            tmpid = i;
            temp_goal = P[tmpid];
            lst = j;
          }
      if(calc_direct(now , ulti_goal) < mn){
        temp_goal = ulti_goal;
        mn = 1;
        tmpid = -1;
      }
      else{
        gnxt[lst] = -1;
        for(int i = tmpid;i != lst; i = gnxt[i])
          gnxt[i] = nxt[i][lst];
      }
      //if(mn == 0x3f3f3f3f) u1_printf("error\n");
      //u1_printf(":390\n");
    }
    
    // get a new temp_goal
    

    // and go 
    now = getVehiclePos();
    
    if(now.x == temp_goal.x  && now.y == temp_goal.y)
      SetGoal(0,0);
    else{
      int distance = (temp_goal.x - now.x)*(temp_goal.x - now.x) + (temp_goal.y - now.y)*(temp_goal.y - now.y);
      double theta = - atan2(temp_goal.y - now.y, temp_goal.x - now.x) , d = GetYaw() / 180 * pi - theta;
      while(d <= -pi + 1e-7) d += pi;
      while(d >= pi - 1e-7) d -= pi;
      if(d < 0) d = -d;
      if(d > 0.01)
        SetGoal(theta , distance>=20*20?60:distance>=10*10?40:15);
      else 
        SetGoal(theta , distance>=20*20?200:distance>=10*10?60:15);
    }
      
    static int cc = 0;
    if(cc++ % 10 == 0){
      u1_printf("(%d,%d) (%d,%d) (%d,%d)\n",now.x, now.y, temp_goal.x, temp_goal.y, ulti_goal.x, ulti_goal.y);
      u1_printf("%.lf\n",GetYaw());
    }
  }
   
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  jy62_Init(&huart2);
  HAL_TIM_Base_Start_IT(&htim1); //使能定时器TIM1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //使能定时器TIM8的四个�?�道为PWM输出
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //使能四个编码�????
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
  last_fix = getGameTime() , start_fix = getGameTime();
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
      if(getGameStatus() == GameStandby){
        SetGoal(0,0);
        continue;
      }
      int stage = getGameStage();
      if(stage == Prematch) continue;
      if(stage == FirstHalf){
          Solve1();
      }
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
