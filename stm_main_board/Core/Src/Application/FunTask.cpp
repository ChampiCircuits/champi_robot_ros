#include "Application/FunTask.h"

#include "usart.h"
#include "math.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Util/logging.h"
#include "tim.h"

#include "cmsis_os2.h"

#define MAX_LED 24
#define USE_BRIGHTNESS 0


volatile int datasentflag = 0;
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness


osThreadId_t FunTaskHandle;
const osThreadAttr_t funTask_attributes = {
  .name = "fun_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};


/*! \brief Convert RGB to HSV color space

  Converts a given set of RGB values `r', `g', `b' into HSV
  coordinates. The input RGB values are in the range [0, 1], and the
  output HSV values are in the ranges h = [0, 360], and s, v = [0,
  1], respectively.

  \param fR Red component, used as input, range: [0, 1]
  \param fG Green component, used as input, range: [0, 1]
  \param fB Blue component, used as input, range: [0, 1]
  \param fH Hue component, used as output, range: [0, 360]
  \param fS Hue component, used as output, range: [0, 1]
  \param fV Hue component, used as output, range: [0, 1]

*/
void RGBtoHSV (float fR, float fG, float fB, float *fH, float *fS, float *fV) {
  float fCMax = fmax(fmax(fR, fG), fB);
  float fCMin = fmin(fmin(fR, fG), fB);
  float fDelta = fCMax - fCMin;

  if(fDelta > 0) {
    if(fCMax == fR) {
      *fH = 60 * (fmod(((fG - fB) / fDelta), 6));
    } else if(fCMax == fG) {
      *fH = 60 * (((fB - fR) / fDelta) + 2);
    } else if(fCMax == fB) {
      *fH = 60 * (((fR - fG) / fDelta) + 4);
    }

    if(fCMax > 0) {
      *fS = fDelta / fCMax;
    } else {
      *fS = 0;
    }

    *fV = fCMax;
  } else {
    *fH = 0;
    *fS = 0;
    *fV = fCMax;
  }

  if(*fH < 0.) {
    *fH = 360. + *fH;
  }
}


/*! \brief Convert HSV to RGB color space

  Converts a given set of HSV values `h', `s', `v' into RGB
  coordinates. The output RGB values are in the range [0, 255], and
  the input HSV values are in the ranges h = [0, 360], and s, v =
  [0, 1], respectively.

  \param fR Red component, used as output, range: [0, 1]
  \param fG Green component, used as output, range: [0, 1]
  \param fB Blue component, used as output, range: [0, 1]
  \param fH Hue component, used as input, range: [0, 360]
  \param fS Hue component, used as input, range: [0, 1]
  \param fV Hue component, used as input, range: [0, 1]

*/
void HSVtoRGB (float *fR, float *fG, float *fB, float fH, float fS, float fV) {

  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if(0 <= fHPrime && fHPrime < 1) {
    *fR = fC;
    *fG = fX;
    *fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    *fR = fX;
    *fG = fC;
    *fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    *fR = 0;
    *fG = fC;
    *fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    *fR = 0;
    *fG = fX;
    *fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    *fR = fX;
    *fG = 0;
    *fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    *fR = fC;
    *fG = 0;
    *fB = fX;
  } else {
    *fR = 0;
    *fG = 0;
    *fB = 0;
  }

  *fR += fM;
  *fG += fM;
  *fB += fM;

  *fR*=255;
  *fG*=255;
  *fB*=255;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = Green;
  LED_Data[LEDnum][2] = Red;
  LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

  if (brightness > 45) brightness = 45;
  for (int i=0; i<MAX_LED; i++)
  {
    LED_Mod[i][0] = LED_Data[i][0];
    for (int j=1; j<4; j++)
    {
      float angle = 90-brightness;  // in degrees
      angle = angle*PI / 180;  // in rad
      LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
    }
  }

#endif

}



uint16_t pwmData[(24*MAX_LED)+200];

void WS2812_Send (void)
{
  uint32_t indx=0;
  uint32_t color;


  for (int i= 0; i<MAX_LED; i++)
  {
#if USE_BRIGHTNESS
    color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
    color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

    for (int i=23; i>=0; i--)
    {
      if (color&(1<<i))
      {
        pwmData[indx] = 142;  // 2/3 of 215
      }

      else pwmData[indx] = 71;  // 1/3 of 215

      indx++;
    }

  }

  for (int i=0; i<200; i++)
  {
    pwmData[indx] = 0;
    indx++;
  }

  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)pwmData, indx);
  while (!datasentflag){};
  datasentflag = 0;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
  datasentflag=1;
}



void clear()
{
  for (int i=0;i<MAX_LED;i++)
  {
    Set_LED(i, 0, 0, 0);
  }
  WS2812_Send();
}

void explosion_animation()
{
  Set_LED(0, 255, 255, 255);
  WS2812_Send();

  static int pos_1 = 0;
  static int pos_2 = MAX_LED-1;
  static int in_explosion = 0;

  if (in_explosion == 0)
  {
    // applique
    Set_LED(pos_1, 255, 0, 0);
    Set_LED(pos_2, 0, 255, 255);
    WS2812_Send();

    osDelay(100);
    //efface
    Set_LED(pos_1, 0, 0, 0);
    Set_LED(pos_2, 0, 0, 0);
    WS2812_Send();

    //update
    pos_1++;
    pos_2--;


    // si les pos sont égales, on explose
    if (pos_1 == pos_2+1 || pos_1 == pos_2-1 || pos_1 == pos_2)
    {
      in_explosion = 1;
    }
  }
  else
  {
    if (pos_1==MAX_LED || pos_2<0)
    {
      pos_1 = 0;
      pos_2 = MAX_LED-1;
      in_explosion = 0;
      for (int t=255;t>=0;t-=3) {
        for (int i=0;i<MAX_LED;i++) {
          Set_LED(i, t, t, 0);
        }
        WS2812_Send();
        osDelay(10);
      }
      osDelay(3000);
    }
    Set_LED(pos_1, 255, 255, 0);
    Set_LED(pos_2, 255, 255, 0);
    WS2812_Send();
    pos_1++;
    pos_2--;
    osDelay(20);
  }

}

void unique_led_animation()
{
  static int count_red = 0;
  static int count_blue = MAX_LED-1;
  Set_LED(count_red, 255, 0, 0);
  Set_LED(count_blue, 0, 0, 255);

  if (count_red > 0)
    Set_LED(count_red-1, 0, 0, 0);
  if (count_blue < MAX_LED-1)
    Set_LED(count_blue+1, 0, 0, 0);
  count_red++;
  count_blue--;
  if (count_red >= MAX_LED)
  {
    Set_LED(MAX_LED-1, 0, 0, 0);
    count_red = 0;
  }
  if (count_blue <= 0)
  {
    Set_LED(1, 0, 0, 0);
    count_blue = MAX_LED-1;
  }
  WS2812_Send();
  osDelay(20);
}
void snake_animation()
{
  static int offset = 0;
  static float step = 255/50;
  // generation du tableau
  static int leds_val[MAX_LED];
  for (int i=0;i<15;i++)
  {
    leds_val[i] = 1/(1+exp((((MAX_LED-i)*step/21)-6)*-1))*255;

  }
  for (int i=15;i<MAX_LED;i++)
  {
    leds_val[i] = 0;
  }

  int c;
  for (int i=0;i<MAX_LED;i++)
  {
    c = i+offset;
    if (c>MAX_LED)
      c -= MAX_LED;
    Set_LED(i, 0, 0, leds_val[c]);
  }

  WS2812_Send();
  osDelay(50);


  offset+=1;
  if (offset>MAX_LED)
    offset = 0;
}
void brush_animation() {
  static int ON = 1;
  static int red;

  if (ON==1)
    red = 255;
  else
    red = 0;

  for (int i=0;i<MAX_LED;i++) {
    Set_LED(i, red, 0, 0);
    WS2812_Send();
    osDelay(18);
  }
  if (ON==1)
    ON = 0;
  else
    ON = 1;
}

void mistake_animation() {
  static int count = 0;
  static int ON = 1;
  static int red;

  if (ON==1)
    red = 255;
  else
    red = 0;

  for (int i=MAX_LED-1;i>count;i--) {
    Set_LED(i, red, 0, 0);
    WS2812_Send();
    osDelay(20);
  }
  if (ON==1)
    ON = 0;
  else
    ON = 1;


  count++;
  if (count==MAX_LED)
    count = 0;
}

void loading_animation() {
  static int count = 0;
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

	for (int i=0;i<count;i++) {
	  h = i*10;
	  HSVtoRGB(&r,&g,&b,h,s,v);
		Set_LED(i, r, g, b); // R G B
	}
  WS2812_Send();


    for (int i=MAX_LED-1;i>count;i--) {
      h = i*10;
      HSVtoRGB(&r,&g,&b,h,s,v);
    	Set_LED(i, r, g, b);
    	if (i+1<MAX_LED)
    		Set_LED(i+1, 0, 0, 0);
    	WS2812_Send();
    	osDelay(20);
    }

    count++;
    if (count==MAX_LED)
    	count = 0;
}

void hyper_style_animation()
{
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

  static float step = 360/MAX_LED;
  static int offset = 0;

  for (int i=0;i<MAX_LED;i++) {
    HSVtoRGB(&r,&g,&b,h,s,v);
    Set_LED(i, r, g, b);
    h+=step+offset;
    if (h>360)
      h = h-360;
  }
  WS2812_Send();
  osDelay(100);
  offset+=1;
}

void turning_rainbow_animation()
{
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

  static float step = 360/MAX_LED;
  static int offset = 0;

  // ANIMATION
  h = offset;
  for (int i=0;i<MAX_LED;i++) {
    HSVtoRGB(&r,&g,&b,h,s,v);
    Set_LED(i, r, g, b);
    h+=step;
    if (h>=360)
      h = h-360;
  }
  WS2812_Send();
  osDelay(1);
  offset+=1;
  if (offset>=360)
    offset = 0;

}







void FunTask(void *argument) {


  LOG_INFO("fun", "Starting loop.");


  while (true) {
    clear();
    //loading_animation();
    osDelay(100);
  }
}

void FunTaskStart() {
  FunTaskHandle = osThreadNew(FunTask, NULL, &funTask_attributes);
}
