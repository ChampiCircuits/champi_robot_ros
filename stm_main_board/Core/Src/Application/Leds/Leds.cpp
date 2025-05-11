#include "Application/Leds/Leds.h"

namespace led_holo
{
float brightness = 5;
int color;
}

namespace led_otos
{
float brightness = 5;
int color;
}

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness
uint16_t pwmData[(24*MAX_LED)+200];
volatile int datasentflag = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
  datasentflag=1;
}


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

void Set_Ring_Brightness (float brightness)  // [0-100] Must be done after Set_LED
{
#if USE_BRIGHTNESS
  brightness /= 100;
  brightness *= 45;

  if (brightness > 45) brightness = 45;
  for (int i=2; i<MAX_LED; i++)
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



void clear_Ring()
{
  for (int i=2;i<MAX_LED;i++)
  {
    Set_LED(i, 0, 0, 0);
  }
  WS2812_Send();
}


void Set_Brightness(float brightness, int led)  // [0-100] Must be done after Set_LED
{
#if USE_BRIGHTNESS
  brightness /= 100;
  brightness *= 45;

if (brightness > 45) brightness = 45;

  LED_Mod[led][0] = LED_Data[led][0];
  for (int j=1; j<4; j++)
  {
    float angle = 90-brightness;  // in degrees
    angle = angle*PI / 180;  // in rad
    LED_Mod[led][j] = (LED_Data[led][j])/(tan(angle));
  }

#endif
}