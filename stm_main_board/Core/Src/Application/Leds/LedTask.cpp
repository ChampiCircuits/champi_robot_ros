#include "Application/Leds/LedTask.h"

#include "math.h"
#include "usart.h"
#include <vector>

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds/Leds.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "tim.h"
#include "semphr.h"
#include "cmsis_os2.h"





osThreadId_t LedTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "fun_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};



std::vector<std::vector<int>> colors = {
  {0,0,0},{83,89,96},{125,88,91},{122,124,127},{119,120,124},{114,74,78},{64,70,77},{0,0,0},
  {83,96,102},{175,86,89},{175,150,152},{244,244,244},{243,243,243},{168,140,142},{164,64,65},{57,64,70},
  {145,151,154},{184,84,85},{187,186,188},{243,243,243},{181,180,181},{173,61,62},{123,130,135},{172,178,180},
  {182,82,83},{169,114,116},{225,226,226},{223,224,225},{163,102,104},{167,57,59},{145,150,153},{129,102,105},
  {205,73,73},{207,70,70},{155,70,72},{150,65,67},{194,53,53},{189,48,48},{105,73,78},{120,83,86},
  {164,79,79},{165,143,137},{186,173,163},{186,174,163},{165,140,133},{159,58,59},{104,53,56},{85,85,85},
  {136,135,132},{215,198,184},{186,174,164},{186,173,163},{221,201,186},{118,115,114},{28,57,57},{0,0,0},
  {78,89,94},{173,163,156},{182,171,161},{183,171,161},{166,156,148},{61,72,72},{0,0,0}
};


//----------------------------------------------------------------------------//
//-------------------------------ANIMATIONS-----------------------------------//
//----------------------------------------------------------------------------//

void explosion_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  Set_LED(0, 255, 255, 255);
  Set_Ring_Brightness(LED_RING_BRIGHTNESS);
  WS2812_Send();

  static int pos_1 = 0;
  static int pos_2 = MAX_LED-1;
  static int in_explosion = 0;

  if (in_explosion == 0)
  {
    // applique
    Set_LED(pos_1, 255, 0, 0);
    Set_LED(pos_2, 0, 255, 255);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
    WS2812_Send();

    osDelay(10);
    //efface
    Set_LED(pos_1, 0, 0, 0);
    Set_LED(pos_2, 0, 0, 0);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
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
      pos_1 = 2;
      pos_2 = MAX_LED-1;
      in_explosion = 0;
      for (int t=255;t>=0;t-=3) {
        for (int i=2;i<MAX_LED;i++) {
          Set_LED(i, t, t, 0);
        }
        WS2812_Send();
        osDelay(1);
      }
      osDelay(3000);
    }
    Set_LED(pos_1, 255, 255, 0);
    Set_LED(pos_2, 255, 255, 0);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
    WS2812_Send();
    pos_1++;
    pos_2--;
    osDelay(1);
  }

}
void unique_led_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static int count_red = 2;
  static int count_blue = MAX_LED-1;
  Set_LED(count_red, 255, 0, 0);
  Set_LED(count_blue, 0, 0, 255);

  if (count_red > 2)
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
  if (count_blue <= 2)
  {
    Set_LED(1, 0, 0, 0);
    count_blue = MAX_LED-1;
  }
  Set_Ring_Brightness(LED_RING_BRIGHTNESS);
  WS2812_Send();
  osDelay(20);
}
void snake_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static int offset = 0;
  static float step = 255/50;
  // generation du tableau
  static int leds_val[MAX_LED];
  for (int i=2;i<15;i++)
  {
    leds_val[i] = 1/(1+exp((((MAX_LED-i)*step/21)-6)*-1))*255;

  }
  for (int i=15;i<MAX_LED;i++)
  {
    leds_val[i] = 0;
  }

  int c;
  for (int i=2;i<MAX_LED;i++)
  {
    c = i+offset;
    if (c>MAX_LED)
      c -= MAX_LED;
    Set_LED(i, 0, 0, leds_val[c]);
  }
  Set_Ring_Brightness(LED_RING_BRIGHTNESS);
  WS2812_Send();
  osDelay(50);


  offset+=1;
  if (offset>MAX_LED)
    offset = 0;
}
void brush_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static int ON = 1;
  static int red;

  if (ON==1)
    red = 255;
  else
    red = 0;

  for (int i=2;i<MAX_LED;i++) {
    Set_LED(i, red, 0, 0);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
    WS2812_Send();
    osDelay(18);
  }
  if (ON==1)
    ON = 0;
  else
    ON = 1;
}
void mistake_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static int count = 2;
  static int ON = 1;
  static int red;

  if (ON==1)
    red = 255;
  else
    red = 0;

  for (int i=MAX_LED-1;i>count;i--) {
    Set_LED(i, red, 0, 0);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
    WS2812_Send();
    osDelay(20);
  }
  if (ON==1)
    ON = 0;
  else
    ON = 1;


  count++;
  if (count==MAX_LED)
    count = 2;
}
void loading_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static int count = 0;
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

	for (int i=2;i<count;i++) {
	  h = i*10;
	  HSVtoRGB(&r,&g,&b,h,s,v);
		Set_LED(i, r, g, b); // R G B
	}
  Set_Ring_Brightness(brightness);
  WS2812_Send();


  for (int i=MAX_LED-1;i>count;i--) {
    h = i*10;
    HSVtoRGB(&r,&g,&b,h,s,v);
    Set_LED(i, r, g, b);
    if (i+1<MAX_LED) {
    	Set_LED(i+1, 0, 0, 0);
    }

    Set_Ring_Brightness(brightness);
    WS2812_Send();
    osDelay(20);
  }

  count++;
  if (count==MAX_LED)
    count = 0;
}
void hyper_style_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

  static float step = 360/MAX_LED;
  static int offset = 0;

  for (int i=2;i<MAX_LED;i++) {
    HSVtoRGB(&r,&g,&b,h,s,v);
    Set_LED(i, r, g, b);
    h+=step+offset;
    if (h>360)
      h = h-360;
  }
  Set_Ring_Brightness(brightness);
  WS2812_Send();
  osDelay(100);
  offset+=1;
}
void turning_rainbow_animation() {
  static float brightness = LED_RING_BRIGHTNESS;
  static float h,s,v, r,g,b;
  h = 0;
  s = 1;
  v = 1;

  static float step = 360/MAX_LED;
  static int offset = 0;

  // ANIMATION
  h = offset;
  for (int i=2;i<MAX_LED;i++) {
    HSVtoRGB(&r,&g,&b,h,s,v);
    Set_LED(i, r, g, b);
    h+=step;
    if (h>=360)
      h = h-360;
  }
  Set_Ring_Brightness(brightness);
  WS2812_Send();
  offset+=5;
  if (offset>=360)
    offset = 0;
}
void BAU_pushed_animation() {
  float step = 0.5;
  static float brightness = 0;
  static int dir = 1;

  // ANIMATION
  for (int i=2;i<MAX_LED;i++) {
    Set_LED(i, 255, 0, 0);
  }
  Set_Ring_Brightness(brightness);
  brightness += dir * step;

  if (brightness > LED_RING_BRIGHTNESS or brightness < 0.0)
    dir *= -1;

}
//----------------------------------------------------------------------------//
//---------------------------------TASK---------------------------------------//
//----------------------------------------------------------------------------//

struct LedState {
  int color;
  float brightness;
};

void applyStatusLedState(const LedState& state, int ledIndex) {
  switch (state.color) {
  case CLEAR:
    led::clear(ledIndex, state.brightness);
    break;
  case GREEN:
    led::setGreen(ledIndex, state.brightness);
    break;
  case RED:
    led::setRed(ledIndex, state.brightness);
    break;
  case ORANGE:
    led::setOrange(ledIndex, state.brightness);
    break;
  case BLUE:
    led::setBlue(ledIndex, state.brightness);
    break;
  }
}


void LedTask(void *argument) {
  clear_Ring();

//  RCC_ClkInitTypeDef clkconfig;
//  RCC_OscInitTypeDef oscconfig;
//  uint32_t pFLatency;
//
//  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
//  HAL_RCC_GetOscConfig(&oscconfig);
//
//  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
//  uint32_t hclk   = HAL_RCC_GetHCLKFreq();
//  uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
//  uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();

//  osDelay(500);
//  LOG_INFO("led", "SYSCLK  = %lu Hz\n", sysclk);
//  LOG_INFO("led", "HCLK    = %lu Hz\n", hclk);
//  LOG_INFO("led", "PCLK1   = %lu Hz\n", pclk1);
//  LOG_INFO("led", "PCLK2   = %lu Hz\n", pclk2);

  LOG_INFO("led", "Starting loop.");
  osDelay(100);

//  while (true) {
//	  for (int i = 0; i < MAX_LED; i++) {
//		  applyStatusLedState({COLOR::ORANGE, 50}, i);
//	  }
//	  WS2812_Send();
//	  Set_Ring_Brightness(100);
//	  osDelay(100);
//  }

  while (true) {
    // for (int i=0;i<MAX_LED;i++)
    // {
    //   applyStatusLedState({GREEN, led_holo::brightness}, i);
    // }
    // applyStatusLedState({led_otos::color, led_otos::brightness}, LED_OTOS);

    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    bool e_stop_pressed = mod_reg::state->e_stop_pressed;
    xSemaphoreGive(ModbusH.ModBusSphrHandle);

    if (e_stop_pressed) {
      BAU_pushed_animation();
    }
    else {
       turning_rainbow_animation();
//      float brightness = 50; // ajuste si besoin
//      int startIndex = 25;
//
//      for (int row = 0; row < 8; ++row) {
//        for (int col = 0; col < 8; ++col) {
//          int indexInColors;
//          if (row % 2 == 0) {
//            // ligne paire → gauche à droite
//            indexInColors = row * 8 + col;
//          } else {
//            // ligne impaire → droite à gauche
//            indexInColors = row * 8 + (7 - col);
//          }
//
//          int r = colors[indexInColors][0];
//          int g = colors[indexInColors][1];
//          int b = colors[indexInColors][2];
//          int ledIndex = startIndex + row * 8 + col;
//
//          led::setColor(ledIndex, r, g, b, brightness);
//        }
//      }

    }
    // Set_Ring_Brightness(100);

    WS2812_Send();
    // TODO appelé plusieurs fois
    osDelay(50);
  }
}

void LedTaskStart() {
  LedTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);
}
