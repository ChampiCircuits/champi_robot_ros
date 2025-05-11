#include "Application/Leds/LedTask.h"

#include "math.h"
#include "usart.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds/Leds.h"
#include "Config/Config.h"
#include "Util/logging.h"
#include "tim.h"

#include "cmsis_os2.h"





osThreadId_t LedTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "fun_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
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

    osDelay(100);
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
        osDelay(10);
      }
      osDelay(3000);
    }
    Set_LED(pos_1, 255, 255, 0);
    Set_LED(pos_2, 255, 255, 0);
    Set_Ring_Brightness(LED_RING_BRIGHTNESS);
    WS2812_Send();
    pos_1++;
    pos_2--;
    osDelay(20);
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
  WS2812_Send();

  LOG_INFO("led", "Starting loop.");
  osDelay(100);

  while (true) {
    applyStatusLedState({led_holo::color, led_holo::brightness}, LED_HOLO);
    applyStatusLedState({led_otos::color, led_otos::brightness}, LED_OTOS);

    if (led_ring::e_stop_pressed) {
      BAU_pushed_animation();
    }
    else {
      turning_rainbow_animation();
    }
    WS2812_Send();

    osDelay(50);
  }
}

void LedTaskStart() {
  LedTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);
}
