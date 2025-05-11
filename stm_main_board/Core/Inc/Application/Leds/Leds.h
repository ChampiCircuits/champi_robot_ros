#ifndef LEDS_H
#define LEDS_H

#include "tim.h"


#define MAX_LED (24+2)
#define USE_BRIGHTNESS 1
#define LED_RING_BRIGHTNESS 10

#define PI 3.14159265

#define LED_HOLO 0
#define LED_OTOS 1

enum COLOR {
  CLEAR = 0,
  RED = 1,
  GREEN = 2,
  BLUE = 3,
  ORANGE = 4
};

void WS2812_Send();


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
void RGBtoHSV(float fR, float fG, float fB, float *fH, float *fS, float *fV);



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
void HSVtoRGB (float *fR, float *fG, float *fB, float fH, float fS, float fV);


void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness(float brightness, int led); // [0-100] Must be done after Set_LED
void Set_Ring_Brightness (float brightness);  // [0-100] Must be done after Set_LED

void clear_Ring();

namespace led_ring {
inline bool e_stop_pressed = false;
}

namespace led
{
inline void setColor(int ledIndex, int r, int g, int b, float brightness) {
  Set_LED(ledIndex, r, g, b);
  Set_Brightness(brightness, ledIndex);
  WS2812_Send();
}

inline void clear(int ledIndex, float brightness) {
  setColor(ledIndex, 0, 0, 0, brightness);
}

inline void setGreen(int ledIndex, float brightness) {
  setColor(ledIndex, 0, 255, 0, brightness);
}

inline void setRed(int ledIndex, float brightness) {
  setColor(ledIndex, 255, 0, 0, brightness);
}

inline void setOrange(int ledIndex, float brightness) {
  setColor(ledIndex, 255, 50, 0, brightness);
}

inline void setBlue(int ledIndex, float brightness) {
  setColor(ledIndex, 0, 0, 255, brightness);
}
}

namespace led_holo
{
extern float brightness;
extern int color;

inline void clear()   { color = CLEAR; }
inline void setGreen(){ color = GREEN; }
inline void setRed()  { color = RED; }
inline void setOrange(){ color = ORANGE; }
inline void setBlue() { color = BLUE; }

inline void SetBrightness(float b) { brightness = b; }
}

namespace led_otos
{
extern float brightness;
extern int color;

inline void clear()   { color = CLEAR; }
inline void setGreen(){ color = GREEN; }
inline void setRed()  { color = RED; }
inline void setOrange(){ color = ORANGE; }
inline void setBlue() { color = BLUE; }

inline void SetBrightness(float b) { brightness = b; }
}



#endif //LEDS_H
