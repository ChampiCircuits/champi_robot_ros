#ifndef LEDS_H
#define LEDS_H

#include "tim.h"


#define MAX_LED (24+2)
#define USE_BRIGHTNESS 1

#define PI 3.14159265

#define LED_HOLO 0
#define LED_OTOS 1


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


namespace led
{
inline void setColor(int ledIndex, float r, float g, float b, float brightness) {
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
inline float brightness = 10;

inline void clear()   { led::clear(LED_HOLO, brightness); }
inline void setGreen(){ led::setGreen(LED_HOLO, brightness); }
inline void setRed()  { led::setRed(LED_HOLO, brightness); }
inline void setOrange(){ led::setOrange(LED_HOLO, brightness); }
inline void setBlue() { led::setBlue(LED_HOLO, brightness); }

inline void SetBrightness(float b) { brightness = b; }
}

namespace led_otos
{
inline float brightness = 10;

inline void clear()   { led::clear(LED_OTOS, brightness); }
inline void setGreen(){ led::setGreen(LED_OTOS, brightness); }
inline void setRed()  { led::setRed(LED_OTOS, brightness); }
inline void setOrange(){ led::setOrange(LED_OTOS, brightness); }
inline void setBlue() { led::setBlue(LED_OTOS, brightness); }

inline void SetBrightness(float b) { brightness = b; }
}



#endif //LEDS_H
