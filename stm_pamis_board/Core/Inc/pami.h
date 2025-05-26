#ifndef PAMI_H
#define PAMI_H

#include "Config/DEFINE_PAMI.h"

#ifdef PAMI_1
  #define ID_SERVO_DIR 18
  #define ID_SERVO_TRACTION 17
#endif
  // TODO
#ifdef PAMI_2
#endif
#ifdef PAMI_3
  // TODO
#endif
#ifdef PAMI_SUPERSTAR
  // TODO
#endif


void PAMI_Init();
void PAMI_Loop();

#endif //PAMI_H
