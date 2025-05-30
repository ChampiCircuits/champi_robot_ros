#ifndef DEFINE_PAMI_H
#define DEFINE_PAMI_H

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//#define PAMI_1
 #define PAMI_2
// #define PAMI_3
// #define PAMI_SUPERSTAR
//////////////////////////////////////////////////
//////////////////////////////////////////////////


#ifdef PAMI_1
#define DIR_ANGLE_STRAIGHT_CONST 149
#define DIR_ANGLE_RIGHT____CONST 170
#define DIR_ANGLE_LEFT_____CONST 100
#endif

#ifdef PAMI_2
#define DIR_ANGLE_STRAIGHT_CONST 100
#define DIR_ANGLE_RIGHT____CONST 170
#define DIR_ANGLE_LEFT_____CONST 50
#endif

#ifdef PAMI_3
#define DIR_ANGLE_STRAIGHT_CONST 149
#define DIR_ANGLE_RIGHT____CONST 170
#define DIR_ANGLE_LEFT_____CONST 100
#endif

#ifdef PAMI_SUPERSTAR
#define DIR_ANGLE_STRAIGHT_CONST 149
#define DIR_ANGLE_RIGHT____CONST 180
#define DIR_ANGLE_LEFT_____CONST 90
#endif


#endif //DEFINE_PAMI_H
