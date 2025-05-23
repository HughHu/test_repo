/*
 * luna_math_types.h
 *
 *  Created on: 2020��9��4��
 *      Author: dwwang3
 */

#ifndef __LUNA_MATH_TYPES_H_SIM__
#define __LUNA_MATH_TYPES_H_SIM__

#include <stdint.h>
#include <math.h>

typedef int8_t 		q7_t;
typedef int16_t 	q15_t;
typedef int32_t 	q31_t;
typedef int64_t 	q63_t;
typedef float 		float32_t;
typedef double		float64_t;

typedef int8_t 		int4_t;

#ifndef bool
#define bool int8_t
#endif

#if defined(WIN32) || defined(linux)
#define LUNA_API_SIM(api) api
#else 
#define LUNA_API_SIM(api) luna_##api##_sim
#endif 

#endif // __LUNA_MATH_TYPES_H_SIM__

