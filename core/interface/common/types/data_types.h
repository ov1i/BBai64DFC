#ifndef DFC_BASE_H
#define DFC_BASE_H

#include <stdint.h>
#include <stdbool.h>

#ifndef DFC_HAS_SINT_TYPES
typedef int8_t    sint8;
typedef int16_t   sint16;
typedef int32_t   sint32;
typedef int64_t   sint64;
#define DFC_HAS_SINT_TYPES 1
#endif

#ifndef DFC_HAS_UINT_TYPES
typedef uint8_t   uint8;
typedef uint16_t  uint16;
typedef uint32_t  uint32;
typedef uint64_t  uint64;
#define DFC_HAS_UINT_TYPES 1
#endif

#ifndef DFC_HAS_FLOAT_TYPES
typedef float     float32;
typedef double    float64;
#define DFC_HAS_FLOAT_TYPES 1
#endif

#endif
