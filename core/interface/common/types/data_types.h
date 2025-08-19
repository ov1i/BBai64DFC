#ifndef DATA_TYPES_H
#define DATA_TYPES_H

/// @brief boolean [ 0 : false; 1 : true ]
typedef unsigned char       boolean;

/// @brief TRUE = 1u
#ifndef TRUE
#define TRUE 1u
#endif

/// @brief FALSE = 0u
#ifndef FALSE
#define FALSE 0u
#endif

/// @brief uint8 [ 0 : 255 -> 8 bit unsigned integer ]
typedef unsigned char       uint8;

/// @brief uint16 [ 0 : 65535 -> 16 bit unsigned integer ]
typedef unsigned short      uint16;

/// @brief uint32 [ 0 : 4294967296 -> 32 bit unsigned integer ]
typedef unsigned int        uint32;

/// @brief  uint64 [ 0 : 1.844674407E+19 -> 64 bit unsigned integer ] !platform based x86/64
#ifdef __x86_64__
typedef unsigned long       uint64;
#else
typedef unsigned long long  uint64;
#endif

/// @brief uint8 [ -128 : 127 -> 8 bit signed integer ]
typedef signed char         sint8;

/// @brief uint16 [ -32768 : 32767 -> 16 bit signed integer ]
typedef signed short        sint16;

/// @brief uint32 [ -2147483648 : 2147483647 -> 32 bit signed integer ]
typedef signed int          sint32;

/// @brief  uint64 [ -9.223372035E+18 : 9.223372034E+18 -> 64 bit signed integer ] !platform based x86/64
#ifdef __x86_64__
typedef signed long         sint64;
#else
typedef signed long long    sint64;
#endif

/// @brief float32 [ -3.4028235E+38 : +3.4028235E+38 -> 32 bit signed float ]
typedef float               float32;

/// @brief float64 [ -1.7976931348623157E+308 : +1.7976931348623157E+308 -> 64 bit signed float ]
typedef double              float64;

/// @brief puint8 -> pointer to uint8
typedef uint8*              puint8;

/// @brief puint16 -> pointer to uint16
typedef uint16*             puint16;

/// @brief puint32 -> pointer to uint32
typedef uint32*             puint32;

/// @brief puint64 -> pointer to uint64
typedef uint64*             puint64;

/// @brief psint8 -> pointer to sint8
typedef sint8*              psint8;

/// @brief psint16 -> pointer to sint16
typedef sint16*             psint16;

/// @brief psint32 -> pointer to sint32
typedef sint32*             psint32;

/// @brief psint64 -> pointer to sint64
typedef sint64*             psint64;

/// @brief pfloat32 -> pointer to float32
typedef float32*            pfloat32;

/// @brief pfloat64 -> pointer to float64
typedef float64*            pfloat64;

/// @brief pvoid -> pointer to void
typedef void*               pvoid;


#endif