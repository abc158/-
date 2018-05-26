#ifndef _TYPE_H_
#define _TYPE_H_
#include <stdint.h>

#ifdef __cplusplus
#define     __I     volatile                  /*!< defines 'read only' permissions      */
#else
#define     __I     volatile const            /*!< defines 'read only' permissions      */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef const int32_t sc32;           /*!< Read Only                                                        */
typedef const int16_t sc16;           /*!< Read Only                                                        */
typedef const int8_t  sc8;            /*!< Read Only                                                        */

typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t  vs8;

typedef __I int32_t vsc32;            /*!< Read Only                                                        */
typedef __I int16_t vsc16;            /*!< Read Only                                                        */
typedef __I int8_t  vsc8;             /*!< Read Only                                                        */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  U32;
typedef unsigned short U16;
typedef unsigned char  U8;

typedef int32_t S32;
typedef int16_t S16;
typedef int8_t  S8;

typedef unsigned long  const uc32;    /*!< Read Only                                                        */
typedef unsigned short const uc16;    /*!< Read Only                                                        */
typedef unsigned char  const uc8;     /*!< Read Only                                                        */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;

typedef __I uint32_t vuc32;           /*!< Read Only                                                        */
typedef __I uint16_t vuc16;           /*!< Read Only                                                        */
typedef __I uint8_t  vuc8;            /*!< Read Only                                                        */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState,ControlStatus,EventStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus,ErrStatus;

#ifndef INT64
#define INT64 long long
#endif // AM_INT64
typedef INT64 S64;

#ifndef AM_REAL
#define AM_REAL float
#endif
#define am_real AM_REAL

#ifndef BOOLEAN
    #define BOOLEAN unsigned char
    #ifndef TRUE
        #define TRUE  1
        #define FALSE 0
    #endif
#endif

#ifndef NULL
    #define NULL    ((void *)0)
#endif

typedef S16 TransVel;

#ifndef MIN
    #define MIN(a,b)        (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
    #define MAX(a,b)        (((a) > (b)) ? (a) : (b))
#endif

#endif

