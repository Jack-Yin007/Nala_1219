/*
 *
 * File:    core_types.h
 * Purpose: Generic type definitions
 *
 * Author:  J. Bar On
 */

#ifndef _CORE_TYPES_H_
#define _CORE_TYPES_H_

#include <limits.h>     /* For MAX and MIN values for each type */

/* Generic types */
#if defined _WIN32
    /* Visual Studio supports <stdint.h>, but double definition of some min/max macros causes warnings like:
     * 4>c:\program files (x86)\microsoft visual studio 10.0\vc\include\stdint.h(72): warning C4005: 'INT8_MIN' : macro redefinition
     * 4>          c:\program files (x86)\microsoft sdks\windows\v7.0a\include\intsafe.h(144) : see previous definition of 'INT8_MIN'.
     *
     * Define used types from <stdint.h> here.
     * Also, VS defines explicitly bool, true and false for C (which is a non-standard extension).
     */
    typedef signed char         int8_t;
    typedef short               int16_t;
    typedef int                 int32_t;
    typedef __int64             int64_t;

    typedef unsigned char       uint8_t;
    typedef unsigned short      uint16_t;
    typedef unsigned int        uint32_t;
    typedef unsigned __int64    uint64_t;
#else
    #include <stdint.h>         /* For standard integer types */
#endif  /* #ifdef _WIN32 */

#if ! defined _I8_MIN	/* Should come from <limits.h> */
	#define _I8_MIN		(-127 - 1)	// minimum signed 8 bit value
#endif
#if ! defined _I8_MAX	/* Should come from <limits.h> */
	#define _I8_MAX		127			// maximum signed 8 bit value
#endif
#if ! defined _UI8_MAX	/* Should come from <limits.h> */
	#define _UI8_MAX	0xFF		// maximum unsigned 8 bit value
#endif

#if ! defined __cplusplus && ! defined bool && ! defined CORE_BOOL_DEFINED
    typedef uint8_t bool;
    /* Use native compiler false and true values for the definition */
    enum { 
        false   = (0 == 1), 
        true    = ! false
    };
    #define CORE_BOOL_DEFINED
#endif

/* Tri-state bool */
typedef enum {
    TBOOL_NOT_ASSIGNED  = -1,
    TBOOL_FALSE         = false,
    TBOOL_TRUE          = true
} tbool;

#endif /* _CORE_TYPES_H_ */
