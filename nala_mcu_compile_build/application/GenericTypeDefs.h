/*
 * GenericTypeDefs.h
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#ifndef GENERICTYPEDEFS_H_
#define GENERICTYPEDEFS_H_

typedef unsigned char           BYTE;                           /* 8-bit unsigned  */
typedef unsigned short int      UINT16;                           /* 16-bit unsigned */
typedef unsigned long           UINT32;                          /* 32-bit unsigned */
typedef enum _BOOL { FALSE = 0, TRUE } BOOL;    /* Undefined size */

typedef signed int          INT;
typedef signed char         INT8;
typedef signed short int    INT16;
typedef signed long int     INT32;

typedef unsigned char		uint8;
typedef signed char         int8;
typedef signed short int    int16;

//// PUMAMCU-136
///* Tri-state bool */
//typedef enum {
//    TBOOL_NOT_ASSIGNED = -1,
//    TBOOL_FALSE = FALSE,
//    TBOOL_TRUE = TRUE
//} tbool;

/* Warning clean-up macros */
#define UNUSED_BUT_SET_VARIABLE(var)    ((void)(var))   /* To silence GCC warning: variable 'var' set but not used [-Wunused-but-set-variable] */
#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(par)		((void)(par))	/* To silence GCC warning: unused parameter 'par' [-Wunused-parameter] */
#endif 
#define IS_IN_RANGE(v, l, h)            ((v) >= (l) && (v) <= (h))

#endif /* GENERICTYPEDEFS_H_ */
