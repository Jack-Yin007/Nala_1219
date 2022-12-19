/*
 *
 * File:    ars_queue.h
 * Purpose: Definitions for handling different types of queue (Arsenal)
 *
 * Author:  J. Bar On
 */

#ifndef _ARS_QUEUE_H_
#define _ARS_QUEUE_H_

#include <stdint.h>

 /* Simple queue implementation of any basic type or a struct/union */
#define ARS_QUEUE_DEFINE_TYPE(qType, elType, max)   \
    typedef struct {                                \
        int       head;                             \
        int       tail;                             \
        size_t    size;                             \
        elType    queue[(max)];                     \
    } qType
#define ARS_QUEUE_DEFINE(qType, name)  \
    qType name

#define ARS_QUEUE_RESET(q)          ((q).head = (q).tail = -1)
#define ARS_QUEUE_INIT(q, s)        (ARS_QUEUE_RESET(q), (q).size = MIN((s), ARRAY_SIZE((q).queue)))
#define ARS_QUEUE_COUNT(q)          ((size_t)((q).tail - (q).head))
#define ARS_QUEUE_IS_EMPTY(q)       ((q).head == (q).tail)  // or (0 == ARS_QUEUE_COUNT(q))
#define ARS_QUEUE_IS_FULL(q)        ((q).size == ARS_QUEUE_COUNT(q))
#define ARS_QUEUE_PUSH(q, e)        (ARS_QUEUE_IS_FULL(q) ? false : ((q).queue[++(q).tail % (q).size] = (e), true))
#define ARS_QUEUE_PUSH_CYCLIC(q, e) ((ARS_QUEUE_IS_FULL(q) ? ++(q).head : 0), (q).queue[++q.tail % (q).size] = (e), true)
/*  Safe version of macro to avoid GCC warning when the target buffer is on the stack (local variable):
    warning: the comparison will always evaluate as 'false' for the address of 'data' will never be NULL [-Waddress]
 */
#define ARS_QUEUE_POP_SAFE(q, pe)   (ARS_QUEUE_IS_EMPTY(q) ? \
                                        false : \
                                        (*(pe) = (q).queue[++(q).head % (q).size], true) \
                                    )
#define ARS_QUEUE_POP(q, pe)        ((NULL == (pe)) ? false : ARS_QUEUE_POP_SAFE(q, pe))
#define ARS_QUEUE_PEEK(q, index)    ((size_t)(index) < ARS_QUEUE_COUNT(q) ? &((q).queue[((q).tail - (index)) % (q).size]) : NULL)

#endif /* _ARS_QUEUE_H_ */
