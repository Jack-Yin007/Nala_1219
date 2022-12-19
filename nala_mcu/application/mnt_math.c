/*
    math.c 
    minimal math functions
*/

#include "mnt_math.h"

#define NUMBER_OF_GUESSES 15

double mnt_sqrt(double a) {
    volatile double x = a;
    int i;
    if (x <= 0.0)
        return x;
    for (i = 0; i < NUMBER_OF_GUESSES; i++) {
        x = 0.5 * (a / x + x);
    }
    return x;
}


