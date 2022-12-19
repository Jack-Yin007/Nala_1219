#ifndef __MNT_MATH_H__
#define __MNT_MATH_H__

//#define PI 3.141592654
#define HALFPI 1.570796327
#define TWOPI 6.283185307
#define RADIANS_PER_DEGREE 1.74532952e-2
#define DEGREES_PER_RADIAN 57.29577951
#define ROOT_2 1.414213562
#define SIN_45 0.707106781

//#define double float

typedef union {
    double fp;

    struct {
        unsigned int lo;
        unsigned int hi;
    } n;
} hack_structure;

double mnt_sqrt(double a);

#endif // #ifndef __MNT_MATH_H__
