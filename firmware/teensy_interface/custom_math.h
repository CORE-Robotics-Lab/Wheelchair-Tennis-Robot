#ifndef CUSTOM_MATH_H
#define CUSTOM_MATH_H

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define between(lower, x, upper) ((x >= lower) && (x <= upper))

// Alternative map() function that computes using floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

#endif