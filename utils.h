#ifndef UTILS_H
#define UTILS_H

#include <stddef.h>
#include <math.h>

char* ltoa(long value, char *string, int radix);
void ftoa(float n, char* res, int afterpoint);
void reverse(char* str, int len);
int intToStr(int x, char str[], int d);

#endif
