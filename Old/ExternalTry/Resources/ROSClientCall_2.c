// A basic external function, used for debugging Modelica's capability of connecting to C

#include <stdio.h>

double ROSClientCall(double time, double x)
{
    printf("time: %f\n", time);
    return 15*(2-x);
}