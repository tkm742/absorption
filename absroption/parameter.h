#include <bits/stdc++.h>

#define EPSILON_INTENSITY 1e-4
#define EPSILON_DISTANCE 1e-4

#define N_LINE1 5
#define N_LINE2 2
#define N_ARC1 2
#define N_ARC2 3

#define MODEL1 0
#define MODEL2 1

#define PI 3.141592653589793

#define LEFT false
#define RIGHT true


enum ID_MEDIUM{
    AIR = 0,
    CHLOROPLAST,
    WATER,
};

enum ID_BOUNDARY_TYPE{
    LINE = 0,
    ARC,
};

enum ID_CIRCLE{
    OUTER = 0,
    INNER,
};