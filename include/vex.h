#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"


#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, vex::msec);                                                             \
  } while (!(condition))