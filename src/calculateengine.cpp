#include "calculateengine.h"

void calculate::firstdefine(int *servo1, int *servo2, int *servo3, int *servo4, pidvalue xpid, pidvalue ypid){
    if(xpid.direction){
        *servo1 = xpid.value;
        *servo3 = -(xpid.value);
    }
    else{
        *servo3 = -(xpid.value);
        *servo1 = xpid.value;
    }
}
