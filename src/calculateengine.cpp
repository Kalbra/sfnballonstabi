#include "calculateengine.h"

void calculate::offset(int fromservo1, int fromservo2, int fromservo3, int fromservo4){
    servo1offset1 = fromservo1;
    servo1offset2 = fromservo2;
    servo1offset3 = fromservo3;
    servo1offset4 = fromservo4;
}

void calculate::run(int *servo1, int *servo2, int *servo3, int *servo4, pidvar xpid, pidvar ypid){
    if(xpid.direction){
        *servo1 =   xpid.value  + servo1offset1;
        *servo3 = -(xpid.value) + servo1offset3;
    }
    else{
        *servo3 =   xpid.value  + servo1offset3;
        *servo1 = -(xpid.value)  + servo1offset1;
    }

    if(ypid.direction){
        *servo2 =   ypid.value  + servo1offset2;
        *servo4 = -(ypid.value) + servo1offset4;
    }
    else{
        *servo4 =   ypid.value  + servo1offset4;
        *servo2 = -(ypid.value) + servo1offset2;
    }
}
